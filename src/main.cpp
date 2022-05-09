#include <Arduino.h>
#include <Wire.h>

#include "I2Cdev.h"

#define DEBUG

#ifdef DEBUG
#define PRINT(str) Serial.print(str)
#else
#define PRINT(str)
#endif

//float kp = 0.5;
//float ki =8.5;
//float kd = 0.07;
//float target = 0.49;
//uint8_t offset = 50;
float kp = 2.00;
float ki =17.00;
float kd = 0.25;
float target = 0.49;
float gain = 0.70;
uint8_t offset = 40;
bool fff = false;

struct YawPitchRoll
{
  float yaw;
  float pitch;
  float roll;
  unsigned long timestamp_ms;
};

struct Motion6
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  unsigned long timestamp_ms;
};

template <int INTERRUPT_PIN>
struct IMU
{
private:
  static volatile bool mpuInterrupt;

  MPU6050 mpu;
  bool dmpReady;          // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  IMU()
      : dmpReady(false)
  {
  }

  static void dmpDataReady()
  {
    mpuInterrupt = true;
    digitalWrite(7, HIGH);
  }

  void initializeDevice()
  {
    PRINT(F("Initializing I2C devices...\n"));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
  }

  void checkConnection()
  {
    PRINT(F("Testing device connections...\n"));
    const auto is_connected = mpu.testConnection();
    const auto message = is_connected ? F("Connected!\n") : F("Not connected!\n");
    PRINT(message);
  }

  bool initializeDMP()
  {
    PRINT(F("Initializing DMP...\n"));
    return mpu.dmpInitialize() == 0;
  }

  void calibrate()
  {
    mpu.setXAccelOffset(837);
    mpu.setYAccelOffset(-939);
    mpu.setZAccelOffset(544); // 1688 factory default for my test chip
    mpu.setXGyroOffset(186);
    mpu.setYGyroOffset(-65);
    mpu.setZGyroOffset(-28);

    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
  }

  void enableDMP()
  {
    // turn on the DMP, now that it's ready
    PRINT(F("Enabling DMP...\n"));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    PRINT(F("Enabling interrupt detection (Arduino external interrupt "));
    PRINT(digitalPinToInterrupt(INTERRUPT_PIN));
    PRINT(F(")...\n"));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, FALLING);
    mpuIntStatus = mpu.getIntStatus();
  }

public:
  static IMU &getInstance()
  {
    static IMU instance;
    return instance;
  }

  void setup()
  {
    initializeDevice();
    checkConnection();
    const auto devStatus = initializeDMP();
    if (!devStatus)
    {
      PRINT(F("DMP Initialization failed (code "));
      PRINT(devStatus);
      PRINT(F(")\n"));
      return;
    }

    calibrate();
    enableDMP();

    PRINT(F("DMP ready! Waiting for first interrupt...\n"));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  bool isReady()
  {
    return dmpReady;
  }

  bool fetchDMPData()
  {
    if (!mpuInterrupt)
    {
      return false;
    }
    mpuInterrupt = false;
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    return true;
  }

  YawPitchRoll getYawPitchRoll()
  {
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorFloat gravity; // [x, y, z]         gravity vector
    float ypr[3];        // [psi, theta, phi]    Euler angle container
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    return YawPitchRoll{
        (ypr[0] / float(M_PI)),
        (ypr[1] / float(M_PI)),
        (ypr[2] / float(M_PI)),
        millis()};
  }

  Motion6 getMotion6()
  {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    return Motion6{
        ax / float(16384.0),
        ay / float(16384.0),
        az / float(16384.0),
        gx / float(131.0),
        gy / float(131.0),
        gz / float(131.0),
        millis(),
    };
  }
};

template <int FORWARD_PIN, int BACKWARD_PIN, int PWM_PIN>
struct MotorDriver
{
private:
  MotorDriver()
  {
    this->set_intensity(0.0);
  }

public:
  static MotorDriver &getInstance()
  {
    static MotorDriver instance;
    return instance;
  }

  void stop()
  {
    pinMode(FORWARD_PIN, OUTPUT);
    digitalWrite(FORWARD_PIN, LOW);

    pinMode(BACKWARD_PIN, OUTPUT);
    digitalWrite(BACKWARD_PIN, LOW);
  }

  void forward()
  {
    pinMode(FORWARD_PIN, INPUT);
    pinMode(BACKWARD_PIN, OUTPUT);
    digitalWrite(BACKWARD_PIN, LOW);
  }

  void backward()
  {
    pinMode(FORWARD_PIN, OUTPUT);
    digitalWrite(FORWARD_PIN, LOW);
    pinMode(BACKWARD_PIN, INPUT);
  }

  void set_intensity(float intensity)
  {
    if (0 < intensity)
    {
      this->forward();
    }
    else if (intensity < 0)
    {
      this->backward();
    }
    else
    {
      this->stop();
    }

    intensity = max(-1.0, min(intensity, 1.0));
    analogWrite(PWM_PIN, uint8_t(abs(intensity) * 210 + offset));
  }
};

struct PendulumController
{
private:
  float target;
  float acc;
  unsigned long last_time;
  float last;
  unsigned long current_time;
  float current;

  PendulumController()
      : target(0.5), last(0.5), current(0.5), last_time(0), current_time(0)
  {
  }

public:
  static PendulumController &getInstance()
  {
    static PendulumController instance;
    return instance;
  }

  void set_target(float target)
  {
    this->target = target;
  }

  void update(const YawPitchRoll &current)
  {
    this->last_time = this->current_time;
    this->current = current.pitch;
    this->current_time = current.timestamp_ms;
  }

  float calc_intensity()
  {
    float diff = (this->target - this->current);
    const float delta_time = (this->current_time - this->last_time) / 1000.0;
    const float delta = (diff - this->last) / delta_time;
    this->acc += diff * delta_time;
    this->last = diff;

    // const float ki = 15.3;
    // const float kd = 0.23;
    // const float kd = 0.43;
    //  auto v = 15.0 * diff + 60.0 * delta + 3.0 * acc;
    auto v = kp * diff + kd * delta + ki * acc;

    if (1.0 < v)
    {
      this->acc -= diff * delta_time;
      v = kp * diff + kd * delta + ki * acc;
    }
    // v = 3.0 * diff + 2.0 * delta + ki * acc;
    v *= gain;
    // PRINT(delta);
    // PRINT("\n");
    // PRINT(diff);
    // PRINT("\n");

    // if(-0.05 < v && v < 0.0) {
    //   v = -0.05;
    // }
    // if(0 < v && v < 0.05  ) {
    //   v = 0.05;
    // }

    return v;
  }
};

template <>
volatile bool IMU<2>::mpuInterrupt = false;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  IMU<2>::getInstance().setup();
  // PendulumController::getInstance().set_target(0.495);
  // PendulumController::getInstance().set_target(0.55);
  // PendulumController::getInstance().set_target(0.530);
  // MadgwickFilter.begin(100); // Madgwickフィルタの周波数を100Hzに設定。
  MadgwickFilter.begin(10); // Madgwickフィルタの周波数を100Hzに設定。
}

unsigned long pre;

void loop()
{
  if (Serial.available() > 0)
  {
    // 受信したデータの1バイトを読み取る
    int incomingByte = Serial.read();
    if (incomingByte == 'q')
    {
      kp += 0.1;
    }
    if (incomingByte == 'a')
    {
      kp -= 0.1;
    }
    if (incomingByte == 'w')
    {
      ki += 0.1;
    }
    if (incomingByte == 's')
    {
      ki -= 0.1;
    }
    if (incomingByte == 'e')
    {
      kd += 0.01;
    }
    if (incomingByte == 'd')
    {
      kd -= 0.01;
    }
    if (incomingByte == 'r')
    {
      target += 0.01;
    }
    if (incomingByte == 'f')
    {
      target -= 0.01;
    }
    if (incomingByte == 't')
    {
      offset += 1;
    }
    if (incomingByte == 'g')
    {
      offset -= 1;
    }
    if (incomingByte == 'y')
    {
      gain += 0.01;
    }
    if (incomingByte == 'h')
    {
      gain -= 0.01;
    }
    if (incomingByte == 'n')
    {
      fff = !fff;
    }
  }
  if (fff)
  {
    PRINT(kp);
    PRINT(" ");
    PRINT(ki);
    PRINT(" ");
    PRINT(kd);
    PRINT(" ");
    PRINT(target);
    PRINT(" ");
    PRINT(gain);
    PRINT(" ");
    PRINT(offset);
    PRINT("\n");
  }

  PendulumController::getInstance().set_target(target);

  IMU<2> &mpu = IMU<2>::getInstance();
  if (!mpu.fetchDMPData())
  {
    return;
  }
  const auto yaw_pitch_roll = mpu.getYawPitchRoll();

  auto cur = millis();
  PRINT("> ");
  PRINT((cur - pre));
  PRINT(" ");
  pre = cur;

  PRINT(yaw_pitch_roll.pitch);
  PRINT(" ");
  PendulumController &controller = PendulumController::getInstance();
  controller.update(yaw_pitch_roll);
  const auto intensity = controller.calc_intensity();
  PRINT(intensity);

  MotorDriver<4, 5, 6>::getInstance().set_intensity(intensity);
  PRINT("\n");
}