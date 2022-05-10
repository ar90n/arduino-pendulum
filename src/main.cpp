#include <Arduino.h>
#include <Wire.h>

#include "debug.h"
#include "IMU.h"
#include "MotorDriver.h"
#include "PIDController.h"

template <>
volatile bool IMU<2>::mpuInterrupt = false;

struct ConfigurableParams
{
private:
  float kp;
  float ki;
  float kd;
  float target;
  float gain;
  uint8_t offset;
  bool verbose;

public:
  ConfigurableParams(float kp, float ki, float kd, float target, float gain, uint8_t offset, bool verbose) : kp(kp), ki(ki), kd(kd), target(target), gain(gain), offset(offset), verbose(false) {}

  bool readSerial()
  {
    bool hasInput = false;
    if (Serial.available() > 0)
    {
      hasInput = true;
      int incomingByte = Serial.read();
      switch (incomingByte)
      {
      case 'q':
      {
        kp += 0.1;
      }
      break;
      case 'a':
      {
        kp -= 0.1;
      }
      break;
      case 'w':
      {
        ki += 0.1;
      }
      break;
      case 's':
      {
        ki -= 0.1;
      }
      break;
      case 'e':
      {
        kd += 0.01;
      }
      break;
      case 'd':
      {
        kd -= 0.01;
      }
      break;
      case 'r':
      {
        target += 0.01;
      }
      break;
      case 'f':
      {
        target -= 0.01;
      }
      break;
      case 't':
      {
        gain += 0.01;
      }
      break;
      case 'g':
      {
        gain -= 0.01;
      }
      break;
      case 'y':
      {
        offset += 1;
      }
      break;
      case 'h':
      {
        offset -= 1;
      }
      break;
      case 'v':
      {
        verbose = !verbose;
      }
      break;
      default:
      {
        hasInput = false;
      }
      break;
      }
    }

    return hasInput;
  }

  void print()
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

  float getKp() { return kp; }
  float getKi() { return ki; }
  float getKd() { return kd; }
  float getTarget() { return target; }
  float getGain() { return gain; }
  uint8_t getOffset() { return offset; }
  bool getVerbose() { return verbose; }
};

struct Duration {
private:
  unsigned long start;

public:
  Duration() : start(0) {}

  unsigned long getElapsed()
  {
    const auto end = millis();
    const auto elapsed = end - start;
    start = end;
    return elapsed;
  }
};

Duration duration;
ConfigurableParams params(2.00, 17.00, 0.25, 0.5, 0.70, 40, false);

void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  PIDController &controller = PIDController::getInstance();
  controller.setTarget(params.getTarget());
  controller.setGain(params.getGain());
  controller.setKp(params.getKp());
  controller.setKi(params.getKi());
  controller.setKd(params.getKd());

  MotorDriver<4, 5, 6> &motor = MotorDriver<4, 5, 6>::getInstance();
  motor.set_offset(params.getOffset());

  IMU<2> &mpu = IMU<2>::getInstance();
  mpu.setup();

  duration.getElapsed();
}

unsigned long pre;

void loop()
{
  PIDController &controller = PIDController::getInstance();
  MotorDriver<4, 5, 6> &motor = MotorDriver<4, 5, 6>::getInstance();
  IMU<2> &mpu = IMU<2>::getInstance();

  if (!mpu.fetchDMPData())
  {
    return;
  }

  if (params.readSerial())
  {
    controller.setTarget(params.getTarget());
    controller.setGain(params.getGain());
    controller.setKp(params.getKp());
    controller.setKi(params.getKi());
    controller.setKd(params.getKd());
    motor.set_offset(params.getOffset());
  }

  const auto yaw_pitch_roll = mpu.getYawPitchRoll();
  controller.update(yaw_pitch_roll);
  const auto intensity = controller.getIntensity();
  motor.drive(intensity);

  if(params.getVerbose()){
    params.print();
    PRINT("> ");
    PRINT(duration.getElapsed());
    PRINT(" ");
    PRINT(yaw_pitch_roll.pitch);
    PRINT(" ");
    PRINT(intensity);
    PRINT("\n");
  }
}