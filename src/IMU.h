// derived from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino
#pragma once

#include "MPU6050_6Axis_MotionApps20.h"
#include "debug.h"
#include "datatype.h"

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
        mpu.setZAccelOffset(544);
        mpu.setXGyroOffset(186);
        mpu.setYGyroOffset(-65);
        mpu.setZGyroOffset(-28);

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
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];
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
