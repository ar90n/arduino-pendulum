#pragma once

#include "debug.h"

template <int FORWARD_PIN, int BACKWARD_PIN, int PWM_PIN>
struct MotorDriver
{
private:
  uint8_t offset;

  MotorDriver()
  {
    set_offset(0);
    drive(0.0);
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

  void set_offset(uint8_t offset)
  {
    this->offset = offset;
  }

  void drive(float intensity)
  {
    const float sat_intensity = max(-1.0, min(intensity, 1.0));

    if (0 < sat_intensity)
    {
      this->forward();
    }
    else if (sat_intensity < 0)
    {
      this->backward();
    }
    else
    {
      this->stop();
    }

    const uint8_t pwd_reg_value = abs(sat_intensity) * (255 - offset) + offset;
    analogWrite(PWM_PIN, pwd_reg_value);
  }
};