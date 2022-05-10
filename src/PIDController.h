#pragma once

#include "datatype.h"

struct PIDController
{
private:
  float last_diff;
  unsigned long last_time;
  float delta_diff;
  float acc_diff;

  float target;

  float kp;
  float ki;
  float kd;
  float gain;

  float intensity;

  PIDController()
      : last_diff(0.0), last_time(0), delta_diff(0.0), acc_diff(0.0), target(0.0), kp(0.0), ki(0.0), kd(0.0), gain(0.0), intensity(0.0)
  {
  }

public:
  static PIDController &getInstance()
  {
    static PIDController instance;
    return instance;
  }

  void setTarget(float target)
  {
    this->target = target;
  }

  void setKi(float ki)
  {
    this->ki = ki;
  }

  void setKd(float kd)
  {
    this->kd = kd;
  }

  void setKp(float kp)
  {
    this->kp = kp;
  }

  void setGain(float gain)
  {
    this->gain = gain;
  }

  void update(const YawPitchRoll &current)
  {
    if (last_time == 0)
    {
      last_time = current.timestamp_ms;
      return;
    }

    const float delta_time_sec = (current.timestamp_ms - last_time) / 1000.0;
    last_time = current.timestamp_ms;

    const float cur_diff = target - current.pitch;
    delta_diff = (cur_diff - last_diff) / delta_time_sec;
    last_diff = cur_diff;

    if (abs(intensity) < 1.0)
    {
      acc_diff += cur_diff * delta_time_sec;
    }
    intensity = gain * (kp * cur_diff + kd * delta_diff + ki * acc_diff);
  }

  float getIntensity()
  {
    return intensity;
  }
};