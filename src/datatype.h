#pragma once

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