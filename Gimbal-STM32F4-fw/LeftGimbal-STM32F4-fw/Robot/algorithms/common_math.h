#pragma once

#include "math.h"

#define PI                                                  ( (float) (3.14159265358979) )
#define RADIAN2DEGREE_VALUE                                 ( (float) (57.29577) )

float Sgn(float fpNumber);
float Clip(float value, float min, float max);
float invSqrt(float x);
float safeAtan2(float y,float x);
