#include "common_math.h"


/**
*@brief     symbolic function
*
*@param     pNumber original value
*
*@return    float 0 or 1 or -1
*/
float Sgn(float fpNumber)
{
	if(fpNumber > 0)
	{
		return 1;
	}
	else if(fpNumber < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}



/**
*@brief clipping function, remove the value beyond the maximum and minimum value, and replace it with the maximum or minimum value
*
*@param value: input data
*@param min: clipping minimum value
*@param max: clipping maximum
*
*@return clipped data
*/
float Clip(float value, float min, float max)
{
	float ret;
	if(value < min)
	{
		ret = min;
	}
	else if(value > max)
	{
		ret = max;
	}
	else
	{
		ret = value;
	}
	return ret;
}



/**
*@brief Fast inverse square-root
* 
*@param 
*/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
 * @brief safe atan2, deal with x == 0 or y == 0
 *
 * @param y
 * @param x
 * @return float
 */
float safeAtan2(float y, float x)
{
	if (x == 0 && y == 0)
	{
		return 0.0 / RADIAN2DEGREE_VALUE;
	}
	else if (x == 0 && y > 0)
	{
		return 90.0 / RADIAN2DEGREE_VALUE;
	}
	else if (x == 0 && y < 0)
	{
		return -90.0 / RADIAN2DEGREE_VALUE;
	}
	else if (x > 0 && y == 0)
	{
		return 0.0 / RADIAN2DEGREE_VALUE;
	}
	else if (x < 0 && y == 0)
	{
		return 180.0 / RADIAN2DEGREE_VALUE;
	}
	else
	{
		return atan2(y, x);
	}
}