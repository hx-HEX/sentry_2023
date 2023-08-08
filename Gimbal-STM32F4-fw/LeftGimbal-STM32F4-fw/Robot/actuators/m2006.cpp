#include "m2006.h"
#include "can_interface.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

/**
 * @brief Update the motor speed
 *
 * @param float speed: the input speed of the motor reducer (n/min)
 */
void M2006::SpeedUpdate(float speed)
{
    m_speed_current = speed / m_reduction_ratio;
}



/**
 * @brief Update the motor angle
 *
 * @param uint32_t value: the feedback encoder value
 */
void M2006::AngleUpdate(uint32_t value)
{
    m_encoder->EncodeValueUpdate(value);
    m_angle_current = m_encoder->m_sum_value/ 
                  m_encoder->m_resolution * 360.0f / m_reduction_ratio;
}



/**
 * @brief Update the motor angle
 *
 * @param 
 */
void M2006::SpeedControl(void)
{
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}



/**
 * @brief Update the motor angle
 *
 * @param 
 */
void M2006::AngleControl(void)
{
    m_angle_pid->CalWeakenPID(m_angle_target - m_angle_current);
    m_speed_target = m_angle_pid->m_output;
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}