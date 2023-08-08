#include "m3508.h"
#include "can_interface.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

/**
 * @brief Update the motor speed
 *
 * @param float speed: the input speed of the motor reducer (rad / s)
 */
void M3508::SpeedUpdate(float speed)
{
    m_speed_current = speed / m_reduction_ratio / 60 * 2 * PI;
}



/**
 * @brief Update the motor angle
 *
 * @param uint32_t value: the feedback encoder value
 */
void M3508::AngleUpdate(uint32_t value)
{
    m_encoder->EncodeValueUpdate(value);
    m_angle_current = m_encoder->m_sum_value/ 
                  m_encoder->m_resolution * 2 * PI / m_reduction_ratio;
}



/**
 * @brief Update the motor angle
 *
 * @param 
 */
void M3508::SpeedControl(void)
{
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}



/**
 * @brief Update the motor angle
 *
 * @param 
 */
void M3508::AngleControl(void)
{
    m_angle_td->CalAdrcTD(m_angle_target);
    m_angle_pid->CalWeakenPID(m_angle_td->m_x1 - m_angle_current);
    m_speed_target = m_angle_pid->m_output;
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}