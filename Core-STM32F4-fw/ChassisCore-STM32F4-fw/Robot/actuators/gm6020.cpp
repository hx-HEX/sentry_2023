#include "gm6020.h"
#include "can_interface.h"
#include "common_math.h"


/**
 * @brief Update the motor speed
 *
 * @param uint8_t   type: speed feedback type
 *                  0 : encoder original speed feedback
 *                  1 : encoder speed feedback filtered by kalman filter
 *                  2 : external speed feedback
 * 
 * @param float     input: speed value (degree / s)  when "type" is "2"
 *                         plus or minus  when "type" is "0" or "1"
 *               
 */
void GM6020::SpeedUpdate(uint8_t type, float input)
{
    if (type == 0) {
        m_speed_current = input * m_speed_current_encoder;
    } else if (type == 1) {
        m_speed_current = input * m_kalman_filter_speed->GetFilterOutput();
    } else if (type == 2) {
        m_speed_current_external = input;
        m_speed_current = m_speed_current_external;
    }
}



/**
 * @brief Update the motor angle
 *
 * @param uint8_t   type: angle feedback type
 *                  0 : encoder original angle feedback
 *                  1 : encoder angle feedback filtered by kalman filter
 *                  2 : external angle feedback
 * 
 * @param float     input: angle value (degree)  when "type" is "2"
 *                         plus or minus  when "type" is "0" or "1"
 *                          
 */
void GM6020::AngleUpdate(uint8_t type, float input)
{
    if (type == 0) {
        m_angle_current = input * m_angle_current_encoder;
    } else if (type == 1) {
        m_angle_current = input * m_kalman_filter_angle->GetFilterOutput();
    } else if (type == 2) {
        m_angle_current_external = input;
        m_angle_current = m_angle_current_external;
    }
}



/**
 * @brief Update the motor speed measured by encoder
 *
 * @param float speed: the input speed of the motor reducer (degree / s)
 */
void GM6020::EncoderSpeedUpdate(float speed)
{
    m_speed_current_encoder = speed / m_reduction_ratio / 60 * 2 * PI * RADIAN2DEGREE_VALUE;
    m_kalman_filter_speed->UpdateFilter(m_speed_current_encoder);
    m_speed_current_encoder_filter = m_kalman_filter_speed->GetFilterOutput();
}



/**
 * @brief Update the motor angle measured by encoder
 *
 * @param uint32_t value: the feedback encoder value
 */
void GM6020::EncoderAngleUpdate(uint32_t value)
{
    m_encoder->EncodeValueUpdate(value);
    m_angle_current_encoder = (m_encoder->m_sum_value - m_encoder->m_zero_value) / 
    m_encoder->m_resolution * 360.0f / m_reduction_ratio;
    m_kalman_filter_angle->UpdateFilter(m_angle_current_encoder);
    m_angle_current_encoder_filter = m_kalman_filter_angle->GetFilterOutput();
}



/**
 * @brief Update the motor angle
 *
 * @param 
 */
void GM6020::SpeedControl(void)
{
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}



/**
 * @brief Update the motor angle
 *
 * @param 
 */
void GM6020::AngleControl(void)
{
    m_angle_td->CalAdrcTD(m_angle_target);
    m_angle_pid->CalWeakenPID(m_angle_td->m_x1 - m_angle_current);
    m_speed_target = m_angle_pid->m_output + m_angle_td->m_k * m_angle_td->m_x2;
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}
