#include "mahony.h"
#include "math.h"
#include "common_math.h"



/**
*@brief Construct Mahony filter 
* 
*@param float dt: IMU sampling time gap
*/
Mahony::Mahony(float dt, float kp, float ki)
{
    m_dt = dt;
    m_acc_error_kp = kp;
    m_acc_error_ki = ki;

    m_quaternion.q0 = 1.0;
    m_quaternion.q1 = 0.0;
    m_quaternion.q2 = 0.0;
    m_quaternion.q3 = 0.0;

    m_acc_error_integral_x = 0.0;
    m_acc_error_integral_y = 0.0;
    m_acc_error_integral_z = 0.0;

    m_com_gyro.x = 0.0;
    m_com_gyro.y = 0.0;
    m_com_gyro.z = 0.0;

    m_esti_acc.x = 0.0;
    m_esti_acc.y = 0.0;
    m_esti_acc.z = 0.0;

    m_eular_angle.pitch = 0.0;
    m_eular_angle.yaw = 0.0;
    m_eular_angle.roll = 0.0;

    m_first_in_flag = true;
}



/**
*@brief Mahony filter to update the current angle
* 
*@param 
*/
void Mahony::AngleUpdate(const float acc[3], const float gyro[3])
{
    float acc_norm_val;
    float quaternion_norm_val;
    float real_acc_x, real_acc_y, real_acc_z;
    float error_acc_x, error_acc_y, error_acc_z;
    float q0_last = m_quaternion.q0;
    float q1_last = m_quaternion.q1;
    float q2_last = m_quaternion.q2;
    float q3_last = m_quaternion.q3;
    float q0_new, q1_new, q2_new, q3_new;

    // Compensate the anglur velocity only if accelerometer measurement valid 
    // (avoids NaN in accelerometer normalisation)
    if (!m_first_in_flag && ((acc[0] != 0.0f) || (acc[1] != 0.0f) || (acc[2] != 0.0f))) {
        // Normalise accelerometer measurement
        acc_norm_val = invSqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        real_acc_x = acc[0] * acc_norm_val;
        real_acc_y = acc[1] * acc_norm_val;
        real_acc_z = acc[2] * acc_norm_val;

        // Calculate error between estimated and measured direction of acceleration (cross product)
        error_acc_x = real_acc_y * m_esti_acc.z - real_acc_z * m_esti_acc.y;
        error_acc_y = real_acc_z * m_esti_acc.x - real_acc_x * m_esti_acc.z;
        error_acc_z = real_acc_x * m_esti_acc.y - real_acc_y * m_esti_acc.x;

        // Calculate error integral
        m_acc_error_integral_x += error_acc_x * m_dt;
        m_acc_error_integral_y += error_acc_y * m_dt;
        m_acc_error_integral_z += error_acc_z * m_dt;

        // Compensate the angular velocity (PI controller)
        m_com_gyro.x = gyro[0] + 
        m_acc_error_kp * error_acc_x + m_acc_error_ki * m_acc_error_integral_x;
        m_com_gyro.y = gyro[1] + 
        m_acc_error_kp * error_acc_y + m_acc_error_ki * m_acc_error_integral_y;
        m_com_gyro.z = gyro[2] + 
        m_acc_error_kp * error_acc_z + m_acc_error_ki * m_acc_error_integral_z;
    } else {
        m_com_gyro.x = gyro[0];
        m_com_gyro.y = gyro[1];
        m_com_gyro.z = gyro[2];
    }

    // Calculate the new quaternion (use compensated angular velocity)
    q0_new = q0_last + 
    0.5 * m_dt * (-q1_last * m_com_gyro.x - q2_last * m_com_gyro.y - q3_last * m_com_gyro.z);
    q1_new = q1_last + 
    0.5 * m_dt * (q0_last * m_com_gyro.x - q3_last * m_com_gyro.y + q2_last * m_com_gyro.z);
    q2_new = q2_last +
    0.5 * m_dt * (q3_last * m_com_gyro.x + q0_last * m_com_gyro.y - q1_last * m_com_gyro.z);
    q3_new = q3_last +
    0.5 * m_dt * (-q2_last * m_com_gyro.x + q1_last * m_com_gyro.y + q0_last * m_com_gyro.z);

    // Normalize and update the quaternion
    quaternion_norm_val = invSqrt(q0_new * q0_new + q1_new * q1_new 
    + q2_new * q2_new + q3_new * q3_new);
    q0_new *= quaternion_norm_val;
    q1_new *= quaternion_norm_val;
    q2_new *= quaternion_norm_val;
    q3_new *= quaternion_norm_val;
    m_quaternion.q0 = q0_new;
    m_quaternion.q1 = q1_new;
    m_quaternion.q2 = q2_new;
    m_quaternion.q3 = q3_new;

    // Calculate some temporary variables (for calculation optimization)
    float calcu_temp1 = q1_new * q3_new - q0_new * q2_new;
    float calcu_temp2 = q2_new * q3_new + q0_new * q1_new;
    float calcu_temp3 = q1_new * q1_new + q2_new * q2_new;
    float calcu_temp4 = q0_new * q3_new + q1_new * q2_new;
    float calcu_temp5 = 1 - 2 * (q2_new * q2_new + q3_new * q3_new);

    // Update the estimated acceleration
    m_esti_acc.x = 2 * calcu_temp1;
    m_esti_acc.y = 2 * calcu_temp2;
    m_esti_acc.z = 1 - 2 * calcu_temp3;
    
    // Update current euler angles (use updated quaternion)
    m_eular_angle.pitch = -asinf(m_esti_acc.x) * RADIANODEGREES;
    m_eular_angle.roll = atan2f(calcu_temp2, m_esti_acc.z) * RADIANODEGREES;
    m_eular_angle.yaw = atan2f(2 * calcu_temp4, calcu_temp5) * RADIANODEGREES;

    m_first_in_flag = false;
}
