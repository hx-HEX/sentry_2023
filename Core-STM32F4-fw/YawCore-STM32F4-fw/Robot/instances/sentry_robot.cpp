#include "user_global.h"
#include "sentry_robot.h"
#include "can.h"
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "pid.h"
#include "adrc.h"
#include "encoder.h"
#include "common_math.h"




/**
*@brief construct sentry robot
* 
*@param 
*/
SentryRobot::SentryRobot(void)
{
    chassis_mode = CHASSIS_SAFE;
    chassis_mode_pre = CHASSIS_SAFE;

    gimbal_mode = GIMBAL_SAFE;
    gimbal_mode_pre = GIMBAL_SAFE;

    shoot_mode = SHOOT_SAFE;
    shoot_mode_pre = SHOOT_SAFE;

}



/**
*@brief Initial the whole Sentry robot
* 
*@param 
*/
void SentryRobot::
Init(void)
{
    // Initial all actuators
    InitAllActuators();

    // Initial all Sensors
    InitAllSensors();
}



/**
*@brief Initial all actuators of the Sentry robot
* 
*@param 
*/
void SentryRobot::InitAllActuators(void)
{

    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR] = new M2006(CAN1, LEFT_SHOOT_DRIVE_MOTOR_ID, SHOOT_MOTOR_REDUCTION_RATIO);
    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR]->m_angle_td = new Adrc_TD(10000, 0.001, 0.001,0);
    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR]->m_angle_pid = new Pid(0.010, 0.00, 0, 10, 10000, 10000, 0, 0);
    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR]->m_speed_pid = new Pid(700, 0.00, 0, 10, 10000, 10000, 0, 0);
    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR]->m_encoder = new AbsEncoder(0, ENCODER_RESOLUTION);   
    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);

    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR] = new M2006(CAN1, RIGHT_SHOOT_DRIVE_MOTOR_ID, SHOOT_MOTOR_REDUCTION_RATIO);
    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR]->m_angle_td = new Adrc_TD(10000, 0.001, 0.001,0);
    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR]->m_angle_pid = new Pid(0.010, 0.00, 0, 10, 10000, 10000, 0, 0);
    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR]->m_speed_pid = new Pid(700, 0.00, 0, 10, 10000, 10000, 0, 0);
    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR]->m_encoder = new AbsEncoder(0, ENCODER_RESOLUTION);   
    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);


}



/**
 *@brief Initial all sensors of the Sentry robot
 * 
 *@param 
*/
void SentryRobot::InitAllSensors(void)
{
    photogate = new Photogate();
}



/**
 *@brief execute the sentry robot shoot control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteShootAlgorithm(void)
{
    if (shoot_mode != SHOOT_SAFE) {
        for(uint8_t i = 0; i < SHOOT_MOTOR_NUM; i++)
        {
            shoot_motor[i]->AngleControl();
        }
    } else {
        for(uint8_t i = 0; i < SHOOT_MOTOR_NUM; i++)
        {
            shoot_motor[i]->m_speed_pid->m_output = 0;
        }
    }
}



/**
 *@brief Send control command to all actuators
 * 
 *@param 
*/
void SentryRobot::SendControlCommand(void)
{
    static uint16_t trans_cnt = 0;//待上电后电平稳定再执行发送函数

    // CAN1 ID 0x200    CAN2 ID 0x200
    can1_context.CANx_TxMsg.StdId = 0x200;
    can2_context.CANx_TxMsg.StdId = 0x200;
    for (int i = 0; i < 8; i++) {
        can1_context.CANx_TxMsg.Data[i] = 0;
        can2_context.CANx_TxMsg.Data[i] = 0;
    }
    for(uint8_t i = 0; i < SHOOT_MOTOR_NUM; i++)
    {
        uint32_t id = shoot_motor[i]->m_id;
        int16_t cmd = (int16_t)shoot_motor[i]->m_speed_pid->m_output;
        if (id >= 1 && id <= 4) {
            if (shoot_motor[i]->m_CANx == CAN1) {
                can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            } else if (shoot_motor[i]->m_CANx == CAN2) {
                can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        } else if (id >= 5 && id <= 8){
                if (shoot_motor[i]->m_CANx == CAN1) {
                can1_context.CANx_TxMsg.Data[(id - 5) * 2] = (uint8_t)(cmd >> 8);
                can1_context.CANx_TxMsg.Data[(id - 5) * 2 + 1] = (uint8_t)(cmd);
            } else if (shoot_motor[i]->m_CANx == CAN2) {
                can2_context.CANx_TxMsg.Data[(id - 5) * 2] = (uint8_t)(cmd >> 8);
                can2_context.CANx_TxMsg.Data[(id - 5) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    } 
    if(trans_cnt > 2000){
        can1_context.CanSendMessage();
        can2_context.CanSendMessage();
    }
  
    // CAN1 ID 0x1ff    CAN2 ID 0x1ff
    can1_context.CANx_TxMsg.StdId = 0x1ff;
    can2_context.CANx_TxMsg.StdId = 0x1ff;
    for (int i = 0; i < 8; i++) {
        can1_context.CANx_TxMsg.Data[i] = 0;
        can2_context.CANx_TxMsg.Data[i] = 0;
    }

    if(trans_cnt > 2000){
        can1_context.CanSendMessage();
        can2_context.CanSendMessage();
    }

    trans_cnt++;
    if(trans_cnt > 2500) trans_cnt=2500;
}



/**
 *@brief Increase shoot bullet count target
 * 
 *@param 
*/
void SentryRobot::IncreaseLeftShootBulletTarget(uint32_t time_now) {
        if (time_now - m_left_shoot_bullet_last_increase_time > (1000 / m_left_shoot_bullet_fps)) {
            m_left_shoot_bullet_cnt_target++;
            m_left_shoot_bullet_last_increase_time = time_now;
        }
}



/**
 *@brief Increase shoot bullet count target
 * 
 *@param 
*/
void SentryRobot::IncreaseRightShootBulletTarget(uint32_t time_now) {
        if (time_now - m_right_shoot_bullet_last_increase_time > (1000 / m_right_shoot_bullet_fps)) {
            m_right_shoot_bullet_cnt_target++;
            m_right_shoot_bullet_last_increase_time = time_now;
        }
}



/**
 *@brief Set the shoot drive motor position target
 * 
 *@param 
*/
void SentryRobot::SetShootDriverTarget(float left_angle,float right_angle) {
    shoot_motor[LEFT_SHOOT_DRIVE_MOTOR]->m_angle_target = left_angle;
    shoot_motor[RIGHT_SHOOT_DRIVE_MOTOR]->m_angle_target = right_angle;
}