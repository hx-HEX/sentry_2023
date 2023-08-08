#include "sentry_robot.h"
#include "can.h"
#include "m3508.h"
#include "gm6020.h"
#include "pid.h"
#include "adrc.h"
#include "encoder.h"
#include "mahony.h"
#include "kalman.h"
#include "smc.h"
#include "user_global.h"
#include "can_interface.h"


/**
*@brief Initial the whole Sentry robot
* 
*@param 
*/
void SentryRobot::Init(void)
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
    shoot_motor[LEFT_FRIC_MOTOR] = new M3508(CAN1, LEFT_FRIC_MOTOR_ID,  FRIC_MOTOR_REDUCTION_RATIO);
    shoot_motor[LEFT_FRIC_MOTOR]->m_smc = new Smc(4.4936132, 8000, 20, 20, 16200);
    shoot_motor[LEFT_FRIC_MOTOR]->m_smc->m_TD = new Adrc_TD((float)5000, 0.001, 0.001,0);
    shoot_motor[LEFT_FRIC_MOTOR]->m_encoder = new AbsEncoder(LEFT_FRIC_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);  
    
    shoot_motor[RIGHT_FRIC_MOTOR] = new M3508(CAN1, RIGHT_FRIC_MOTOR_ID,  FRIC_MOTOR_REDUCTION_RATIO);
    shoot_motor[RIGHT_FRIC_MOTOR]->m_smc = new Smc(4.4936132, 8000, 20, 20, 16200);
    shoot_motor[RIGHT_FRIC_MOTOR]->m_smc->m_TD = new Adrc_TD((float)5000, 0.001, 0.001,0);    
    shoot_motor[RIGHT_FRIC_MOTOR]->m_encoder = new AbsEncoder(RIGHT_FRIC_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);  

    gimbal_motor[GIMBAL_PITCH_MOTOR] = new GM6020(CAN1, GIMBAL_PITCH_MOTOR_ID, GIMBAL_MOTOR_REDUCTION_RATIO);
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_angle_td = new Adrc_TD((float)20000, 0.01, 0.01,0.8);
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_angle_pid = new Pid(50, 0.2, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_speed_pid = new Pid(150, 0.00, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_encoder = new AbsEncoder(GIMBAL_PITCH_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);

    gimbal_motor[GIMBAL_YAW_MOTOR] = new GM6020(CAN1, GIMBAL_YAW_MOTOR_ID, GIMBAL_MOTOR_REDUCTION_RATIO);
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_angle_td = new Adrc_TD((float)20000, 0.01, 0.01,0.8);
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_angle_pid = new Pid(50, 0.08, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_speed_pid = new Pid(150, 0.01, 0, 10, 20000, 20000, 5000, 2000);
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_encoder = new AbsEncoder(GIMBAL_YAW_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.001f, 0.1f);
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);
}



/**
 *@brief Initial all sensors of the Sentry robot
 * 
 *@param 
*/
void SentryRobot::InitAllSensors(void)
{
    gimbal_imu[GIMBAL_FIRST_IMU] = new BMI088();
    int imu_flag = -1;
    do {
        imu_flag = gimbal_imu[GIMBAL_FIRST_IMU]->Init(IMU_CALIBRATE);
    } while (imu_flag != 0);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_mahony_filter = new Mahony(0.001f, 0.5f, 0.001f);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_x = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z = new Kalman(1, 0.001f, 0.0001f,0.08f, 0.01f);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_y = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
}



/**
 *@brief control the shoot motor to move 
 * 
 *@param 
*/
void SentryRobot::MoveShoot(void)
{
    if(shoot_mode != SHOOT_SAFE){
        for (int i = 0; i < SHOOT_MOTOR_NUM; i++) {
            shoot_motor[i]->m_smc->CalSMC();
        }
    }
}



/**
 *@brief control the gimbal to move
 * 
 *@param 
*/
void SentryRobot::MoveGimbal(void)
{
    if (gimbal_mode != GIMBAL_SAFE) {
        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++) {
            gimbal_motor[i]->AngleControl();
        }
    } 
}



/**
 *@brief Set the sentry robot first gimbal pitch to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetGimbalAngleTarget(float target_p , float target_y)
{
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_angle_target = target_p;
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_angle_target = target_y;
}



/**
 *@brief Set the sentry robot chassis steer angle to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetFricWheelSpeedTarget(float l_target, float r_target)
{
    shoot_motor[LEFT_FRIC_MOTOR]->m_smc->m_fpDes = l_target;
    shoot_motor[RIGHT_FRIC_MOTOR]->m_smc->m_fpDes = r_target;
}



void SentryRobot::UpdateGimbalPitchState(float angle,float speed)
{
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_angle_current = angle;
    gimbal_motor[GIMBAL_PITCH_MOTOR]->m_speed_current = speed;
}



void SentryRobot::UpdateGimbalYawState(float angle,float speed)
{
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_angle_current = angle;
    gimbal_motor[GIMBAL_YAW_MOTOR]->m_speed_current = speed;
}



void SentryRobot::UpdateFricWheelState(float l_speed,float r_speed)
{
    shoot_motor[LEFT_FRIC_MOTOR]->m_smc->m_fpFB = l_speed;
    shoot_motor[RIGHT_FRIC_MOTOR]->m_smc->m_fpFB = r_speed;
}



/**
 *@brief execute the sentry robot chassis control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteShootAlgorithm(void)
{
    if(shoot_mode != SHOOT_SAFE)
    {
        for (uint8_t i = 0; i < SHOOT_MOTOR_NUM; i++)
            shoot_motor[i]->m_smc->CalSMC();
    }
}



/**
 *@brief execute the sentry robot gimbal control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteGimbalAlgorithm(void)
{
    if (gimbal_mode != GIMBAL_SAFE) 
    {
        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++)
            gimbal_motor[i]->AngleControl();
    }
}



/**
 *@brief Send control command to all actuators
 * 
 *@param 
*/
void SentryRobot::SendControlCommand(void)
{
    uint8_t transmint_1;
    uint8_t transmint_2;

    // CAN1 ID 0x200    CAN2 ID 0x200
    can1_context.CANx_TxMsg.StdId = 0x200;
    can2_context.CANx_TxMsg.StdId = 0x200;
    for (int i = 0; i < 8; i++) {
        can1_context.CANx_TxMsg.Data[i] = 0;
        can2_context.CANx_TxMsg.Data[i] = 0;
    }

    for (int i = 0; i < SHOOT_MOTOR_NUM; i++) {
    uint32_t id = shoot_motor[i]->m_id;
    int16_t cmd = (int16_t)shoot_motor[i]->m_smc->m_fpU;
    if (id >= 1 && id <= 4) {
        if (shoot_motor[i]->m_CANx == CAN1) {
            can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
            can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
        } else if (shoot_motor[i]->m_CANx == CAN2) {
            can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
            can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    }

    transmint_1 = can1_context.CanSendMessage();
    transmint_2 = can2_context.CanSendMessage();

    if(transmint_1 == CAN_TxStatus_NoMailBox)
    {   
        can1_context.CANx->TSR|= CAN_TSR_ABRQ0;
        can1_context.CANx->TSR|= CAN_TSR_ABRQ1;
        can1_context.CANx->TSR|= CAN_TSR_ABRQ2;
    }
    if(transmint_2 == CAN_TxStatus_NoMailBox)
    {   
        can2_context.CANx->TSR|= CAN_TSR_ABRQ0;
        can2_context.CANx->TSR|= CAN_TSR_ABRQ1;
        can2_context.CANx->TSR|= CAN_TSR_ABRQ2;
    }



    // CAN1 ID 0x1ff    CAN2 ID 0x1ff
    can1_context.CANx_TxMsg.StdId = 0x1ff;
    can2_context.CANx_TxMsg.StdId = 0x1ff;
    for (int i = 0; i < 8; i++) {
        can1_context.CANx_TxMsg.Data[i] = 0;
        can2_context.CANx_TxMsg.Data[i] = 0;
    }

    for (int i = 0; i < GIMBAL_MOTOR_NUM; i++) {
        uint32_t id = gimbal_motor[i]->m_id;
        int16_t cmd = (int16_t)gimbal_motor[i]->m_speed_pid->m_output;
        if (id >= 1 && id <= 4) {
            if (gimbal_motor[i]->m_CANx == CAN1) {
                can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            } else if (gimbal_motor[i]->m_CANx == CAN2) {
                can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    }

    transmint_1 = can1_context.CanSendMessage();
    transmint_2 = can2_context.CanSendMessage();

    if(transmint_1 == CAN_TxStatus_NoMailBox)
    {   
        can1_context.CANx->TSR|= CAN_TSR_ABRQ0;
        can1_context.CANx->TSR|= CAN_TSR_ABRQ1;
        can1_context.CANx->TSR|= CAN_TSR_ABRQ2;
    }
    if(transmint_2 == CAN_TxStatus_NoMailBox)
    {   
        can2_context.CANx->TSR|= CAN_TSR_ABRQ0;
        can2_context.CANx->TSR|= CAN_TSR_ABRQ1;
        can2_context.CANx->TSR|= CAN_TSR_ABRQ2;
    }
}