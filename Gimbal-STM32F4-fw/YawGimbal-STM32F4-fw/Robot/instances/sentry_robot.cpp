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
    yaw_gimbal_motor[LEFT_YAW_MOTOR] = new M3508(CAN2, YAW_GIMBAL_LEFT_MOTOR_ID, YAW_GIMBAL_MOTOR_REDUCTION_RAITO);
    yaw_gimbal_motor[LEFT_YAW_MOTOR]->m_angle_td = new Adrc_TD(7000, 0.001, 0.001,0);
    yaw_gimbal_motor[LEFT_YAW_MOTOR]->m_angle_pid = new Pid(25, 0.005, 0, 10, 2000, 2000, 5000, 2000);
    yaw_gimbal_motor[LEFT_YAW_MOTOR]->m_speed_pid = new Pid(70, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    yaw_gimbal_motor[LEFT_YAW_MOTOR]->m_encoder = new AbsEncoder(YAW_GIMBAL_LEFT_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    yaw_gimbal_motor[LEFT_YAW_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    yaw_gimbal_motor[LEFT_YAW_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);


    yaw_gimbal_motor[RIGHT_YAW_MOTOR] = new M3508(CAN2, YAW_GIMBAL_RIGHT_MOTOR_ID, YAW_GIMBAL_MOTOR_REDUCTION_RAITO);
    yaw_gimbal_motor[RIGHT_YAW_MOTOR]->m_angle_td = new Adrc_TD(7000, 0.001, 0.001,0);
    yaw_gimbal_motor[RIGHT_YAW_MOTOR]->m_angle_pid = new Pid(25, 0.005, 0, 10, 2000, 2000, 5000, 2000);
    yaw_gimbal_motor[RIGHT_YAW_MOTOR]->m_speed_pid = new Pid(70, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    yaw_gimbal_motor[RIGHT_YAW_MOTOR]->m_encoder = new AbsEncoder(YAW_GIMBAL_RIGHT_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    yaw_gimbal_motor[RIGHT_YAW_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    yaw_gimbal_motor[RIGHT_YAW_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);
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
    gimbal_imu[GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_imu[GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_y = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
}



/**
 *@brief Set the sentry robot first gimbal pitch to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetGimbalAngleTarget(float target_l , float target_r)
{
    yaw_gimbal_motor[LEFT_YAW_MOTOR]->m_angle_target = target_l;
    yaw_gimbal_motor[RIGHT_YAW_MOTOR]->m_angle_target = target_r;
}



/**
 *@brief execute the sentry robot gimbal control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteGimbalAlgorithm(void)
{
    if (gimbal_mode != YAW_GIMBAL_SAFE) {
        for (int i = 0; i < YAW_GIMBAL_MOTOR_NUM; i++) {
            yaw_gimbal_motor[i]->AngleControl();
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
    uint8_t transmint_1;
    uint8_t transmint_2;

    // CAN1 ID 0x200    CAN2 ID 0x200
    can1_context.CANx_TxMsg.StdId = 0x200;
    can2_context.CANx_TxMsg.StdId = 0x200;
    for (int i = 0; i < 8; i++) {
        can1_context.CANx_TxMsg.Data[i] = 0;
        can2_context.CANx_TxMsg.Data[i] = 0;
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