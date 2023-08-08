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

    yawgimbal_mode = YAWGIMBAL_SAFE;
    yawgimbal_mode_pre = YAWGIMBAL_SAFE;

    m_yaw_angle = 0;

    spinning_flag = 0;
    ctrl_spinning_flag = 0;
    vision_flag = 0;
    safe_flag = 0;

    yaw_calibrate_flag = 0;
    yaw_calibrate_flag_pre = 0;
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
    chassis_line_motor[CHASSIS_FRL_MOTOR] = new M3508(CAN1, CHASSIS_FRL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_FRL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.05f, 0.005f);

    
    chassis_line_motor[CHASSIS_FLL_MOTOR] = new M3508(CAN1, CHASSIS_FLL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_FLL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.001f, 0.5f);

    
    chassis_line_motor[CHASSIS_BLL_MOTOR] = new M3508(CAN1, CHASSIS_BLL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_BLL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);

    
    chassis_line_motor[CHASSIS_BRL_MOTOR] = new M3508(CAN1, CHASSIS_BRL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_BRL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);


    yaw_gimbal_motor[LEFT_MOTOR] = new M3508(CAN2, YAW_GIMBAL_LEFT_MOTOR_ID, YAW_GIMBAL_MOTOR_REDUCTION_RAITO);
    yaw_gimbal_motor[LEFT_MOTOR]->m_angle_td = new Adrc_TD(7000, 0.001, 0.001,0);
    yaw_gimbal_motor[LEFT_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    yaw_gimbal_motor[LEFT_MOTOR]->m_speed_pid = new Pid(0, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    yaw_gimbal_motor[LEFT_MOTOR]->m_encoder = new AbsEncoder(YAW_GIMBAL_LEFT_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    yaw_gimbal_motor[LEFT_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    yaw_gimbal_motor[LEFT_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);


    yaw_gimbal_motor[RIGHT_MOTOR] = new M3508(CAN2, YAW_GIMBAL_RIGHT_MOTOR_ID, YAW_GIMBAL_MOTOR_REDUCTION_RAITO);
    yaw_gimbal_motor[RIGHT_MOTOR]->m_angle_td = new Adrc_TD(7000, 0.001, 0.001,0);
    yaw_gimbal_motor[RIGHT_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    yaw_gimbal_motor[RIGHT_MOTOR]->m_speed_pid = new Pid(0, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    yaw_gimbal_motor[RIGHT_MOTOR]->m_encoder = new AbsEncoder(YAW_GIMBAL_RIGHT_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    yaw_gimbal_motor[RIGHT_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    yaw_gimbal_motor[RIGHT_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);

}


/**
 *@brief Initial all sensors of the Sentry robot
 * 
 *@param 
*/
void SentryRobot::InitAllSensors(void)
{
    capacitor = new Capacitor();
    capacitor->m_power_control_pid = new Pid(1,0.02,0,10,10,10,10,0);
}



/**
 *@brief execute the sentry robot chassis control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteChassisAlgorithm(void)
{
    if (chassis_mode != CHASSIS_SAFE) {
        #ifdef CHASSIS_OMNIDIRECTIONAL_MODE
        for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++) {
            chassis_line_motor[i]->SpeedControl();
        }
        #endif
        Power_Control();
    }
}



void SentryRobot::ExecuteYawGimbakAlgorithm(void)
{
    if(yawgimbal_mode != YAWGIMBAL_SAFE && yawgimbal_mode != YAWGIMBAL_CALIBRATE){
        for(uint8_t i = 0; i < YAW_GIMBAL_NUM; i++) {
            yaw_gimbal_motor[i]->AngleControl();
        }
    }
}



/**
 *@brief Set the sentry robot chassis line wheel speed to the specified speed
 * 
 *@param 
*/
void SentryRobot::SetChassisSpeedTarget(float fll_motor, float bll_motor, float frl_motor, float brl_motor)
{
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_speed_target = fll_motor;
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_speed_target = bll_motor;
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_speed_target = frl_motor;
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_speed_target = brl_motor;
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
    if (chassis_mode != CHASSIS_SAFE) {
        for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++) {
            uint32_t id = chassis_line_motor[i]->m_id;
            int16_t cmd = (int16_t)chassis_line_motor[i]->m_speed_pid->m_output;
            if ( cmd >= WHEEL_CURRENT_DEAD_AREA ) {
                cmd += FORWARD_DAMP_FACTOR;
            } else if ( cmd < -WHEEL_CURRENT_DEAD_AREA ){
                cmd -= FORWARD_DAMP_FACTOR;
            }
            if (id >= 1 && id <= 4) {
                if (chassis_line_motor[i]->m_CANx == CAN1) {
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                } else if (chassis_line_motor[i]->m_CANx == CAN2) {
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                }
            }
        }
    }
    if (yawgimbal_mode != YAWGIMBAL_SAFE){
        for (uint8_t i = 0; i < YAW_GIMBAL_NUM; i++) {
            uint32_t id = yaw_gimbal_motor[i]->m_id;
            int16_t cmd = (int16_t)yaw_gimbal_motor[i]->m_speed_pid->m_output;
            if (id >= 1 && id <= 4){
                if(yaw_gimbal_motor[i]->m_CANx == CAN1) {
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                }else if (yaw_gimbal_motor[i]->m_CANx == CAN2){
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                }
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

    // Capacitor power charging target
    
    if(chassis_mode == CHASSIS_SAFE) capacitor->CapVolLimit();
    int16_t cmd = (int16_t)capacitor->m_power_charging * 100;
    can2_context.CANx_TxMsg.Data[2] = (uint8_t)( cmd >> 8 );
    can2_context.CANx_TxMsg.Data[3] = (uint8_t)(cmd);
    
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



void SentryRobot::Power_Control(void)
{
   if(capacitor->CAP_STATE == 0)
   {
       capacitor->m_power_control_pid->m_error_sum=0;
       capacitor->m_power_control_pid->m_error_pre=0;
   }

    if((capacitor->Pre_CAP_STATE == 1) && (capacitor->CAP_STATE==0))
    {
        for(uint8_t i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
        {
            chassis_line_motor[i]->m_speed_pid->m_error_sum = 0;
        }
    }

    if(capacitor->CAP_STATE == 1)
    {
        if(capacitor->CAP_POW_USE > 0.5)
        {
            if(capacitor->CAP_POW_USE > 0.9 || capacitor->FLAG_CAP_Pow == 1)
            {
                capacitor->m_power_control_pid->CalWeakenPID(-log(capacitor->CAP_POW_USE)/log(2.7183));
                capacitor->CAP_Cur_Coe = pow(2.7183,(double)capacitor->m_power_control_pid->m_output);
                capacitor->FLAG_CAP_Pow = 1;
                for(uint8_t i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
                {
                    chassis_line_motor[i]->m_speed_pid->m_output *= capacitor->CAP_Cur_Coe;
                }
            }
        }
        else
        {
            capacitor->FLAG_CAP_Pow = 0;
            capacitor->m_power_control_pid->m_error_pre=0;
            capacitor->m_power_control_pid->m_error_sum=0;
        }
    }

    if(capacitor->CAP_STATE == 2)
    {
        capacitor->m_power_control_pid->m_error_pre=0;
        capacitor->m_power_control_pid->m_error_sum=0;

         for(uint8_t i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
        {
            chassis_line_motor[i]->m_speed_pid->m_output = 0;
        }
    }

    capacitor->Pre_CAP_STATE = capacitor->CAP_STATE;
}


