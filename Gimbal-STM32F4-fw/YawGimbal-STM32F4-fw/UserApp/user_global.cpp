#include "user_global.h"

SystemMonitor G_system_monitor;
Led G_led;
Vofa G_vofa;
Gimbal G_gimbal;
SentryRobot G_sentry;
ControlMode G_control_mode;


void MotorMonitor(void)
{
    if (G_system_monitor.UART2_rx_fps > 900) {
		G_led.ToggleBlue();
	}
	if (G_system_monitor.UART6_rx_fps > 200) {
		G_led.ToggleGreen();
	}
    if(G_system_monitor.CAN1_rx_fps > 4000){
        G_led.ToggleYellow();
    }
}



/**
 *@brief visualize chassis datas
 * 
 *@param 
*/
void VisualizeGimbalData(void) {
    G_vofa.m_data_send_frame.m_data[0] = G_gimbal.m_data_receive_frame.m_fdata[0];
    G_vofa.m_data_send_frame.m_data[1] = G_gimbal.m_data_receive_frame.m_fdata[1];
    G_vofa.m_data_send_frame.m_data[2] = G_gimbal.m_data_receive_frame.m_fdata[2];
    G_vofa.m_data_send_frame.m_data[3] = G_gimbal.m_data_receive_frame.m_fdata[3];
    G_vofa.m_data_send_frame.m_data[4] = G_gimbal.m_data_receive_frame.m_fdata[4];
    G_vofa.m_data_send_frame.m_data[5] = G_gimbal.m_data_receive_frame.m_fdata[5];
    G_vofa.m_data_send_frame.m_data[6] = G_gimbal.m_data_receive_frame.m_fdata[6];
    G_vofa.m_data_send_frame.m_data[7] = G_gimbal.m_data_receive_frame.m_fdata[7];
    G_vofa.m_data_send_frame.m_data[8] = G_gimbal.m_data_receive_frame.m_fdata[8];
    G_vofa.m_data_send_frame.m_data[9] = G_gimbal.m_data_receive_frame.m_fdata[9];
    G_vofa.m_data_send_frame.m_data[10] = G_gimbal.m_data_receive_frame.m_fdata[10];
    G_vofa.m_data_send_frame.m_data[11] = G_gimbal.m_data_receive_frame.m_fdata[11];
    G_vofa.m_data_send_frame.m_data[12] = G_gimbal.m_data_receive_frame.m_fdata[12];
    G_vofa.m_data_send_frame.m_data[13] = G_gimbal.m_data_receive_frame.m_fdata[13];
    G_vofa.m_data_send_frame.m_data[14] = G_gimbal.m_data_receive_frame.m_fdata[14];
    G_vofa.m_data_send_frame.m_data[15] = G_gimbal.m_data_receive_frame.m_fdata[15];
    G_vofa.m_data_send_frame.m_data[16] = G_gimbal.m_data_receive_frame.m_fdata[16];
    // G_vofa.m_data_send_frame.m_data[17] = G_gimbal.m_data_receive_frame.m_fdata[17];
    // G_vofa.m_data_send_frame.m_data[18] = G_gimbal.m_data_receive_frame.m_fdata[18];
    G_vofa.m_data_send_frame.m_data[17] = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_target;
    G_vofa.m_data_send_frame.m_data[18] = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current;

    G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.UART2_rx_fps;
}



void SendCoreData(void)
{
    G_gimbal.m_data_send_frame.m_sdata[0] = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_speed_pid->m_output;
    G_gimbal.m_data_send_frame.m_sdata[1] = G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_YAW_MOTOR]->m_speed_pid->m_output;

    G_gimbal.m_data_send_frame.m_fdata[0] = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current;

    Append_CRC16_Check_Sum((uint8_t*)&G_gimbal.m_data_send_frame,GIMBAL_DATA_SEND_SIZE);
    G_gimbal.SendData();
}






/**
 *@brief update system control mode
 * 
 *@param 
*/
void ControlModeUpdate(void) 
{
    if(G_system_monitor.UART2_rx_fps > 900)
    {
        if(G_gimbal.m_data_receive_frame.m_cdata[0] > 0.5 && G_gimbal.m_data_receive_frame.m_cdata[0] < 1.5)
            G_sentry.SetGimbalMode(SentryRobot::YAW_GIMBAL_MOVE);
        else if (G_gimbal.m_data_receive_frame.m_cdata[0] > 1.5)
            G_sentry.SetGimbalMode(SentryRobot::YAW_GIMBAL_STATIC);
        else    
            G_sentry.SetGimbalMode(SentryRobot::YAW_GIMBAL_SAFE);
    }
    else
        G_sentry.SetGimbalMode(SentryRobot::YAW_GIMBAL_SAFE);
};



/**
 *@brief update the state of the sentry robot
 * 
 *@param 
*/
void RobotStatesUpdate(void)
{
    float angle_step = 0;

    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAccData();
    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataGyroData();
    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAngleData();

    G_sentry.yaw_angle_fb_pre = G_sentry.yaw_angle_fb;
    G_sentry.yaw_angle_fb = -G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
    angle_step = G_sentry.yaw_angle_fb - G_sentry.yaw_angle_fb_pre;

    if(angle_step > 180)
        angle_step -= 360;
    if(angle_step < -180)
        angle_step += 360;

    G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current += angle_step;
    G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_speed_current = -G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z->GetFilterOutput() * RADIANODEGREES;

    G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_YAW_MOTOR]->m_angle_current += angle_step;
    G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_YAW_MOTOR]->m_speed_current = -G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z->GetFilterOutput() * RADIANODEGREES;
}



/**
 *@brief update the robot targets 
 * 
 *@param 
*/
void RobotTargetsUpdate(void) 
{
    GimbalTargetsUpdate();
}



void GimbalTargetsUpdate(void)
{
    static uint8_t fisrt_flag = 0;
    if(G_sentry.gimbal_mode == SentryRobot::YAW_GIMBAL_MOVE)
    {
        if(!G_sentry.test_flag)
        {
            if(G_system_monitor.UART2_rx_fps > 900)
            {
                G_sentry.yaw_angle_des = G_gimbal.m_data_receive_frame.m_fdata[0];
            }
            else
            {
                G_sentry.yaw_angle_des = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current;
                G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_td->m_x1 = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current;
                G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_td->m_aim = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current;

                for (int i = 0; i < YAW_GIMBAL_MOTOR_NUM; i++)
                    G_sentry.yaw_gimbal_motor[i]->m_speed_pid->m_output = 0;
            }
            fisrt_flag = 0;
        }
        else
            GimbalTest();

    }
    else if (G_sentry.gimbal_mode == SentryRobot::YAW_GIMBAL_STATIC)
    {
        if(!fisrt_flag)
        {
            fisrt_flag = 1;
            G_sentry.yaw_angle_des = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current;
        }
    }
    else 
    {
        G_sentry.yaw_angle_des = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_YAW_MOTOR]->m_angle_current;
        fisrt_flag = 0;

        for (int i = 0; i < YAW_GIMBAL_MOTOR_NUM; i++)
        {
            G_sentry.yaw_gimbal_motor[i]->m_speed_pid->m_output = 0;
            G_sentry.yaw_gimbal_motor[i]->m_angle_target = G_sentry.yaw_gimbal_motor[i]->m_angle_current;
            G_sentry.yaw_gimbal_motor[i]->m_speed_target = G_sentry.yaw_gimbal_motor[i]->m_speed_current;
            G_sentry.yaw_gimbal_motor[i]->m_angle_pid->m_error_sum = 0;
            G_sentry.yaw_gimbal_motor[i]->m_angle_td->m_aim = G_sentry.yaw_gimbal_motor[i]->m_angle_current;
        }
    }

    G_sentry.SetGimbalAngleTarget(G_sentry.yaw_angle_des,G_sentry.yaw_angle_des);
}


void GimbalTest(void)
{
    static float yaw_des_pre = 0;
    float yaw_des_step = 0;

    yaw_des_step = G_sentry.yaw_test_des - yaw_des_pre;
    yaw_des_pre = G_sentry.yaw_test_des;

    G_sentry.yaw_angle_des += yaw_des_step;
}



/**
 *@brief execute the robot control 
 * 
 *@param 
*/
void RobotControlExecute(void)
{
    // Execute the robot gimbal control algorithm
    G_sentry.ExecuteGimbalAlgorithm();

    // Send the control commands to the actuators
    G_sentry.SendControlCommand();
}