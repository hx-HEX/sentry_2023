#include "user_global.h"

SystemMonitor G_system_monitor;
Led G_led;
Vofa G_vofa;
Gimbal G_gimbal;
AimAssist G_aim_assist;
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
    G_vofa.m_data_send_frame.m_data[0] = G_gimbal.m_data_receive_frame.m_cdata[0];
    G_vofa.m_data_send_frame.m_data[1] = G_gimbal.m_data_receive_frame.m_cdata[1];
    G_vofa.m_data_send_frame.m_data[2] = G_gimbal.m_data_receive_frame.m_cdata[2];
    G_vofa.m_data_send_frame.m_data[3] = G_gimbal.m_data_receive_frame.m_cdata[3];
    G_vofa.m_data_send_frame.m_data[4] = G_gimbal.m_data_receive_frame.m_fdata[0];
    G_vofa.m_data_send_frame.m_data[5] = G_gimbal.m_data_receive_frame.m_fdata[1];

    G_vofa.m_data_send_frame.m_data[13] = G_system_monitor.UART6_rx_fps;
    G_vofa.m_data_send_frame.m_data[14] = G_system_monitor.CAN2_rx_fps;
}



void SendCoreData(void)
{
    G_gimbal.m_data_send_frame.m_cdata[0] = G_sentry.supply_bullet_flag;
    G_gimbal.m_data_send_frame.m_cdata[1] = G_sentry.enemy_number;
    G_gimbal.m_data_send_frame.m_fdata[0] = G_aim_assist.m_data_receive_frame.m_data_f[3];
    G_gimbal.m_data_send_frame.m_fdata[1] = G_aim_assist.m_data_receive_frame.m_data_f[4];
    G_gimbal.m_data_send_frame.m_fdata[2] = G_aim_assist.m_data_receive_frame.m_data_f[5];
    G_gimbal.m_data_send_frame.m_fdata[3] = G_aim_assist.m_data_receive_frame.m_data_f[6];
    G_gimbal.m_data_send_frame.m_fdata[4] = G_aim_assist.m_data_receive_frame.m_data_f[7];
    G_gimbal.m_data_send_frame.m_fdata[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current;
    G_gimbal.m_data_send_frame.m_fdata[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current;

    Append_CRC16_Check_Sum((uint8_t*)&G_gimbal.m_data_send_frame,GIMBAL_DATA_SEND_SIZE);
    G_gimbal.SendData();
}



void SendAimAssistData(void)
{
    G_aim_assist.m_data_send_frame.m_id = G_sentry.enemy_color;

    G_aim_assist.m_data_send_frame.m_data[0] = 
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current;
    G_aim_assist.m_data_send_frame.m_data[1] = 
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current;

    G_aim_assist.m_data_send_frame.m_data[2] = G_sentry.bullet_speed;
    G_aim_assist.m_data_send_frame.m_data[3] = G_sentry.balance_infantry_num;

    Append_CRC16_Check_Sum((uint8_t*)&G_aim_assist.m_data_send_frame,AIM_ASSIST_DATA_SEND_SIZE);
    G_aim_assist.SendData();
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
        if(G_sentry.shoot_flag < 0.5)
            G_sentry.SetShootMode(SentryRobot::SHOOT_MOVE);
        else
            G_sentry.SetShootMode(SentryRobot::SHOOT_SAFE);
        if(G_sentry.run_flag > 0.5 && G_sentry.run_flag < 1.5) //发的数为1
            G_sentry.SetGimbalMode(SentryRobot::GIMBAL_MOVE);
        else if(G_sentry.run_flag > 1.5)//发的数为2
            G_sentry.SetGimbalMode(SentryRobot::GIMBAL_AUTO);
        else
            G_sentry.SetGimbalMode(SentryRobot::GIMBAL_SAFE); 
    }
    else
    {
        G_sentry.SetShootMode(SentryRobot::SHOOT_SAFE);
        G_sentry.SetGimbalMode(SentryRobot::GIMBAL_SAFE);
    }
    
};



/**
 *@brief update the state of the sentry robot
 * 
 *@param 
*/
void RobotStatesUpdate(void)
{
    //imu数据更新
    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAccData();
    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataGyroData();
    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAngleData();

    //电机反馈更新
    #ifdef FITCH_ANGLE_ENCODER_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current = G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current_encoder;
	#endif
	#ifdef FITCH_ANGLE_IMU_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.pitch;
	#endif
	#ifdef FITCH_SPEED_IMU_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_speed_current = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_y->GetFilterOutput() * RADIANODEGREES;
	#endif
	#ifdef FITCH_SPEED_ENCODER_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_speed_current =  G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_speed_current_encoder;
    #endif

    #ifdef YAW_ANGLE_ENCODER_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current = G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current_encoder;
	#endif
	#ifdef YAW_ANGLE_IMU_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
	#endif
	#ifdef YAW_SPEED_IMU_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_speed_current = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z->GetFilterOutput() * RADIANODEGREES;
	#endif
	#ifdef YAW_SPEED_ENCODER_FEEDBACK
	G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_speed_current =  G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_speed_current_encoder;
    #endif

    G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_speed_current = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpFB;
    G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_speed_current = G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpFB;

    //接收主控变量更新
    if(G_system_monitor.UART2_rx_fps > 900)
    {
        G_sentry.pitch_rc_des = G_gimbal.m_data_receive_frame.m_fdata[0];
        G_sentry.yaw_rc_des = G_gimbal.m_data_receive_frame.m_fdata[1];
        G_sentry.bullet_speed = G_gimbal.m_data_receive_frame.m_fdata[2];

        G_sentry.shoot_flag = G_gimbal.m_data_receive_frame.m_cdata[0];
        G_sentry.run_flag = G_gimbal.m_data_receive_frame.m_cdata[1];
        G_sentry.balance_infantry_num = G_gimbal.m_data_receive_frame.m_cdata[2];
        G_sentry.enemy_color = G_gimbal.m_data_receive_frame.m_cdata[3];
        G_sentry.m_scan_flag = G_gimbal.m_data_receive_frame.m_cdata[4];
    }
    else
    {
        G_sentry.pitch_rc_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current;
        G_sentry.yaw_rc_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current;
        G_sentry.bullet_speed = 28;

        G_sentry.shoot_flag = 0;
        G_sentry.run_flag = 0;
        G_sentry.balance_infantry_num = 0;
        G_sentry.enemy_color = 0x09;
        G_sentry.m_scan_flag = 0;
    }
        

    //接收辅瞄数据更新
    G_sentry.enemy_find_flag = 0;
    G_sentry.supply_bullet_flag = 0;
    if(G_system_monitor.UART6_rx_fps > 180)
    {
        if(G_aim_assist.m_data_receive_frame.m_data_f[2] > 0.5 
        && fabs(G_aim_assist.m_data_receive_frame.m_data_f[1] - G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current) < Yaw_Vision_Error 
        && fabs(G_aim_assist.m_data_receive_frame.m_data_f[0] - G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current) < Pitch_Vison_Error)
        {
            G_sentry.supply_bullet_flag = 1;
        }

        if (G_aim_assist.m_data_receive_frame.m_id == 0) 
            G_sentry.enemy_find_flag = 0;
        else 
            G_sentry.enemy_find_flag = 1;
    }
}



/**
 *@brief update the robot targets 
 * 
 *@param 
*/
void RobotTargetsUpdate(void) 
{
    ShootTargetsUpdate();
    GimbalTargetsUpdate();
}



void GimbalTargetsUpdate(void)
{
    if(G_sentry.gimbal_mode == SentryRobot::GIMBAL_MOVE)
    {
        if(!G_sentry.test_flag)
        {
            if(G_system_monitor.UART2_rx_fps > 900)
            {
                G_sentry.pitch_angle_des = G_sentry.pitch_rc_des;
                G_sentry.yaw_angle_des = G_sentry.yaw_rc_des;
            }
            else
            {
                G_sentry.pitch_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current;
                G_sentry.yaw_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current;

                for (int i = 0; i < GIMBAL_MOTOR_NUM; i++)
                    G_sentry.gimbal_motor[i]->m_speed_pid->m_output = 0;
            }
        }
        else    
            GimbalTest();

    }
    else if (G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO)
    {
        if(!G_sentry.m_scan_flag)
        {
            G_sentry.yaw_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current;
            G_sentry.pitch_angle_des = 0;
        }
        else
        {
            if(G_sentry.enemy_find_flag 
                && (G_aim_assist.m_data_receive_frame.m_data_f[0] > PITCH_ANGLE_MIN) && (G_aim_assist.m_data_receive_frame.m_data_f[0] < PITCH_ANGLE_MAX)
                && (G_aim_assist.m_data_receive_frame.m_data_f[1] > YAW_ANGLE_MIN) && (G_aim_assist.m_data_receive_frame.m_data_f[1] < YAW_ANGLE_MAX))
            {
                if(fpclassify(G_aim_assist.m_data_receive_frame.m_data_f[0])== FP_NORMAL)
                    G_sentry.pitch_angle_des = G_aim_assist.m_data_receive_frame.m_data_f[0];
                if(fpclassify(G_aim_assist.m_data_receive_frame.m_data_f[1])== FP_NORMAL)
                    G_sentry.yaw_angle_des = G_aim_assist.m_data_receive_frame.m_data_f[1];

                if(fabs(G_aim_assist.m_data_receive_frame.m_data_f[1] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current) < 5.0f)
                {
                    if((fabs(G_aim_assist.m_data_receive_frame.m_data_f[1] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current) < 2.0f) || G_sentry.change_yaw_flag)
                    {
                        G_sentry.change_yaw_flag = 1;
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_pid->m_kp = 35;   
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_td->m_k = 1;
                    }               
                }
                else{
                    G_sentry.change_yaw_flag = 0;
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_pid->m_kp = 15; 
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_td->m_k = 0;
                }

                if(fabs(G_aim_assist.m_data_receive_frame.m_data_f[0] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current) < 3.0f)
                {
                    if((fabs(G_aim_assist.m_data_receive_frame.m_data_f[0] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current) < 1.5f) || G_sentry.change_pitch_flag)
                    {
                        G_sentry.change_pitch_flag = 1;
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_pid->m_kp = 40;  
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_td->m_k = 0.8;
                    }
                }
                else{
                        G_sentry.change_pitch_flag = 0;
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_pid->m_kp = 20;  
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_td->m_k = 0;
                }
                
                if(G_aim_assist.m_data_receive_frame.m_data_f[3] < 0.5f)
                    G_sentry.change_scan_flag = 0;
                else
                    G_sentry.change_scan_flag = 1;

                G_sentry.m_vision_delay_cnt = 0;
                G_sentry.direction_flag = 1;
            }
            else
            {
                if(G_sentry.m_vision_delay_cnt < 500)
                {
                    G_sentry.m_vision_delay_cnt++;
                }
                else
                {
                    if(G_sentry.yaw_scan_dir)
                        G_sentry.yaw_angle_des += GIMBAL_PITCH_ANGLE_SCAN_SPEED;
                    else
                        G_sentry.yaw_angle_des -= GIMBAL_PITCH_ANGLE_SCAN_SPEED;
                    if(G_sentry.pitch_scan_dir)
                        G_sentry.pitch_angle_des += GIMBAL_YAW_ANGLE_SCAN_SPEED;
                    else    
                        G_sentry.pitch_angle_des -= GIMBAL_YAW_ANGLE_SCAN_SPEED;
                    
                    if(G_sentry.pitch_angle_des > GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MAX)
                        G_sentry.pitch_scan_dir = 0;
                    else if(G_sentry.pitch_angle_des < GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MIN)
                        G_sentry.pitch_scan_dir = 1;
                    
                    if(G_sentry.yaw_angle_des > GIMBAL_YAW_ANGLE_SCAN_ANGLE_MAX)
                        G_sentry.yaw_scan_dir = 0;
                    else if(G_sentry.yaw_angle_des < GIMBAL_YAW_ANGLE_SCAN_ANGLE_MIN)
                        G_sentry.yaw_scan_dir = 1;

                    if (G_aim_assist.m_data_receive_frame.m_data_f[3] < 0.5f && G_sentry.direction_flag)
                        G_sentry.yaw_scan_dir = 0;
                    else if (G_aim_assist.m_data_receive_frame.m_data_f[3] > 0.5f && G_sentry.direction_flag)
                        G_sentry.yaw_scan_dir = 1;
                
                    G_sentry.m_vision_delay_cnt = 600;
                    G_sentry.direction_flag = 0;
                }
            }
        }
    }
    else
    {
        G_sentry.pitch_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_angle_current;
        G_sentry.yaw_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_angle_current;

        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++)
            G_sentry.gimbal_motor[i]->m_speed_pid->m_output = 0;
    }
    
    G_sentry.pitch_angle_des = Clip(G_sentry.pitch_angle_des,PITCH_ANGLE_MIN,PITCH_ANGLE_MAX);
    G_sentry.yaw_angle_des = Clip(G_sentry.yaw_angle_des,YAW_ANGLE_MIN,YAW_ANGLE_MAX);
    G_sentry.SetGimbalAngleTarget(G_sentry.pitch_angle_des,G_sentry.yaw_angle_des);
}



void GimbalTest(void)
{
    static float pitch_des_pre = 0;
    static float yaw_des_pre = 0;
    float pitch_des_step = 0;
    float yaw_des_step = 0;

    pitch_des_step = G_sentry.test_pitch_des - pitch_des_pre;
    pitch_des_pre = G_sentry.test_pitch_des;

    yaw_des_step = G_sentry.test_yaw_des - yaw_des_pre;
    yaw_des_pre = G_sentry.test_yaw_des;

    G_sentry.pitch_angle_des += pitch_des_step;
    G_sentry.yaw_angle_des += yaw_des_step;
}



void ShootTargetsUpdate(void)
{
    if (G_system_monitor.UART2_rx_fps > 900) 
    {
        if(G_sentry.shoot_mode != SentryRobot::SHOOT_SAFE)
        {
            G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpDes = -FRIC_WHEEL_SPEED;
            G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpDes = FRIC_WHEEL_SPEED;
        }
        else
        {
            for (uint8_t i = 0; i < SHOOT_MOTOR_NUM; i++)
            {
                G_sentry.shoot_motor[i]->m_smc->m_fpDes = 0;
                G_sentry.shoot_motor[i]->m_smc->m_fpU = 0;
            }
        }  
    } 
    else 
    {
        for (uint8_t i = 0; i < SHOOT_MOTOR_NUM; i++)
        {
            G_sentry.shoot_motor[i]->m_smc->m_fpDes = 0;
            G_sentry.shoot_motor[i]->m_smc->m_fpU = 0;
        }
    }
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

    // Execute the robot shoot control algorithm
    G_sentry.ExecuteShootAlgorithm();

    // Send the control commands to the actuators
    G_sentry.SendControlCommand();
}