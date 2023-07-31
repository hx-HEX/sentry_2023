#include "user_global.h"

SystemMonitor G_system_monitor;
RefereeMonitor G_referee_monitor;
Led G_led;
Gimbal G_gimbal;
Vofa G_vofa;
Referee G_referee;
SentryRobot G_sentry;
ControlMode G_control_mode;



/**
 *@brief Motor state monitor
 *
 *@param
 */
void MotorMonitor(void)
{
    static uint32_t times = 0;
    times++;
    if (G_system_monitor.CAN1_rx_fps < 5000 && G_system_monitor.CAN2_rx_fps)
    {
        if (times % 3 == 0)
        {
            G_led.ToggleGreen();
        }
    }
    else if (G_system_monitor.CAN2_rx_fps < 5000)
    {
        if (times % 2 == 0)
        {
            G_led.ToggleGreen();
        }
    }
    else if (G_system_monitor.CAN1_rx_fps < 5000)
    {
        G_led.ToggleGreen();
    }
    else
    {
        G_led.SetGreen(false);
    }
}



/**
 *@brief Communication state monitor
 *
 *@param
 */
void CommunicationMonitor(void)
{
    static uint32_t times = 0;
    times++;
    if (G_system_monitor.UART2_rx_fps < 50)
    {
        G_led.ToggleBlue();
    }
    if (G_system_monitor.UART4_rx_fps < 900)
    {
        G_led.ToggleOrange();
    }
}



/**
 *@brief 
 *
 *@param
 */
void VisualizeData(void)
{
    G_vofa.m_data_send_frame.m_data[0] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_target;
    G_vofa.m_data_send_frame.m_data[1] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_current;
    G_vofa.m_data_send_frame.m_data[2] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_target;
    G_vofa.m_data_send_frame.m_data[3] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_current;
    G_vofa.m_data_send_frame.m_data[4] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_target;
    G_vofa.m_data_send_frame.m_data[5] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_current;
    G_vofa.m_data_send_frame.m_data[6] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_target;
    G_vofa.m_data_send_frame.m_data[7] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_current;

    G_vofa.m_data_send_frame.m_data[16] = G_system_monitor.CAN1_rx_fps;
    G_vofa.m_data_send_frame.m_data[17] = G_system_monitor.CAN2_rx_fps;
    G_vofa.m_data_send_frame.m_data[18] = G_system_monitor.UART2_rx_fps;
    G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.UART4_rx_fps;

}



/**
 *@brief send gimabl datas
 *
 *@param
 */
void SendGimbalData(void)
{
    G_gimbal.m_data_send_frame.m_fdata[0] = G_sentry.m_yaw_angle;
    uint32_t gimbal_data_num = sizeof(G_gimbal.m_data_send_frame);

    if(G_gimbal.gimbal_send_buff_cnt + gimbal_data_num < GIMBAL_SEND_BUFF_SIZE){
		memcpy(&G_gimbal.gimbal_send_buff[G_gimbal.gimbal_send_buff_cnt],&G_gimbal.m_data_send_frame,gimbal_data_num);
		G_gimbal.gimbal_send_buff_cnt += gimbal_data_num;
	}
    
    G_gimbal.usart_gimbal->USART_RT.txlen = G_gimbal.gimbal_send_buff_cnt;
    G_gimbal.SendData();
    G_gimbal.gimbal_send_buff_cnt = 0;
}


/**
 *@brief send referee datas
 *
 *@param
 */
void SendRefereeData(void)
{
    // TEST
    // G_referee.temp_cnt++;
    // if(G_referee.temp_cnt >= 10){
    //     G_referee.temp_x += 1;
    //     G_referee.temp_y += 1;
    //     G_referee.temp_cnt = 0;
    // }
    // TEST
    G_referee.m_loc_update_cnt++;
    if(G_referee.m_loc_update_cnt >= 10){

#ifdef CHASSIS_STEER_DRIVING_MODE
        if(G_sentry.m_robot_chassis_spin_ready){
            G_referee.MapSentryData.intention = 2; // SPIN
        }else if(!G_sentry.m_robot_chassis_spin_ready && G_sentry.m_robot_chassis_speed > CHASSIS_SPIN_OVER_LIMIT){
            G_referee.MapSentryData.intention = 1; // MOVE
        }else{
            G_referee.MapSentryData.intention = 3; // STOP
        }
#endif
#ifdef CHASSIS_OMNIDIRECTIONAL_MODE
        G_referee.MapSentryData.intention = 3; // STOP
#endif


        uint16_t current_pos_x = G_gimbal.m_data_receive_frame.m_fdata[4]*10;
        uint16_t current_pos_y = G_gimbal.m_data_receive_frame.m_fdata[5]*10;
        // uint16_t current_pos_x = G_referee.temp_x; // TEST
        // uint16_t current_pos_y = G_referee.temp_y; // TEST
        if(!G_referee.m_loc_send_init_flag){
            G_referee.m_loc_delta_x_total = 0;
            G_referee.m_loc_delta_y_total = 0;
            G_referee.m_loc_pos_x_pre = current_pos_x;
            G_referee.m_loc_pos_y_pre = current_pos_y;
            for(uint8_t i = 0; i < REFEREE_MAP_SIZE; i++){
                *(G_referee.MapSentryData.delta_x+i) = 0;
                *(G_referee.MapSentryData.delta_y+i) = 0;
            }
            G_referee.m_loc_queue_len = 0;
            G_referee.m_loc_send_init_flag = true;
        }
        int8_t delta_x = current_pos_x - G_referee.m_loc_pos_x_pre;
        int8_t delta_y = current_pos_y - G_referee.m_loc_pos_y_pre;
        if(G_referee.m_loc_queue_len >= REFEREE_MAP_SIZE){
            G_referee.m_loc_delta_x_total -= *(G_referee.MapSentryData.delta_x);
            G_referee.m_loc_delta_y_total -= *(G_referee.MapSentryData.delta_y);
            for(int8_t i = 0; i < REFEREE_MAP_SIZE - 1; i++){
                *(G_referee.MapSentryData.delta_x + i) = *(G_referee.MapSentryData.delta_x + i + 1);
                *(G_referee.MapSentryData.delta_y + i) = *(G_referee.MapSentryData.delta_y + i + 1);
            }
            G_referee.m_loc_queue_len--;
        }
        *(G_referee.MapSentryData.delta_x + G_referee.m_loc_queue_len) = delta_x;
        *(G_referee.MapSentryData.delta_y + G_referee.m_loc_queue_len) = delta_y;
        G_referee.m_loc_delta_x_total += delta_x;
        G_referee.m_loc_delta_y_total += delta_y;
        G_referee.m_loc_queue_len++;
        G_referee.MapSentryData.start_position_x = current_pos_x - G_referee.m_loc_delta_x_total;
        G_referee.MapSentryData.start_position_y = current_pos_y - G_referee.m_loc_delta_y_total;
        G_referee.m_loc_pos_x_pre = current_pos_x;
        G_referee.m_loc_pos_y_pre = current_pos_y;
        G_referee.m_loc_update_cnt = 0;
    }
    G_referee.m_loc_send_cnt++;
    if(G_referee.m_loc_send_cnt >= 100){
        memcpy(&G_referee.m_data_send_buff[7],&G_referee.MapSentryData,sizeof(G_referee.MapSentryData));
        Append_CRC16_Check_Sum(G_referee.m_data_send_buff,REFEREE_SEND_SIZE);
        G_referee.SendData();
        G_referee.m_loc_send_cnt = 0;
    }
}



/**
 *@brief update system control mode
 *
 *@param
 */
void ControlModeUpdate(void)
{
    if(G_system_monitor.UART4_rx_fps > 900)
    {
        if(G_gimbal.m_data_receive_frame.m_cdata[0] > 0.5 && G_gimbal.m_data_receive_frame.m_cdata[0] < 1.5)//1
            G_sentry.SetChassisMode(SentryRobot::CHASSIS_MANNAL);
        else if (G_gimbal.m_data_receive_frame.m_cdata[0] > 1.5)
            G_sentry.SetChassisMode(SentryRobot::CHASSIS_AUTO);
        else    
            G_sentry.SetChassisMode(SentryRobot::CHASSIS_SAFE);

        if(G_gimbal.m_data_receive_frame.m_cdata[1] > 0.5 && G_gimbal.m_data_receive_frame.m_cdata[1] < 1.5)
            G_sentry.SetYawGimbalMode(SentryRobot::YAWGIMBAL_MOVE);
        else if (G_gimbal.m_data_receive_frame.m_cdata[1] > 1.5)
            G_sentry.SetYawGimbalMode(SentryRobot::YAWGIMBAL_CALIBRATE);
        else
            G_sentry.SetYawGimbalMode(SentryRobot::YAWGIMBAL_SAFE);
    }
    else
    {
        G_sentry.SetChassisMode(SentryRobot::CHASSIS_SAFE);
        G_sentry.SetYawGimbalMode(SentryRobot::YAWGIMBAL_SAFE);
    }
        
}



/**
 *@brief update the state of the sentry robot
 *
 *@param
 */
void RobotStatesUpdate(void)
{
    // update sentry robot current chassis states
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->SpeedUpdate(1, 1);
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->SpeedUpdate(1, 1);
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->SpeedUpdate(1, 1);
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->SpeedUpdate(1, 1);

    G_sentry.m_yaw_angle = (G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_angle_current_encoder_filter + 
                            G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_angle_current_encoder_filter)/2.0f;

    if (G_system_monitor.UART4_rx_fps > 900)
    {
        G_sentry.m_world_vx = G_gimbal.m_data_receive_frame.m_fdata[0];
        G_sentry.m_world_vy = G_gimbal.m_data_receive_frame.m_fdata[1];
        G_sentry.m_chassis_w = G_gimbal.m_data_receive_frame.m_fdata[2];
        G_sentry.m_world2chassis_angle = G_sentry.m_yaw_angle - G_gimbal.m_data_receive_frame.m_fdata[3];

        if(G_sentry.m_chassis_w > 1e-6 && G_sentry.ctrl_spinning_flag == true) 
            G_sentry.spinning_flag = true;
        else if(G_sentry.m_chassis_w < 1e-6)
            G_sentry.spinning_flag = false;
        
        if(G_sentry.yawgimbal_mode != SentryRobot::YAWGIMBAL_SAFE && G_sentry.yawgimbal_mode != SentryRobot::YAWGIMBAL_CALIBRATE)
        {
            G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_speed_pid->m_output
            = G_gimbal.m_data_receive_frame.m_sdata[0];

            G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_speed_pid->m_output
            = G_gimbal.m_data_receive_frame.m_sdata[0];
        }
        else
        {
            G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_speed_pid->m_output = 0;
            G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_speed_pid->m_output = 0;
        }

        G_sentry.yaw_calibrate_flag_pre = G_sentry.yaw_calibrate_flag;
        G_sentry.yaw_calibrate_flag = G_gimbal.m_data_receive_frame.m_cdata[2];
        
    }
    else
    {
        G_sentry.m_world_vx = 0;
        G_sentry.m_world_vy = 0;
        G_sentry.m_chassis_w = 0;
        G_sentry.m_world2chassis_angle = 0;
        G_sentry.spinning_flag = false;
        G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_speed_pid->m_output = 0;
        G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_speed_pid->m_output = 0;
        G_sentry.yaw_calibrate_flag = 0;
        G_sentry.yaw_calibrate_flag_pre = 0;
    }

    

    // Update the capacitor power charging value
    if(G_system_monitor.UART2_rx_fps > 50)
    {
        // cap update
        float power_now = G_referee.PowerHeatData.chassis_power;
        float power_limit = G_referee.GameRobotStatus.chassis_power_limit;
        float power_buffer_now = G_referee.PowerHeatData.chassis_power_buffer;

        G_sentry.capacitor->UpdateChargingPower(power_now, power_buffer_now, power_limit);
        G_sentry.capacitor->PowerJudgeByVol();
        G_sentry.capacitor->PowerDesUpdate(power_limit);
    }
    
    // key command from referee
    if(G_referee.RobotCommand.commd_keyboard == PRESS_F){
        G_sentry.spinning_flag = true;
        G_sentry.ctrl_spinning_flag = true;
    }
    if(G_referee.RobotCommand.commd_keyboard == PRESS_G){
        G_sentry.spinning_flag = false;
        G_sentry.ctrl_spinning_flag = false;
    }
    if(G_referee.RobotCommand.commd_keyboard == PRESS_Z)  G_sentry.vision_flag = true;
    if(G_referee.RobotCommand.commd_keyboard == PRESS_X)  G_sentry.vision_flag = false;
    if(G_referee.RobotCommand.commd_keyboard == PRESS_C)  G_sentry.safe_flag = true;
    if(G_referee.RobotCommand.commd_keyboard == PRESS_V)  G_sentry.safe_flag = false;

    if(G_sentry.yawgimbal_mode == SentryRobot::YAWGIMBAL_CALIBRATE)
        YawGimbalCalibrate();
}



void YawGimbalCalibrate(void)
{
    static int32_t yaw_left_encoder = 0;
    static int32_t yaw_right_encoder = 0;
    static int32_t yaw_left_encoder_pre = 0;
    static int32_t yaw_right_encoder_pre = 0;
    static uint8_t calibrate_times = 0;
    static uint8_t first_flag = 0;

    float yaw_left_reduction_ratio = 0;
    float yaw_right_reducion_ratio = 0;

    if(G_sentry.yaw_calibrate_flag && !G_sentry.yaw_calibrate_flag_pre)
    {
        yaw_left_encoder_pre = yaw_left_encoder;
        yaw_right_encoder_pre = yaw_right_encoder;

        yaw_left_encoder = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_encoder->m_sum_value;
        yaw_right_encoder = G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_encoder->m_sum_value;

        calibrate_times++;

        if(!first_flag)
        {
            G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_encoder->m_zero_value 
            = G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_encoder->m_sum_value;

            G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_encoder->m_zero_value 
            = G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_encoder->m_sum_value;

            first_flag = 1;
        }
    }

    if(calibrate_times >= 2)
    {
        yaw_left_reduction_ratio = fabs((float)(yaw_left_encoder - yaw_left_encoder_pre))/8192.0f;
        yaw_right_reducion_ratio = fabs((float)(yaw_right_encoder - yaw_right_encoder_pre))/8192.0f;

        G_sentry.yaw_gimbal_motor[SentryRobot::LEFT_MOTOR]->m_reduction_ratio = yaw_left_reduction_ratio;
        G_sentry.yaw_gimbal_motor[SentryRobot::RIGHT_MOTOR]->m_reduction_ratio = yaw_right_reducion_ratio;
    }

}



/**
 *@brief update the robot targets
 *
 *@param
 */
void RobotTargetsUpdate(void)
{
    ChassisTargetsUpdate();
}



/**
 *@brief update the robot chassis targets
 *
 *@param
 */
void ChassisTargetsUpdate(void)
{
#ifdef CHASSIS_STEER_DRIVING_MODE
    SteerDriveChassisTargetsUpdate();

#elif defined CHASSIS_STANDARD_DRIVING_MODE
    StandardDriveChassisTargetsUpdate();

#elif defined CHASSIS_DIFFERENTIAL_DRIVING_MODE
    DiffDriveChassisTargetsUpdate();

#elif defined CHASSIS_COMMOM_DRIVING_MODE
    CommonDriveChassisTargetsUpdate();
#elif defined CHASSIS_OMNIDIRECTIONAL_MODE
    OmnidirectionalChassisTargetUpdate();
#endif
}



void OmnidirectionalChassisTargetUpdate(void)
{
    float frl_speed_des = 0;
    float brl_speed_des = 0;
    float bll_speed_des = 0;
    float fll_speed_des = 0;

    if(G_sentry.chassis_mode == SentryRobot::CHASSIS_SAFE)
    {
        for (uint8_t i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
        {
            G_sentry.chassis_line_motor[i]->m_speed_target = G_sentry.chassis_line_motor[i]->m_speed_current;
        }
    }

    else
    {   
        if(G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO)
        {
            if(G_sentry.spinning_flag)
                G_sentry.m_chassis_w = CHASSIS_SPIN_SPEED_MAX;
            else 
                G_sentry.m_chassis_w = 0.0;
        }

        frl_speed_des = -(G_sentry.m_world_vy*cosf((225.0f - G_sentry.m_world2chassis_angle)/RADIAN2DEGREE_VALUE) 
                         + G_sentry.m_world_vx*cosf((G_sentry.m_world2chassis_angle - 135.0f)/RADIAN2DEGREE_VALUE) - G_sentry.m_chassis_w)/CHASSIS_WHEEL_RADIUS* RADIAN2DEGREE_VALUE;
        brl_speed_des = -(G_sentry.m_world_vy*cosf((G_sentry.m_world2chassis_angle - 135.0f)/RADIAN2DEGREE_VALUE) 
                        -G_sentry.m_world_vx*cosf((225.0f - G_sentry.m_world2chassis_angle)/RADIAN2DEGREE_VALUE) - G_sentry.m_chassis_w)/CHASSIS_WHEEL_RADIUS* RADIAN2DEGREE_VALUE;
        bll_speed_des = (G_sentry.m_world_vy*cosf((225.0f - G_sentry.m_world2chassis_angle)/RADIAN2DEGREE_VALUE) 
                        + G_sentry.m_world_vx*cosf((G_sentry.m_world2chassis_angle - 135.0f)/RADIAN2DEGREE_VALUE) + G_sentry.m_chassis_w)/CHASSIS_WHEEL_RADIUS* RADIAN2DEGREE_VALUE;
        fll_speed_des = (G_sentry.m_world_vy*cosf((G_sentry.m_world2chassis_angle - 135.0f)/RADIAN2DEGREE_VALUE) 
                        -G_sentry.m_world_vx*cosf((225.0f - G_sentry.m_world2chassis_angle)/RADIAN2DEGREE_VALUE) + G_sentry.m_chassis_w)/CHASSIS_WHEEL_RADIUS* RADIAN2DEGREE_VALUE;

        G_sentry.SetChassisSpeedTarget(fll_speed_des,frl_speed_des,bll_speed_des,brl_speed_des);
    }
    
}


/**
 *@brief execute the robot control
 *
 *@param
 */
void RobotControlExecute(void)
{
    // Execute the robot chassis control algorithm
    G_sentry.ExecuteChassisAlgorithm();

    // Send the control commands to the actuators
    G_sentry.SendControlCommand();
}





