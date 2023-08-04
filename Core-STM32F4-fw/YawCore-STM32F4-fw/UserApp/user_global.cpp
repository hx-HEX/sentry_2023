#include "user_global.h"

SystemMonitor G_system_monitor;
RefereeMonitor G_referee_monitor;
Led G_led;
Left_Gimbal G_left_gimbal;
Right_Gimbal G_right_gimbal;
Yaw_Gimbal G_yaw_gimbal;
Chassis_Core G_chassis_core;
DJIRC G_djirc;
Vofa G_vofa;
Navigation G_navigation;
Referee G_referee;
SentryRobot G_sentry;
ControlMode G_control_mode;

/**
 *@brief Remote control state monitor
 *
 *@param
 */
void RemoteControlMonitor(void)
{
    if (G_system_monitor.UART1_rx_fps < 50)
    {
        G_djirc.channel.Ch0 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch1 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch2 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch3 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch4 = RC_CH_VALUE_OFFSET;
    }
}

/**
 *@brief Motor state monitor
 *
 *@param
 */
void MotorMonitor(void)
{
    static uint32_t times = 0;
    times++;
    if (G_system_monitor.CAN1_rx_fps < 2000 && G_system_monitor.CAN2_rx_fps < 2000)
    {
        if (times % 3 == 0)
        {
            G_led.ToggleGreen();
        }
    }
    else if (G_system_monitor.CAN2_rx_fps < 2000)
    {
        if (times % 2 == 0)
        {
            G_led.ToggleGreen();
        }
    }
    else if (G_system_monitor.CAN1_rx_fps < 2000)
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
    if (G_system_monitor.UART3_rx_fps < 50)
    {
        if (times % 10 < 3)
        {
            G_led.SetRed(true);
            G_led.SetBlue(false);
            G_led.SetGreen(false);
        }
    }
    if (G_system_monitor.UART6_rx_fps < 50)
    {
        if (times % 10 >= 3 && times % 10 <= 6)
        {
            G_led.SetBlue(true);
            G_led.SetRed(false);
            G_led.SetGreen(false);
        }
    }
    if (G_system_monitor.UART6_rx_fps < 500)
    {
        if (times % 10 > 6)
        {
            G_led.SetGreen(true);
            G_led.SetRed(false);
            G_led.SetBlue(false);
        }
    }
}

/**
 *@brief send navigation datas
 *
 *@param
 */
void SendNavigationData(void)
{
    G_navigation.m_data_send_frame.m_data[2] = G_sentry.m_robot_chassis_speed;
    G_navigation.m_data_send_frame.m_data[3] = G_sentry.m_robot_wheel_world_yaw; // warning

    G_navigation.m_data_send_frame.m_data[4] = G_left_gimbal.m_data_receive_frame.m_fdata[0];
    G_navigation.m_data_send_frame.m_data[5] = G_left_gimbal.m_data_receive_frame.m_fdata[1];
    G_navigation.m_data_send_frame.m_data[6] = G_left_gimbal.m_data_receive_frame.m_fdata[2];
    G_navigation.m_data_send_frame.m_data[7] = G_left_gimbal.m_data_receive_frame.m_fdata[3];
    G_navigation.m_data_send_frame.m_data[8] = G_left_gimbal.m_data_receive_frame.m_fdata[4];

    uint32_t navigation_data_num = sizeof(G_navigation.m_data_send_frame);
    if (G_navigation.navigation_send_buff_cnt + navigation_data_num + NAVIGATION_SEND_HEAD_SIZE < NAVIGATION_SEND_BUFF_SIZE)
    {
        G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = 0xAB;
        G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = 0xA0;
        G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = navigation_data_num;
        memcpy(&G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt], &G_navigation.m_data_send_frame, navigation_data_num);
        G_navigation.navigation_send_buff_cnt += navigation_data_num;
    }
    G_navigation.usart_navigation->USART_RT.txlen = G_navigation.navigation_send_buff_cnt;
    G_navigation.SendData();
    G_navigation.navigation_send_buff_cnt = 0;

    G_navigation.m_data_send_frame.m_id = 0x00;
}

/**
 *@brief send navigation datas
 *
 *@param
 */
void CompetitionStateUpdate(void)
{
    if (G_referee.GameRobotStatus.robot_id == ROBOT_ID_SENTRY_RED)
    {
        if (G_referee.m_enemy_1_HP_pre == 0 && G_referee.GameRobotHP.blue_1_robot_HP != 0)
        {
            G_sentry.m_enemy_1_res_flag = true;
        }
        if (G_referee.m_enemy_2_HP_pre == 0 && G_referee.GameRobotHP.blue_2_robot_HP != 0)
        {
            G_sentry.m_enemy_2_res_flag = true;
        }
        if (G_referee.m_enemy_3_HP_pre == 0 && G_referee.GameRobotHP.blue_3_robot_HP != 0)
        {
            G_sentry.m_enemy_3_res_flag = true;
        }
        if (G_referee.m_enemy_4_HP_pre == 0 && G_referee.GameRobotHP.blue_4_robot_HP != 0)
        {
            G_sentry.m_enemy_4_res_flag = true;
        }
        if (G_referee.m_enemy_5_HP_pre == 0 && G_referee.GameRobotHP.blue_5_robot_HP != 0)
        {
            G_sentry.m_enemy_5_res_flag = true;
        }
        if (G_referee.GameRobotHP.blue_outpost_HP == 0)
        {
            G_sentry.m_enemy_7_res_flag = false;
        }
        else
        {
            G_sentry.m_enemy_7_res_flag = true;
        }
        if (G_sentry.m_enemy_1_res_flag == true && G_sentry.m_enemy_1_res_cnt <= 10000)
        {
            G_sentry.m_enemy_1_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_1_res_flag = false;
            G_sentry.m_enemy_1_res_cnt = 0;
        }
        if (G_sentry.m_enemy_2_res_flag == true && G_sentry.m_enemy_2_res_cnt <= 10000)
        {
            G_sentry.m_enemy_2_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_2_res_flag = false;
            G_sentry.m_enemy_2_res_cnt = 0;
        }
        if (G_sentry.m_enemy_3_res_flag == true && G_sentry.m_enemy_3_res_cnt <= 10000)
        {
            G_sentry.m_enemy_3_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_3_res_flag = false;
            G_sentry.m_enemy_3_res_cnt = 0;
        }
        if (G_sentry.m_enemy_4_res_flag == true && G_sentry.m_enemy_4_res_cnt <= 10000)
        {
            G_sentry.m_enemy_4_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_4_res_flag = false;
            G_sentry.m_enemy_4_res_cnt = 0;
        }
        if (G_sentry.m_enemy_5_res_flag == true && G_sentry.m_enemy_5_res_cnt <= 10000)
        {
            G_sentry.m_enemy_5_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_5_res_flag = false;
            G_sentry.m_enemy_5_res_cnt = 0;
        }
        G_referee.m_enemy_1_HP_pre = G_referee.GameRobotHP.blue_1_robot_HP;
        G_referee.m_enemy_2_HP_pre = G_referee.GameRobotHP.blue_2_robot_HP;
        G_referee.m_enemy_3_HP_pre = G_referee.GameRobotHP.blue_3_robot_HP;
        G_referee.m_enemy_4_HP_pre = G_referee.GameRobotHP.blue_4_robot_HP;
        G_referee.m_enemy_5_HP_pre = G_referee.GameRobotHP.blue_5_robot_HP;
        if (G_referee.GameStatus.game_progress == 4 && !G_sentry.balance_infantry_flag)
        {
            G_sentry.balance_infantry_flag = true;
            if (G_referee.GameRobotHP.blue_3_robot_HP == 300)
                G_sentry.balance_infantry_num += 3;
            if (G_referee.GameRobotHP.blue_4_robot_HP == 300)
                G_sentry.balance_infantry_num += 4;
            if (G_referee.GameRobotHP.blue_5_robot_HP == 300)
                G_sentry.balance_infantry_num += 5;
        }
        if (G_referee.GameStatus.game_progress != 4)
        {
            G_sentry.balance_infantry_flag = false;
            G_sentry.balance_infantry_num = 0;
        }
    }
    else
    {
        if (G_referee.m_enemy_1_HP_pre == 0 && G_referee.GameRobotHP.red_1_robot_HP != 0)
        {
            G_sentry.m_enemy_1_res_flag = true;
        }
        if (G_referee.m_enemy_2_HP_pre == 0 && G_referee.GameRobotHP.red_2_robot_HP != 0)
        {
            G_sentry.m_enemy_2_res_flag = true;
        }
        if (G_referee.m_enemy_3_HP_pre == 0 && G_referee.GameRobotHP.red_3_robot_HP != 0)
        {
            G_sentry.m_enemy_3_res_flag = true;
        }
        if (G_referee.m_enemy_4_HP_pre == 0 && G_referee.GameRobotHP.red_4_robot_HP != 0)
        {
            G_sentry.m_enemy_4_res_flag = true;
        }
        if (G_referee.m_enemy_5_HP_pre == 0 && G_referee.GameRobotHP.red_5_robot_HP != 0)
        {
            G_sentry.m_enemy_5_res_flag = true;
        }
        if (G_referee.GameRobotHP.red_outpost_HP == 0)
        {
            G_sentry.m_enemy_7_res_flag = false;
        }
        else
        {
            G_sentry.m_enemy_7_res_flag = true;
        }
        if (G_sentry.m_enemy_1_res_flag == true && G_sentry.m_enemy_1_res_cnt <= 10000)
        {
            G_sentry.m_enemy_1_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_1_res_flag = false;
            G_sentry.m_enemy_1_res_cnt = 0;
        }
        if (G_sentry.m_enemy_2_res_flag == true && G_sentry.m_enemy_2_res_cnt <= 10000)
        {
            G_sentry.m_enemy_2_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_2_res_flag = false;
            G_sentry.m_enemy_2_res_cnt = 0;
        }
        if (G_sentry.m_enemy_3_res_flag == true && G_sentry.m_enemy_3_res_cnt <= 10000)
        {
            G_sentry.m_enemy_3_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_3_res_flag = false;
            G_sentry.m_enemy_3_res_cnt = 0;
        }
        if (G_sentry.m_enemy_4_res_flag == true && G_sentry.m_enemy_4_res_cnt <= 10000)
        {
            G_sentry.m_enemy_4_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_4_res_flag = false;
            G_sentry.m_enemy_4_res_cnt = 0;
        }
        if (G_sentry.m_enemy_5_res_flag == true && G_sentry.m_enemy_5_res_cnt <= 10000)
        {
            G_sentry.m_enemy_5_res_cnt++;
        }
        else
        {
            G_sentry.m_enemy_5_res_flag = false;
            G_sentry.m_enemy_5_res_cnt = 0;
        }
        G_referee.m_enemy_1_HP_pre = G_referee.GameRobotHP.red_1_robot_HP;
        G_referee.m_enemy_2_HP_pre = G_referee.GameRobotHP.red_2_robot_HP;
        G_referee.m_enemy_3_HP_pre = G_referee.GameRobotHP.red_3_robot_HP;
        G_referee.m_enemy_4_HP_pre = G_referee.GameRobotHP.red_4_robot_HP;
        G_referee.m_enemy_5_HP_pre = G_referee.GameRobotHP.red_5_robot_HP;
        if (G_referee.GameStatus.game_progress == 4 && !G_sentry.balance_infantry_flag)
        {
            G_sentry.balance_infantry_flag = true;
            if (G_referee.GameRobotHP.red_3_robot_HP == 300)
                G_sentry.balance_infantry_num += 3;
            if (G_referee.GameRobotHP.red_4_robot_HP == 300)
                G_sentry.balance_infantry_num += 4;
            if (G_referee.GameRobotHP.red_5_robot_HP == 300)
                G_sentry.balance_infantry_num += 5;
        }
        if (G_referee.GameStatus.game_progress != 4)
        {
            G_sentry.balance_infantry_flag = false;
            G_sentry.balance_infantry_num = 0;
        }
    }
}

/**
 *@brief send gimabl datas
 *
 *@param
 */
void SendYawGimbalData(void)
{
    if (G_sentry.yawgimbal_mode == SentryRobot::YAW_SAFE)
        G_yaw_gimbal.m_data_send_frame.m_cdata[0] = 0;
    else if (G_sentry.yawgimbal_mode == SentryRobot::YAW_MOVE)
        G_yaw_gimbal.m_data_send_frame.m_cdata[0] = 1;
    else
        G_yaw_gimbal.m_data_send_frame.m_cdata[0] = 2;

    G_yaw_gimbal.m_data_send_frame.m_fdata[0] = G_sentry.yaw_des;
    G_yaw_gimbal.m_data_send_frame.m_fdata[1] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[2] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[3] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[4] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[5] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[6] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[7] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[8] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[9] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[10] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[11] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[12] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[13] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[14] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[15] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[16] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[17] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[18] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[19] = 0;
    G_yaw_gimbal.m_data_send_frame.m_fdata[20] = 0;

    Append_CRC16_Check_Sum((uint8_t *)&G_yaw_gimbal.m_data_send_frame, YAW_GIMBAL_DATA_SEND_SIZE);
    G_yaw_gimbal.SendData();
}

void SendLeftGimbalData(void)
{
    G_left_gimbal.m_data_send_frame.m_cdata[0] = G_sentry.left_shoot_insurance;
    G_left_gimbal.m_data_send_frame.m_cdata[1] = G_sentry.left_run_flag;
    G_left_gimbal.m_data_send_frame.m_cdata[2] = G_sentry.balance_infantry_num;
    if (G_referee.GameRobotStatus.robot_id == ROBOT_ID_SENTRY_RED)
        G_left_gimbal.m_data_send_frame.m_cdata[3] = 0x09;
    else if (G_referee.GameRobotStatus.robot_id == ROBOT_ID_SENTRY_BLUE)
        G_left_gimbal.m_data_send_frame.m_cdata[3] = 0x0A;
    G_left_gimbal.m_data_send_frame.m_cdata[4] = G_sentry.m_scan_flag;

    G_left_gimbal.m_data_send_frame.m_fdata[0] = G_sentry.left_pitch_des;
    G_left_gimbal.m_data_send_frame.m_fdata[1] = G_sentry.left_yaw_des;
    G_left_gimbal.m_data_send_frame.m_fdata[2] = G_referee.ShootData_1.bullet_speed;

    Append_CRC16_Check_Sum((uint8_t *)&G_left_gimbal.m_data_send_frame, LEFT_GIMBAL_DATA_SEND_SIZE);
    G_left_gimbal.SendData();
}

void SendRightGimbalData(void)
{
    G_right_gimbal.m_data_send_frame.m_cdata[0] = G_sentry.right_shoot_insurance;
    G_right_gimbal.m_data_send_frame.m_cdata[1] = G_sentry.right_run_flag;
    G_right_gimbal.m_data_send_frame.m_cdata[2] = G_sentry.balance_infantry_num;
    if (G_referee.GameRobotStatus.robot_id == ROBOT_ID_SENTRY_RED)
        G_right_gimbal.m_data_send_frame.m_cdata[3] = 0x09;
    else if (G_referee.GameRobotStatus.robot_id == ROBOT_ID_SENTRY_BLUE)
        G_right_gimbal.m_data_send_frame.m_cdata[3] = 0x0A;
    G_right_gimbal.m_data_send_frame.m_cdata[4] = G_sentry.m_scan_flag;

    G_right_gimbal.m_data_send_frame.m_fdata[0] = G_sentry.right_pitch_des;
    G_right_gimbal.m_data_send_frame.m_fdata[1] = G_sentry.right_yaw_des;
    G_left_gimbal.m_data_send_frame.m_fdata[2] = G_referee.ShootData_2.bullet_speed;

    Append_CRC16_Check_Sum((uint8_t *)&G_right_gimbal.m_data_send_frame, RIGHT_GIMBAL_DATA_SEND_SIZE);
    G_right_gimbal.SendData();
}

void SendChassisCoreData(void)
{
    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_SAFE)
        G_chassis_core.m_data_send_frame.m_cdata[0] = 0;
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL)
        G_chassis_core.m_data_send_frame.m_cdata[0] = 1;
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO)
        G_chassis_core.m_data_send_frame.m_cdata[0] = 2;

    if (G_sentry.yawgimbal_mode == SentryRobot::YAW_SAFE)
        G_chassis_core.m_data_send_frame.m_cdata[1] = 0;
    else if (G_sentry.yawgimbal_mode == SentryRobot::YAW_MOVE)
        G_chassis_core.m_data_send_frame.m_cdata[1] = 1;
    else if (G_sentry.yawgimbal_mode == SentryRobot::YAW_STATIC)
        G_chassis_core.m_data_send_frame.m_cdata[1] = 2;
    else if (G_sentry.yawgimbal_mode == SentryRobot::YAW_CALIBRATE)
        G_chassis_core.m_data_send_frame.m_cdata[1] = 3;

    G_chassis_core.m_data_send_frame.m_cdata[2] = G_sentry.photogate->calibrate_flag;

    G_chassis_core.m_data_send_frame.m_sdata[0] = G_sentry.leftyaw_output;
    G_chassis_core.m_data_send_frame.m_sdata[1] = G_sentry.rightyaw_output;

    G_chassis_core.m_data_send_frame.m_fdata[0] = G_sentry.m_world_vx;
    G_chassis_core.m_data_send_frame.m_fdata[1] = G_sentry.m_world_vy;
    G_chassis_core.m_data_send_frame.m_fdata[2] = G_sentry.m_chassis_w;
    G_chassis_core.m_data_send_frame.m_fdata[3] = G_sentry.m_radar_world_yaw_angle;
    G_chassis_core.m_data_send_frame.m_fdata[4] = G_sentry.m_navigation_x;
    G_chassis_core.m_data_send_frame.m_fdata[5] = G_sentry.m_navigation_y;
    G_chassis_core.m_data_send_frame.m_fdata[5] = G_sentry.yaw_des;

    Append_CRC16_Check_Sum((uint8_t *)&G_chassis_core.m_data_send_frame, CHASSIS_CORE_DATA_SEND_SIZE);
    G_chassis_core.SendData();
}

/**
 *@brief update system control mode
 *
 *@param
 */
void ControlModeUpdate(void)
{
    // Control mode update
    if (G_system_monitor.UART1_rx_fps < 5)
    {
        if (G_referee.GameStatus.game_progress == 4)
        {
            G_control_mode = AUTO_CGS;
            if (G_sentry.vision_flag)
                G_control_mode = AUTO_GS_RC_C;
            else
                G_control_mode = AUTO_CGS;
            if (G_sentry.safe_flag)
                G_control_mode = SAFE;
            else
                G_control_mode = AUTO_CGS;

            G_sentry.left_shoot_insurance = false;
            G_sentry.right_shoot_insurance = false;

            if (G_sentry.m_shoot_cnt < 2500)
                G_sentry.m_scan_flag = false;
            else
                G_sentry.m_scan_flag = true;

            if (G_sentry.m_shoot_cnt < 3000)
                G_sentry.m_shoot_cnt++;
            else
                G_sentry.m_shoot_cnt = 3000;
        }
        else
        {
            G_sentry.m_shoot_cnt = 0;
            G_control_mode = SAFE;
            G_sentry.m_scan_flag = true;
        }
    }
    else
    {
        if (G_djirc.channel.SW_L == RC_SW_DOWN && G_djirc.channel.SW_R == RC_SW_DOWN)
        {
            G_control_mode = SAFE;
        }
        else if (G_djirc.channel.SW_L == RC_SW_DOWN && G_djirc.channel.SW_R == RC_SW_MID)
        {
            G_control_mode = RC_GS;
        }
        else if (G_djirc.channel.SW_L == RC_SW_DOWN && G_djirc.channel.SW_R == RC_SW_UP)
        {
            G_control_mode = AUTO_G_RC_S;
        }
        else if (G_djirc.channel.SW_L == RC_SW_MID && G_djirc.channel.SW_R == RC_SW_UP)
        {
            G_control_mode = AUTO_GS_RC_C;
        }
        else if (G_djirc.channel.SW_L == RC_SW_UP && G_djirc.channel.SW_R == RC_SW_UP)
        {
            G_control_mode = CALIBRATE;
        }
        else if (G_djirc.channel.SW_L == RC_SW_MID && G_djirc.channel.SW_R == RC_SW_MID)
        {
            G_control_mode = RC_C;
        }
        else if (G_djirc.channel.SW_L == RC_SW_MID && G_djirc.channel.SW_R == RC_SW_DOWN)
        {
            G_control_mode = AUTO_CGS;
        }
        else if (G_djirc.channel.SW_L == RC_SW_UP && G_djirc.channel.SW_R == RC_SW_DOWN)
        {
            G_control_mode = AUTO_G_RC_C;
        }
        else
        {
            G_control_mode = SAFE;
        }
    }

    // Set sentry robot gimbal mode
    if (G_control_mode == AUTO_G_RC_S || G_control_mode == AUTO_CGS || G_control_mode == AUTO_GS_RC_C || G_control_mode == AUTO_G_RC_C)
    {
        G_sentry.SetGimbalMode(SentryRobot::GIMBAL_AUTO);
    }
    else if (G_control_mode == RC_GS)
    {
        G_sentry.SetGimbalMode(SentryRobot::GIMBAL_MANNAL);
    }
    else
    {
        G_sentry.SetGimbalMode(SentryRobot::GIMBAL_SAFE);
    }

    // Set sentry robot shoot mode
    if ((G_control_mode == AUTO_GS_RC_C || G_control_mode == AUTO_CGS))
    {
        G_sentry.SetShootMode(SentryRobot::SHOOT_AUTO);
    }
    else if (G_control_mode == RC_GS || G_control_mode == AUTO_G_RC_S)
    {
        G_sentry.SetShootMode(SentryRobot::SHOOT_MANNAL);
    }
    else
    {
        G_sentry.SetShootMode(SentryRobot::SHOOT_SAFE);
    }

    // Set sentry robot chassis mode
    if (G_control_mode == AUTO_CGS)
    {
        G_sentry.SetChassisMode(SentryRobot::CHASSIS_AUTO);
    }
    else if (G_control_mode == AUTO_GS_RC_C || G_control_mode == RC_C || G_control_mode == AUTO_G_RC_C || G_control_mode == CALIBRATE)
    {
        G_sentry.SetChassisMode(SentryRobot::CHASSIS_MANNAL);
    }
    else
    {
        G_sentry.SetChassisMode(SentryRobot::CHASSIS_SAFE);
    }

    if (G_control_mode == SAFE)
    {
        G_sentry.SetYawGimbalMode(SentryRobot::YAW_SAFE);
    }
    else if (G_control_mode == RC_GS)
    {
        G_sentry.SetYawGimbalMode(SentryRobot::YAW_MOVE);
    }
    else if (G_control_mode == CALIBRATE)
    {
        G_sentry.SetYawGimbalMode(SentryRobot::YAW_CALIBRATE);
    }
    else
    {
        G_sentry.SetYawGimbalMode(SentryRobot::YAW_STATIC);
    }
}

/**
 *@brief update the state of the sentry robot
 *
 *@param
 */
void RobotStatesUpdate(void)
{
    G_sentry.shoot_motor[SentryRobot::LEFT_SHOOT_DRIVE_MOTOR]->AngleUpdate(0, 1);
    G_sentry.shoot_motor[SentryRobot::LEFT_SHOOT_DRIVE_MOTOR]->SpeedUpdate(0, 1);
    G_sentry.shoot_motor[SentryRobot::RIGHT_SHOOT_DRIVE_MOTOR]->AngleUpdate(0, 1);
    G_sentry.shoot_motor[SentryRobot::RIGHT_SHOOT_DRIVE_MOTOR]->SpeedUpdate(0, 1);

    G_sentry.right_pitch_fb = G_right_gimbal.m_data_receive_frame.m_fdata[5];
    G_sentry.right_yaw_fb = G_right_gimbal.m_data_receive_frame.m_fdata[6];

    G_sentry.left_pitch_fb = G_left_gimbal.m_data_receive_frame.m_fdata[5];
    G_sentry.left_yaw_fb = G_left_gimbal.m_data_receive_frame.m_fdata[6];

    G_sentry.yaw_fb = G_yaw_gimbal.m_data_receive_frame.m_fdata[0];
    G_sentry.photogate->calibrate_flag = G_sentry.photogate->Get_Photogate_State();

    if (G_system_monitor.UART5_rx_fps > 70)
    {
        G_sentry.m_radar_world_yaw_angle = G_navigation.m_data_receive_frame.m_data_f[1] * RADIAN2DEGREE_VALUE;
        G_sentry.m_navigation_x = G_navigation.m_data_receive_frame.m_data_f[7];
        G_sentry.m_navigation_y = G_navigation.m_data_receive_frame.m_data_f[8];
        G_sentry.spinning_flag = G_navigation.m_data_receive_frame.m_data_c[2];
    }
    else
    {
        G_sentry.m_radar_world_yaw_angle = 0;
        G_sentry.m_navigation_x = 0;
        G_sentry.m_navigation_y = 0;
        G_sentry.spinning_flag = 0;
    }

    if (G_system_monitor.UART2_rx_fps > 50)
    {
        G_sentry.m_robot_wheel_world_yaw = G_chassis_core.m_data_receive_frame.m_fdata[0] - G_sentry.m_radar_world_yaw_angle;
        G_sentry.m_robot_chassis_speed = G_chassis_core.m_data_receive_frame.m_fdata[1];
    }
    else
    {
        G_sentry.m_robot_wheel_world_yaw = 0;
        G_sentry.m_robot_chassis_speed = 0;
    }

    if (G_system_monitor.UART3_rx_fps > 900)
    {
        G_sentry.leftyaw_output = G_yaw_gimbal.m_data_receive_frame.m_sdata[0];
        G_sentry.rightyaw_output = G_yaw_gimbal.m_data_receive_frame.m_sdata[1];
    }
    else
    {
        G_sentry.leftyaw_output = 0;
        G_sentry.rightyaw_output = 0;
    }

    if (G_referee.RobotCommand.commd_keyboard == PRESS_Z)
        G_sentry.vision_flag = true;
    if (G_referee.RobotCommand.commd_keyboard == PRESS_X)
        G_sentry.vision_flag = false;
    if (G_referee.RobotCommand.commd_keyboard == PRESS_C)
        G_sentry.safe_flag = true;
    if (G_referee.RobotCommand.commd_keyboard == PRESS_V)
        G_sentry.safe_flag = false;

    // gun shift strategy
    if ((G_referee.PowerHeatData.shooter_id1_17mm_cooling_heat > G_referee.GameRobotStatus.shooter_id1_17mm_cooling_limit - 40) && G_referee.GameRobotStatus.shooter_id1_17mm_cooling_limit != 0)
    {
        G_sentry.left_stop_shoot_flag = 1;
    }
    if (G_sentry.left_stop_shoot_flag)
        G_sentry.m_left_stop_shoot_cnt++;
    if (G_sentry.m_left_stop_shoot_cnt > 2500)
    {
        G_sentry.left_stop_shoot_flag = 0;
        G_sentry.m_left_stop_shoot_cnt = 0;
    }

    if ((G_referee.PowerHeatData.shooter_id2_17mm_cooling_heat > G_referee.GameRobotStatus.shooter_id2_17mm_cooling_limit - 40) && G_referee.GameRobotStatus.shooter_id2_17mm_cooling_limit != 0)
    {
        G_sentry.right_stop_shoot_flag = 1;
    }
    if (G_sentry.right_stop_shoot_flag)
        G_sentry.m_right_stop_shoot_cnt++;
    if (G_sentry.m_right_stop_shoot_cnt > 2500)
    {
        G_sentry.right_stop_shoot_flag = 0;
        G_sentry.m_right_stop_shoot_cnt = 0;
    }

    // shoot speed filter update
    if (G_referee.ShootData_1.bullet_speed != G_sentry.m_shoot_speed_pre_1 && G_referee.ShootData_1.bullet_speed != 0)
    {
        G_sentry.m_shoot_speed_filter->Update(G_referee.ShootData_1.bullet_speed);
    }
    G_sentry.m_shoot_speed_pre_1 = G_referee.ShootData_1.bullet_speed;

    // shoot speed filter update
    if (G_referee.ShootData_2.bullet_speed != G_sentry.m_shoot_speed_pre_2 && G_referee.ShootData_2.bullet_speed != 0)
    {
        G_sentry.m_shoot_speed_filter->Update(G_referee.ShootData_2.bullet_speed);
    }
    G_sentry.m_shoot_speed_pre_2 = G_referee.ShootData_2.bullet_speed;
}

/**
 *@brief update the robot targets
 *
 *@param
 */
void RobotTargetsUpdate(void)
{
    ChassisTargetsUpdate();
    GimbalTargetsUpdate();
    ShootTargetsUpdate();
}

/**
 *@brief update the robot chassis targets
 *
 *@param
 */
void ChassisTargetsUpdate(void)
{
    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL)
    {
        if ((float)G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            G_sentry.m_world_vx = ((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        else if ((float)G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            G_sentry.m_world_vx = ((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        else
        {
            G_sentry.m_world_vx = 0;
        }
        if ((float)G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            G_sentry.m_world_vy = -((float)(G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        else if ((float)G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            G_sentry.m_world_vy = -((float)(G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        else
        {
            G_sentry.m_world_vy = 0;
        }
        if ((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            G_sentry.m_chassis_w = ((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        else if ((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            G_sentry.m_chassis_w = ((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        else
        {
            G_sentry.m_chassis_w = 0;
        }
    }
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO)
    {
        G_sentry.m_world_vx = G_navigation.m_data_receive_frame.m_data_f[0];
        G_sentry.m_world_vy = G_navigation.m_data_receive_frame.m_data_f[2];

        if (G_sentry.spinning_flag)
            G_sentry.m_chassis_w = CHASSIS_SPIN_SPEED_MAX;
        else
            G_sentry.m_chassis_w = 0;
    }
    else
    {
        G_sentry.m_world_vx = 0;
        G_sentry.m_world_vy = 0;
        G_sentry.m_chassis_w = 0;
    }
}

/**
 *@brief update the robot gimbal targets
 *
 *@param
 */
void GimbalTargetsUpdate(void)
{
    static uint16_t mannal_delay_cnt = 0;
    static uint16_t auto_delay_cnt = 0;
    static uint8_t mannal_change_flag = 0;
    static uint8_t change_flag_1 = 0;
    static uint8_t auto_change_flag = 0;
    static uint8_t change_flag_2 = 0;
    float gimbal_pitch_des_step = 0;
    float gimbal_yaw_des_step = 0;
    float yaw_des_step = 0;

    if (G_sentry.gimbal_mode == SentryRobot::GIMBAL_MANNAL)
    {
        if (G_sentry.mode_toggle_flag)
        {
            G_sentry.mode_toggle_flag = 0;
            G_sentry.left_pitch_des = G_sentry.left_pitch_fb;
            G_sentry.left_yaw_des = G_sentry.left_yaw_fb;
            G_sentry.right_pitch_des = G_sentry.right_pitch_fb;
            G_sentry.right_yaw_des = G_sentry.right_yaw_fb;
        }
        if (G_djirc.channel.Ch0 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch1 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch2 > (RC_CH_VALUE_MAX - RC_CH_VALUE_DEAD) && G_djirc.channel.Ch3 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && change_flag_1)
        {
            mannal_delay_cnt++;
            if (mannal_delay_cnt > 1000)
            {
                mannal_change_flag = 1;
                change_flag_1 = 0;
            }
        }
        else if (!(G_djirc.channel.Ch0 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch1 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch2 > (RC_CH_VALUE_MAX - RC_CH_VALUE_DEAD) && G_djirc.channel.Ch3 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD)))
        {
            mannal_delay_cnt = 0;
            change_flag_1 = 1;
        }

        if (mannal_change_flag && G_sentry.gimbalrc_mode == SentryRobot::RIGHT_MANNAL)
        {
            G_sentry.gimbalrc_mode = SentryRobot::LEFT_MANNAL;
            mannal_change_flag = 0;
            G_sentry.left2right_flag = 1;
        }
        else if (mannal_change_flag && G_sentry.gimbalrc_mode == SentryRobot::LEFT_MANNAL)
        {
            G_sentry.gimbalrc_mode = SentryRobot::RIGHT_MANNAL;
            mannal_change_flag = 0;
            G_sentry.right2left_flag = 1;
        }
        gimbal_pitch_des_step = -((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET) /
                                  RC_CH_VALUE_RANGE * GIMBAL_PITCH_ANGLE_MANNAL_CONTROL_SENSITIVITY);

        gimbal_yaw_des_step = -((float)(G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET) /
                                RC_CH_VALUE_RANGE * GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY);

        yaw_des_step = -((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET) /
                         RC_CH_VALUE_RANGE * GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY);

        if (!mannal_delay_cnt)
        {
            if (G_sentry.gimbalrc_mode == SentryRobot::LEFT_MANNAL)
            {
                if(G_sentry.right2left_flag)
                {
                    G_sentry.right2left_flag = 0;
                    G_sentry.left_pitch_des = G_sentry.left_pitch_fb;
                    G_sentry.left_yaw_des = G_sentry.left_yaw_fb;
                }
                else
                {
                    G_sentry.left_pitch_des += gimbal_pitch_des_step;
                    G_sentry.left_yaw_des += gimbal_yaw_des_step;
                    G_sentry.left_pitch_des = Clip(G_sentry.left_pitch_des, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
                    G_sentry.left_yaw_des = Clip(G_sentry.left_yaw_des, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
                    G_sentry.left_run_flag = 1;
                    G_sentry.right_shoot_insurance = 1;
                    G_sentry.right_run_flag = 0;
                }
            }
            else if (G_sentry.gimbalrc_mode == SentryRobot::RIGHT_MANNAL)
            {
                if(G_sentry.left2right_flag)
                {
                    G_sentry.left2right_flag = 0;
                    G_sentry.right_pitch_des = G_sentry.right_pitch_fb;
                    G_sentry.right_yaw_des = G_sentry.right_yaw_fb;
                }
                else
                {
                    G_sentry.right_pitch_des += gimbal_pitch_des_step;
                    G_sentry.right_yaw_des += gimbal_yaw_des_step;
                    G_sentry.right_pitch_des = Clip(G_sentry.right_pitch_des, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
                    G_sentry.right_yaw_des = Clip(G_sentry.right_yaw_des, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
                    G_sentry.right_run_flag = 1;
                    G_sentry.left_shoot_insurance = 1;
                    G_sentry.left_run_flag = 0;
                }
            }

            if(G_sentry.yaws_taitc2yaw_move_flag)
            {
                G_sentry.yaw_des = G_sentry.yaw_fb;
                G_sentry.yaws_taitc2yaw_move_flag = 0;
            }
            G_sentry.yaw_des += yaw_des_step;
        }
    }
    else if (G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO)
    {
        G_sentry.mode_toggle_flag = 1;
        if (G_djirc.channel.Ch0 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch1 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch2 > (RC_CH_VALUE_MAX - RC_CH_VALUE_DEAD) && G_djirc.channel.Ch3 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && change_flag_2)
        {
            auto_delay_cnt++;
            if (auto_delay_cnt > 2000)
            {
                auto_change_flag = 1;
                change_flag_2 = 0;
            }
        }
        else if (!(G_djirc.channel.Ch0 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch1 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD) && G_djirc.channel.Ch2 > (RC_CH_VALUE_MAX - RC_CH_VALUE_DEAD) && G_djirc.channel.Ch3 < (RC_CH_VALUE_MIN + RC_CH_VALUE_DEAD)))
        {
            auto_delay_cnt = 0;
            change_flag_2 = 1;
        }

        if (auto_change_flag && G_sentry.gimbalauto_mode == SentryRobot::LEFT_AUTO)
        {
            auto_change_flag = 0;
            G_sentry.gimbalauto_mode = SentryRobot::RIGHT_AUTO;
        }
        else if (auto_change_flag && G_sentry.gimbalauto_mode == SentryRobot::RIGHT_AUTO)
        {
            auto_change_flag = 0;
            G_sentry.gimbalauto_mode = SentryRobot::ALL_AUTO;
        }
        else if (auto_change_flag && G_sentry.gimbalauto_mode == SentryRobot::ALL_AUTO)
        {
            auto_change_flag = 0;
            G_sentry.gimbalauto_mode = SentryRobot::LEFT_AUTO;
        }

        if (G_control_mode != AUTO_G_RC_S)
            G_sentry.gimbalauto_mode = SentryRobot::ALL_AUTO;

        if (G_sentry.gimbalauto_mode == SentryRobot::LEFT_AUTO)
        {
            G_sentry.left_run_flag = 2;
            G_sentry.right_run_flag = 0;
            G_sentry.right_shoot_insurance = 1;
        }
        else if (G_sentry.gimbalauto_mode == SentryRobot::RIGHT_AUTO)
        {
            G_sentry.right_run_flag = 2;
            G_sentry.left_run_flag = 0;
            G_sentry.left_shoot_insurance = 1;
        }
        else if (G_sentry.gimbalauto_mode == SentryRobot::ALL_AUTO)
        {
            G_sentry.left_run_flag = 2;
            G_sentry.right_run_flag = 2;
        }
    }
    else
    {
        G_sentry.left_pitch_des = G_sentry.left_pitch_fb;
        G_sentry.right_pitch_des = G_sentry.right_pitch_fb;
        G_sentry.yaw_des = G_sentry.yaw_fb;
        mannal_delay_cnt = 0;
        auto_delay_cnt = 0;
        mannal_change_flag = 0;
        auto_change_flag = 0;
        G_sentry.left_run_flag = 0;
        G_sentry.right_run_flag = 0;
        G_sentry.gimbalrc_mode = SentryRobot::LEFT_MANNAL;
        G_sentry.gimbalauto_mode = SentryRobot::LEFT_AUTO;
        G_sentry.mode_toggle_flag = 1;
    }
}

/**
 *@brief update the robot gimbal targets
 *
 *@param
 */
void ShootTargetsUpdate(void)
{
    static uint32_t left_bullut_cnt_target_pre = 0;
    static uint32_t right_bullut_cnt_target_pre = 0;

    left_bullut_cnt_target_pre = G_sentry.m_left_shoot_bullet_cnt_target;
    right_bullut_cnt_target_pre = G_sentry.m_right_shoot_bullet_cnt_target;

    // Set the shoot insurance (control the gimbal friction wheels)
    if (G_sentry.shoot_mode == SentryRobot::SHOOT_AUTO)
    {
        G_sentry.left_shoot_insurance = false;
        G_sentry.right_shoot_insurance = false;
    }
    else if (G_sentry.shoot_mode == SentryRobot::SHOOT_MANNAL)
    {
        if ((G_sentry.gimbalrc_mode == SentryRobot::LEFT_MANNAL && G_sentry.gimbal_mode == SentryRobot::GIMBAL_MANNAL) || (G_sentry.gimbalauto_mode == SentryRobot::LEFT_AUTO && G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO) || (G_sentry.gimbalauto_mode == SentryRobot::ALL_AUTO && G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO))
        {
            if (G_djirc.channel.Ch4 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
            {
                G_sentry.left_shoot_insurance = true;
            }
            else if (G_djirc.channel.Ch4 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
            {
                G_sentry.left_shoot_insurance = false;
            }
        }
        if ((G_sentry.gimbalrc_mode == SentryRobot::RIGHT_MANNAL && G_sentry.gimbal_mode == SentryRobot::GIMBAL_MANNAL) || (G_sentry.gimbalauto_mode == SentryRobot::RIGHT_AUTO && G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO) || (G_sentry.gimbalauto_mode == SentryRobot::ALL_AUTO && G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO))
        {
            if (G_djirc.channel.Ch4 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
            {
                G_sentry.right_shoot_insurance = true;
            }
            else if (G_djirc.channel.Ch4 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
            {
                G_sentry.right_shoot_insurance = false;
            }
        }
    }
    else
    {
        G_sentry.left_shoot_insurance = true;
        G_sentry.right_shoot_insurance = true;
    }

    // Update the robot shoot bullet count target
    if (G_sentry.shoot_mode == SentryRobot::SHOOT_MANNAL)
    {
        if (G_sentry.left_shoot_insurance == false && !G_sentry.left_stop_shoot_flag)
        {
            if (G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
            {
                G_sentry.m_left_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_1;
                G_sentry.IncreaseLeftShootBulletTarget(G_system_monitor.SysTickTime);
            }
            else if (G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
            {
                G_sentry.m_left_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_4;
                G_sentry.IncreaseLeftShootBulletTarget(G_system_monitor.SysTickTime);
            }
        }

        if (G_sentry.right_shoot_insurance == false && !G_sentry.right_stop_shoot_flag)
        {
            if (G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
            {
                G_sentry.m_right_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_1;
                G_sentry.IncreaseRightShootBulletTarget(G_system_monitor.SysTickTime);
            }
            else if (G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
            {
                G_sentry.m_right_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_4;
                G_sentry.IncreaseRightShootBulletTarget(G_system_monitor.SysTickTime);
            }
        }
    }
    else if (G_sentry.shoot_mode == SentryRobot::SHOOT_AUTO &&
             G_sentry.left_shoot_insurance == false &&
             !G_sentry.left_stop_shoot_flag &&
             G_sentry.m_scan_flag && G_left_gimbal.m_data_receive_frame.m_cdata[0] &&
             !((G_left_gimbal.m_data_receive_frame.m_cdata[1] == 1 && G_sentry.m_enemy_1_res_flag) ||
               (G_left_gimbal.m_data_receive_frame.m_cdata[1] == 2 && (G_sentry.m_enemy_2_res_flag || (G_navigation.m_data_receive_frame.m_data_c[1] == 0))) ||
               (G_left_gimbal.m_data_receive_frame.m_cdata[1] == 3 && G_sentry.m_enemy_3_res_flag) ||
               (G_left_gimbal.m_data_receive_frame.m_cdata[1] == 4 && G_sentry.m_enemy_4_res_flag) ||
               (G_left_gimbal.m_data_receive_frame.m_cdata[1] == 5 && G_sentry.m_enemy_5_res_flag) ||
               (G_left_gimbal.m_data_receive_frame.m_cdata[1] == 7 && G_sentry.m_enemy_7_res_flag)))
    {
        G_sentry.m_left_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_4;
        G_sentry.IncreaseLeftShootBulletTarget(G_system_monitor.SysTickTime);
    }
    else if (G_sentry.shoot_mode == SentryRobot::SHOOT_AUTO &&
             G_sentry.right_shoot_insurance == false &&
             !G_sentry.right_stop_shoot_flag &&
             G_sentry.m_scan_flag && G_right_gimbal.m_data_receive_frame.m_cdata[0] &&
             !((G_right_gimbal.m_data_receive_frame.m_cdata[1] == 1 && G_sentry.m_enemy_1_res_flag) ||
               (G_right_gimbal.m_data_receive_frame.m_cdata[1] == 2 && (G_sentry.m_enemy_2_res_flag || (G_navigation.m_data_receive_frame.m_data_c[1] == 0))) ||
               (G_right_gimbal.m_data_receive_frame.m_cdata[1] == 3 && G_sentry.m_enemy_3_res_flag) ||
               (G_right_gimbal.m_data_receive_frame.m_cdata[1] == 4 && G_sentry.m_enemy_4_res_flag) ||
               (G_right_gimbal.m_data_receive_frame.m_cdata[1] == 5 && G_sentry.m_enemy_5_res_flag) ||
               (G_right_gimbal.m_data_receive_frame.m_cdata[1] == 7 && G_sentry.m_enemy_7_res_flag)))
    {
        G_sentry.m_right_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_4;
        G_sentry.IncreaseRightShootBulletTarget(G_system_monitor.SysTickTime);
    }
    else
    {
        G_sentry.m_left_shoot_bullet_cnt_target = 0;
        left_bullut_cnt_target_pre = 0;
        G_sentry.left_shoot_driver_angle_target =
            G_sentry.shoot_motor[SentryRobot::LEFT_SHOOT_DRIVE_MOTOR]->m_angle_current;

        G_sentry.m_right_shoot_bullet_cnt_target = 0;
        right_bullut_cnt_target_pre = 0;
        G_sentry.right_shoot_driver_angle_target =
            G_sentry.shoot_motor[SentryRobot::RIGHT_SHOOT_DRIVE_MOTOR]->m_angle_current;
    }

    // Update the shoot drive motor position target
    G_sentry.left_shoot_driver_angle_target += SHOOT_DRIVER_SUPPLY_ANGLE_STEP *
                                               (G_sentry.m_left_shoot_bullet_cnt_target - left_bullut_cnt_target_pre);
    G_sentry.right_shoot_driver_angle_target += SHOOT_DRIVER_SUPPLY_ANGLE_STEP *
                                                (G_sentry.m_right_shoot_bullet_cnt_target - right_bullut_cnt_target_pre);

    G_sentry.SetShootDriverTarget(G_sentry.left_shoot_driver_angle_target, G_sentry.right_shoot_driver_angle_target);
}

/**
 *@brief execute the robot control
 *
 *@param
 */
void RobotControlExecute(void)
{
    // Execute the robot shoot control algorithm
    G_sentry.ExecuteShootAlgorithm();

    // Send the control commands to the actuators
    G_sentry.SendControlCommand();
}
