#pragma once
#include "system_monitor.h"
#include "led.h"
#include "usart_interface.h"
#include "left_gimbal.h"
#include "right_gimbal.h"
#include "yaw_gimbal.h"
#include "chassis_core.h"
#include "djirc.h"
#include "vofa.h"
#include "navigation.h"
#include "referee.h"
#include "uart_protocol.h"
#include "common_math.h"
#include "sentry_robot.h"
#include "photogate.h"

enum ControlMode
{
    SAFE = 0,
    CALIBRATE,
    RC_GS,
    AUTO_G_RC_S,
    AUTO_GS_RC_C,
    AUTO_CGS,
    RC_C,
    AUTO_G_RC_C
};

extern SystemMonitor G_system_monitor;
extern RefereeMonitor G_referee_monitor;
extern Led G_led;
extern Left_Gimbal G_left_gimbal;
extern Right_Gimbal G_right_gimbal;
extern Yaw_Gimbal G_yaw_gimbal;
extern Chassis_Core G_chassis_core;
extern DJIRC G_djirc;
extern Vofa G_vofa;
extern Navigation G_navigation;
extern Referee G_referee;
extern SentryRobot G_sentry;
extern ControlMode G_control_mode;

void ControlModeUpdate(void);
void RobotStatesUpdate(void);
void RobotTargetsUpdate(void);
void RobotControlExecute(void);
void ChassisTargetsUpdate(void);
void GimbalTargetsUpdate(void);
void ShootTargetsUpdate(void);
void CompetitionStateUpdate(void);

void SendNavigationData(void);
void SendYawGimbalData(void);
void SendLeftGimbalData(void);
void SendRightGimbalData(void);
void SendChassisCoreData(void);

void RemoteControlMonitor(void);
void CommunicationMonitor(void);
void MotorMonitor(void);