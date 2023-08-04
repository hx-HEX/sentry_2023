#pragma once
#include "system_monitor.h"
#include "led.h"
#include "usart_interface.h"
#include "gimbal.h"
#include "vofa.h"
#include "uart_protocol.h"
#include "common_math.h"
#include "sentry_robot.h"
#include "verify.h"
#include "referee.h"


enum ControlMode
{
    Chassis_mannal = 0,
    Chassis_auto,
    Chassis_safe,
    Gimbal_move,
    Gimbal_calibration,
    Gimbal_safe
};

extern SystemMonitor G_system_monitor;
extern RefereeMonitor G_referee_monitor;
extern Led G_led;
extern Gimbal G_gimbal;
extern Vofa G_vofa;
extern Referee G_referee;
extern SentryRobot G_sentry;
extern ControlMode G_control_mode;

void ControlModeUpdate(void);
void RobotStatesUpdate(void);
void RobotTargetsUpdate(void);
void RobotControlExecute(void);
void YawGimbalCalibrate(void);
void ChassisTargetsUpdate(void);
void GimbalTargrtUpdata(void);
void OmnidirectionalChassisTargetUpdate(void);

void SendGimbalData(void);
void SendRefereeData(void);

void VisualizeData(void);

void CommunicationMonitor(void);
void MotorMonitor(void);