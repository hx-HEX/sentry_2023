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

enum ControlMode
{
    Gimbal_safe = 0,
    Gimbal_move,
    Gimbal_auto,
    Shoot_safe,
    Shoot_move
};

extern SystemMonitor G_system_monitor;
extern Led G_led;
extern Vofa G_vofa;
extern Gimbal G_gimbal;
extern SentryRobot G_sentry;
extern ControlMode G_control_mode;

void MotorMonitor(void);

void VisualizeGimbalData(void);

void ControlModeUpdate(void);
void RobotStatesUpdate(void);
void RobotTargetsUpdate(void);
void RobotControlExecute(void);

void GimbalTargetsUpdate(void);
void ShootTargetsUpdate(void);

void SendCoreData(void);
void SendAimAssistData(void);




