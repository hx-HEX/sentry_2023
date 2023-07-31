#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "rtos.h"
#include "system_monitor.h"
#include "user_global.h"
#include "uart_protocol.h"
#include "can_protocol.h"

#define SystemMonitor_TASK_CYCLE 500
#define DataVisualize_TASK_CYCLE 1
#define RobotControl_TASK_CYCLE 1
#define GimbalDataSend_TASK_CYCLE 10
#define RefreeDataSend_Task_CYCLE 10

void LaunchAllTasks(void);