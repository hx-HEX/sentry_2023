#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "rtos.h"
#include "system_monitor.h"
#include "user_global.h"
#include "uart_protocol.h"
#include "can_protocol.h"
#include "common_math.h"

#define Led_TASK_CYCLE 500
#define DataVisualize_TASK_CYCLE 1
#define RobotControl_TASK_CYCLE 1
#define CoreDataSend_TASK_CYCLE 1
#define AimAssistDataSend_TASK_CYCLE 1

void LaunchAllTasks(void);