#ifndef OS_TYPES_H
#define OS_TYPES_H

#include "stm32f4xx.h"

// 系统监视器时间源
#define TIME() TIM5->CNT

// 系统监视
#define DECLARE_MONITOR(name) 		unsigned int name##_cnt; unsigned int name##_fps
#define DECLARE_EXECUTE_TIME(name)	unsigned int name##_ExecuteTime

// 任务帧率统计结构体
typedef struct
{		
	// 任务执行频率监视
	DECLARE_MONITOR(SystemMonitorTask);
	DECLARE_MONITOR(DataVisualTask);
	DECLARE_MONITOR(RobotControlTask);
	DECLARE_MONITOR(GimbalDataSendTask);
	DECLARE_MONITOR(RefereeDataSendTask);
	DECLARE_MONITOR(Referee_Receive);
	DECLARE_MONITOR(Vofa_Receive);
	DECLARE_MONITOR(Gimbal_Receive);


	// 任务执行时间监视
	DECLARE_EXECUTE_TIME(SystemMonitorTask);
	DECLARE_EXECUTE_TIME(DataVisualTask);
	DECLARE_EXECUTE_TIME(RobotControlTask);
	DECLARE_EXECUTE_TIME(GimbalDataSendTask);
	DECLARE_EXECUTE_TIME(RefereeDataSendTask);
	DECLARE_EXECUTE_TIME(UsartDecodeTask);

	// 中断监视
	DECLARE_MONITOR(UART1_rx);
	DECLARE_MONITOR(UART2_rx);
	DECLARE_MONITOR(UART3_rx);
	DECLARE_MONITOR(UART4_rx);
    DECLARE_MONITOR(UART5_rx);
	DECLARE_MONITOR(UART6_rx);
	
	DECLARE_MONITOR(CAN1_rx);
	DECLARE_MONITOR(CAN2_rx);

	// 系统时间
	unsigned int SysTickTime;
} SystemMonitor;



/** 裁判系统通讯帧率统计结构体 */
typedef struct
{
	DECLARE_MONITOR(GameStatus);
	DECLARE_MONITOR(GameRobotHP);
	DECLARE_MONITOR(DartStatus);
	DECLARE_MONITOR(EventData);
	DECLARE_MONITOR(RefereeWarning);
	DECLARE_MONITOR(DartRemainingTime);
	DECLARE_MONITOR(GameRobotStatus);
	DECLARE_MONITOR(PowerHeatData);
	DECLARE_MONITOR(GameRobotPos);
	DECLARE_MONITOR(Buff);
	DECLARE_MONITOR(RobotHurt);
	DECLARE_MONITOR(ShootData);
	DECLARE_MONITOR(BulletRemaining);
	DECLARE_MONITOR(RFIDStatus);
	DECLARE_MONITOR(RobotInteractiveData);
	DECLARE_MONITOR(RobotCommand);
	
} RefereeMonitor;

#endif