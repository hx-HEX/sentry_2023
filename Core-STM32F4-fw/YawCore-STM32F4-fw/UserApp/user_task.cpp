#include "user_task.h"

DECLARE_RM_OS_TASK()
DECLARE_RM_OS_FLAG()

void LaunchAllTasks(void){
    taskENTER_CRITICAL();

    CREATE_OS_TASK(SystemMonitorTask);
		CREATE_OS_TASK(RobotControlTask);
		CREATE_OS_TASK(YawGimbalDataSendTask);
		CREATE_OS_TASK(RightGimbalDataSendTask);
		CREATE_OS_TASK(LeftGimbalDataSendTask);
		CREATE_OS_TASK(ChassisCoreDataSendTask);
		CREATE_OS_TASK(NavigationDataSendTask);
		CREATE_OS_TASK(UsartDecodeTask);

		USARTHandler = xEventGroupCreate();//创建事件组，返回句柄

    taskEXIT_CRITICAL();
    vTaskStartScheduler();
}

void SystemMonitorTask(void *pvParameters){
    static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(SystemMonitor_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		// tasks executes normally
		G_led.TogglePink();
		// Remote control lost connection protection
		RemoteControlMonitor();
		// Communication lost connection detection
		CommunicationMonitor();
		// Motr lost connection detection
		MotorMonitor();

		G_system_monitor.SystemMonitorTask_cnt++; // Statistic task execution times
		G_system_monitor.SystemMonitorTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}

void ChassisCoreDataSendTask(void *pvParameters){
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(ChassisCoreDataSend_Task_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		SendChassisCoreData();

		G_system_monitor.ChassisCoreDataSendTask_cnt++;
		G_system_monitor.ChassisCoreDataSendTask_ExecuteTime = TIME() - TaskStartTime;
		
		vTaskDelay(RouteTimes);
	}
}

void NavigationDataSendTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(NavigationDataSend_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		SendNavigationData();
		
		G_system_monitor.NavigationDataSendTask_cnt++; // Statistic task execution times
		G_system_monitor.NavigationDataSendTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}

void LeftGimbalDataSendTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(LeftGimbalDataSend_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		SendLeftGimbalData();
		
		G_system_monitor.LeftGimbalDataSendTask_cnt++; // Statistic task execution times
		G_system_monitor.LeftGimbalDataSendTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}

void RightGimbalDataSendTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(RightGimbalDataSend_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		SendRightGimbalData();
		
		G_system_monitor.RightYawDataSendTask_cnt++; // Statistic task execution times
		G_system_monitor.RightYawDataSendTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}

void YawGimbalDataSendTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(YawGimbalDataSend_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		SendYawGimbalData();

		G_system_monitor.YawGimbalDataSendTask_cnt++; // Statistic task execution times
		G_system_monitor.YawGimbalDataSendTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}

void RobotControlTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(RobotControl_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		// Update the control mode
		ControlModeUpdate();
		// Update the robot state
		RobotStatesUpdate();
		// Update the robot targets
		RobotTargetsUpdate();	
		// Execute the robot control
		RobotControlExecute();

		G_system_monitor.RobotControlTask_cnt++; // Statistic task execution times
		G_system_monitor.RobotControlTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}


void UsartDecodeTask(void *pvParameters) {
	static u32 TaskStartTime;
	EventBits_t EventValue=0;

	while(1)
	{
		EventValue = GetEventGroupFlag(USART);
		TaskStartTime = TIME();

		UART_Decode(EventValue);

		G_system_monitor.UsartDecodeTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
	}
}