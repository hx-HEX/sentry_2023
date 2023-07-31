#include "user_task.h"

DECLARE_RM_OS_TASK()
DECLARE_RM_OS_FLAG()

void LaunchAllTasks(void){
    taskENTER_CRITICAL();

    CREATE_OS_TASK(SystemMonitorTask);
		CREATE_OS_TASK(GimbalDataSendTask);
		CREATE_OS_TASK(RefereeDataSendTask);
		CREATE_OS_TASK(DataVisualTask);
		CREATE_OS_TASK(RobotControlTask);
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
		// Communication lost connection detection
		CommunicationMonitor();
		// Motr lost connection detection
		MotorMonitor();

		G_system_monitor.SystemMonitorTask_cnt++; // Statistic task execution times
		G_system_monitor.SystemMonitorTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}

void RefereeDataSendTask(void *pvParameters){
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(RefreeDataSend_Task_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		SendRefereeData();

		G_system_monitor.RefereeDataSendTask_cnt++;
		G_system_monitor.RefereeDataSendTask_ExecuteTime = TIME() - TaskStartTime;
		
		vTaskDelay(RouteTimes);
	}
}

void GimbalDataSendTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(GimbalDataSend_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		SendGimbalData();
		
		G_system_monitor.GimbalDataSendTask_cnt++; // Statistic task execution times
		G_system_monitor.GimbalDataSendTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}

void DataVisualTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(DataVisualize_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		// the ID of mdata should be less than 20
		VisualizeData();
		
		G_vofa.m_data_send_frame.m_data[20] = G_system_monitor.SysTickTime;

		G_vofa.SendData();

		G_system_monitor.DataVisualTask_cnt++; // Statistic task execution times
		G_system_monitor.DataVisualTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
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