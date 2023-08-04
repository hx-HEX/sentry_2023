#include "user_task.h"
#include "verify.h"

DECLARE_RM_OS_TASK()
DECLARE_RM_OS_FLAG()

void LaunchAllTasks(void){
    taskENTER_CRITICAL();

		CREATE_OS_TASK(LedTask);
		CREATE_OS_TASK(DataVisualTask);
		CREATE_OS_TASK(RobotControlTask);
		CREATE_OS_TASK(UsartDecodeTask);
        CREATE_OS_TASK(CoreDataSendTask);
		CREATE_OS_TASK(AimAssistDataSendTask);

		USARTHandler = xEventGroupCreate();//创建事件组，返回句柄

    taskEXIT_CRITICAL();
    vTaskStartScheduler();
}



/**
 * @brief communication task
 *
 * @param NULL
 */
void CoreDataSendTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(CoreDataSend_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		SendCoreData();
		
		G_system_monitor.CoreDataSendTask_cnt++; // Statistic task execution times
		G_system_monitor.CoreDataSendTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}


/**
 * @brief communication task
 *
 * @param NULL
 */
void AimAssistDataSendTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(AimAssistDataSend_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		SendAimAssistData();
		
		G_system_monitor.AimAssistDataSendTask_cnt++; // Statistic task execution times
		G_system_monitor.AimAssistDataSendTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}


/**
 * @brief Data visual task
 *
 * @param NULL
 */
void DataVisualTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(DataVisualize_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		// the ID of mdata should be less than 15
		VisualData();
		
		G_vofa.m_data_send_frame.m_data[15] = G_system_monitor.SysTickTime;

		G_vofa.SendData();

		G_system_monitor.DataVisualizeTask_cnt++; // Statistic task execution times
		G_system_monitor.DataVisualizeTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



/**
 * @brief LED task
 *
 * @param NULL
 */
void LedTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(Led_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		MotorMonitor();
		
		G_system_monitor.LedTask_cnt++; // Statistic task execution times
		G_system_monitor.LedTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



/**
 * @brief Robot control task
 *
 * @param NULL
 */
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