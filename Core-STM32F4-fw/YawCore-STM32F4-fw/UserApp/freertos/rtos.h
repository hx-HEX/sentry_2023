#ifndef __RM_OS_H_
#define __RM_OS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/****************************************************************************************************/			 
/*********************************************TASK***************************************************/		
/****************************************************************************************************/	

/*Defines the stack length of the task*/
#define SystemMonitorTask_SIZE          100
#define RobotControlTask_SIZE           200
#define YawGimbalDataSendTask_SIZE      200
#define RightGimbalDataSendTask_SIZE     	200
#define LeftGimbalDataSendTask_SIZE     200
#define ChassisCoreDataSendTask_SIZE    200
#define NavigationDataSendTask_SIZE		200
#define UsartDecodeTask_SIZE			200

/*Define the priority of the task*/
#define SystemMonitorTask_PRIO          1
#define RobotControlTask_PRIO           2
#define YawGimbalDataSendTask_PRIO		3
#define RightGimbalDataSendTask_PRIO       3
#define LeftGimbalDataSendTask_PRIO     3
#define ChassisCoreDataSendTask_PRIO    3
#define NavigationDataSendTask_PRIO     3
#define UsartDecodeTask_PRIO			4

/****************************************************************************************************
Macro function name：DECLARE_OS_TASK(NAME)
The function of function：Declare the OS task stack and the OS tasks
Parameters：Function name
****************************************************************************************************/
#define DECLARE_OS_TASK(NAME)\
TaskHandle_t NAME##_Handler;\
void NAME(void *pvParameters)

/*Call the function 'DECLARE_OS_TASK' to declare the created task*/
#define DECLARE_RM_OS_TASK();\
DECLARE_OS_TASK(SystemMonitorTask);\
DECLARE_OS_TASK(RobotControlTask);\
DECLARE_OS_TASK(YawGimbalDataSendTask);\
DECLARE_OS_TASK(RightGimbalDataSendTask);\
DECLARE_OS_TASK(LeftGimbalDataSendTask);\
DECLARE_OS_TASK(ChassisCoreDataSendTask);\
DECLARE_OS_TASK(NavigationDataSendTask);\
DECLARE_OS_TASK(UsartDecodeTask);

/****************************************************************************************************
Macro function name：CREATE_OS_TASK(NAME)
The function of function：Create an operating system task based on the task name, 
                        assign the stack based on the name, and prioritize the task
Parameters：Function name
****************************************************************************************************/
#define CREATE_OS_TASK(NAME)\
xTaskCreate((TaskFunction_t )NAME,\
			(const char *   )"NAME",\
			(uint16_t       )NAME##_SIZE,\
			(void *         )NULL,\
			(UBaseType_t    )NAME##_PRIO,\
			(TaskHandle_t * )&NAME##_Handler) 



/****************************************************************************************************/			 
/*********************************************EVENT_GROUP********************************************/		
/****************************************************************************************************/

/****************************************************************************************************
Macro function name：DECLARE_OS_FLAG(NAME)
The function of function：Define event groups
Parameters：the name of event group 
****************************************************************************************************/
#define DECLARE_OS_FLAG(NAME)\
EventGroupHandle_t NAME##Handler

/*Call the function 'DECLARE_OS_FLAG' to declare the created event groups*/			 
#define DECLARE_RM_OS_FLAG() \
DECLARE_OS_FLAG(USART);\

/****************************************************************************************************
Macro function name：GetEventGroupFlag(NAME)
The function of function：Create an event group that waits for four event flags or operations
Parameter specification：1.Handle to the event group
			             2.Specifies the event bits to wait for
                         3.Clear event bits before exiting the function when set to pdTRUE 
                         4.Any of the set event bits will be returned by the set 1 function when set 
                           to pdFALSE
                         5.Maximum blocking time
Parameters：the name of event group 
****************************************************************************************************/
#define GetEventGroupFlag(NAME)\
xEventGroupWaitBits((EventGroupHandle_t	)NAME##Handler,\
					(EventBits_t		)0xff,\
					(BaseType_t			)pdTRUE,\
					(BaseType_t			)pdFALSE,\
					(TickType_t			)portMAX_DELAY);
					
#endif

