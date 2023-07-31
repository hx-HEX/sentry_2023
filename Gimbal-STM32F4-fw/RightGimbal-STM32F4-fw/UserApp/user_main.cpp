#include "user_main.h"
#include "user_task.h"
#include "communication.h"

void UserMain(void){
    //通讯类初始化
    InitCommunication();
    //sentry类初始化
    G_sentry.Init();
    //使能接收中断
    StartCommunication();
    //启动任务
    LaunchAllTasks();
}