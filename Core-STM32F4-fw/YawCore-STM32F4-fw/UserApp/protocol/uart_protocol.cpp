#include "uart_protocol.h"
#include "user_global.h"

static void MatchRSData(u16 cmdID, u8 *pData, u16 Size);

void UART_Decode(EventBits_t EventValue){
    static EventBits_t usart_djirc_bit;
    static EventBits_t usart_chassis_core_bit;
    static EventBits_t usart_yaw_gimbal_bit;
    static EventBits_t usart_right_gimbal_bit;
    static EventBits_t usart_navigation_bit;
	static EventBits_t usart_left_gimbal_bit;

    usart_djirc_bit = EventValue & 0x01;
    usart_chassis_core_bit = EventValue & 0x02;
    usart_yaw_gimbal_bit = EventValue & 0x04;
    usart_right_gimbal_bit = EventValue & 0x08;
    usart_navigation_bit = EventValue & 0x10;
    usart_left_gimbal_bit = EventValue & 0x20;
	
    if(usart_djirc_bit){
		UART_DJIRCDataDecode();
        G_system_monitor.Dijrc_Receive_cnt++;
    }

    if(usart_chassis_core_bit){
		UART_ChassisCoreDataDecode(G_chassis_core.usart_chassis_core->USART_RT.pMailbox,G_chassis_core.usart_chassis_core->USART_RT.rxSize);
		G_system_monitor.Chassis_Core_Receive_cnt++;
    }

    if(usart_yaw_gimbal_bit){
		UART_YawGimbalDataDecode(G_yaw_gimbal.usart_yaw_gimbal->USART_RT.pMailbox);
		G_system_monitor.Yaw_Gimbal_Receive_cnt++;
    }

    if(usart_right_gimbal_bit){
		UART_RightGimbalDataDecode(G_right_gimbal.usart_right_gimbal->USART_RT.pMailbox);
        G_system_monitor.Right_Gimbal_Receive_cnt++;
    }

    if(usart_navigation_bit){
		UART_NavigationDataDecode(G_navigation.usart_navigation->USART_RT.pMailbox);
        G_system_monitor.Navigation_Receive_cnt++;
    }

    if(usart_left_gimbal_bit){
		UART_LeftGimbalDataDecode(G_left_gimbal.usart_left_gimbal->USART_RT.pMailbox);
        G_system_monitor.Left_Gimbal_Receive_cnt++;
    }
}

void UART_DJIRCDataDecode(void){
        // right rocker bar
	G_djirc.channel.Ch0 = (G_djirc.m_data_receive_frame[0] | (G_djirc.m_data_receive_frame[1] << 8)) & 0x07FF;           					// Channe0——水平通道
	G_djirc.channel.Ch1 = ((G_djirc.m_data_receive_frame[1]) >> 3 | (G_djirc.m_data_receive_frame[2] << 5)) & 0x07FF;   					// Channe1——垂直通道
	
	
	// left rocker bar
	G_djirc.channel.Ch2 = ((G_djirc.m_data_receive_frame[3] >> 6 | (G_djirc.m_data_receive_frame[3] << 2) | (G_djirc.m_data_receive_frame[4] << 10))) & 0x07FF;	// Channe2——水平通道
	G_djirc.channel.Ch3 = ((G_djirc.m_data_receive_frame[4] >> 1) | (G_djirc.m_data_receive_frame[5] << 7)) & 0x07FF;                   	// Channe3——垂直通道
	
	
	// pulley
	G_djirc.channel.Ch4 = (G_djirc.m_data_receive_frame[16] | (G_djirc.m_data_receive_frame[17] << 8)) & 0x07ff;                        	// Channe4——垂直通道

	
	// left switch
	G_djirc.channel.SW_L = ((G_djirc.m_data_receive_frame[5] >> 4) & 0x000C) >> 2;
	
	
	// right switch
	G_djirc.channel.SW_R = ((G_djirc.m_data_receive_frame[5] >> 4) & 0x0003);
}


void UART_ChassisCoreDataDecode(unsigned char* buffer,uint16_t size)
{
	uint8_t pData[250] = {0};
	uint8_t referee_rxsize = 0;

	for(uint16_t i; i < size; i++)
	{
		if(buffer[i] == 0x55 && buffer[i+1] == 0x00)
			memcpy(&(G_chassis_core.m_data_receive_frame),(u8*)&buffer[i],
			sizeof(G_chassis_core.m_data_receive_frame));

		if(buffer[i] == 0xAB && buffer[i+1] == 0xA1)
		{
			referee_rxsize = buffer[i+2];
			memcpy(pData,(u8*)&buffer[i+CHASSIS_CORE_RECEIVE_HEAD_SIZE],referee_rxsize);
			UART_RSDataDecode(pData,referee_rxsize);
			if(G_navigation.navigation_send_buff_cnt + referee_rxsize + NAVIGATION_SEND_HEAD_SIZE < NAVIGATION_SEND_BUFF_SIZE)
			{
				G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = 0xAB;
				G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = 0xA1;
				G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = referee_rxsize;
				memcpy(&G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt],pData,referee_rxsize);
				G_navigation.navigation_send_buff_cnt += referee_rxsize;
			}
			else
			{
					G_navigation.navigation_send_buff_cnt = 0;
			}
		}
	}
}



void UART_RSDataDecode(u8* buffer, u16 Size){
    RS_RX_Status RXStatus = RS_RX_FREE;
	u8 BufNum = 0;
	u8 RsData;
	u16 RsDataLength;
	u8 CorrectFlag = 0;															// 正确接收标志位
	u8 pData[250] = {0};
	u16 RsCmdID;

	for (int i = 0; i < Size; i++)												// 遍历每个接收的字节
	{
		RsData = buffer[i];														// 第i个接收的字节
		switch (RXStatus)														// 判断该字节在数据包中所处的位置
		{
			case RS_RX_FREE:
				if (RsData == 0xA5)												// 数据包起始帧SOF固定值0xA5
				{
					BufNum = 0;													// 第0位数据
					RXStatus = RS_RX_Length;									// 起始帧校验成功准备解码下一个数据段：有效数据长度
					pData[BufNum++] = 0xA5;										// 把该位数据存入用户定义的接收数组中			
				}
				else															// 数据失真则直接放弃接下来的数据解码防止解码错误，等待下一次接收
				{
					RXStatus = RS_RX_FREE;
				}
				break;
			
			case RS_RX_Length:
				pData[BufNum++] = RsData;										// 接收表示数据长度的两个字节
				if(BufNum == 3)
				{
					RsDataLength = pData[1] | (pData[2] << 8);					// 得到有效数据长度（data_length占两个字节）
					RXStatus = RS_RX_Num;										// 准备接收包序号
				}
				break;
			
			case RS_RX_Num:
				pData[BufNum++] = RsData;										// 接收包序号
				RXStatus = RS_RX_CRC8;											// 准备接收CRC8校验字
				break;
			
			case RS_RX_CRC8:
				pData[BufNum++] = RsData;										// 接收CRC8校验字
				if (Verify_CRC8_Check_Sum(pData, BufNum))
				{	
					RXStatus = RS_RX_CmdID;										// CRC8校验成功，准备接收数据ID
				}
				else															
				{
					RXStatus = RS_RX_FREE;										// CRC8校验失败，放弃此次接收的数据
				}
				break;
			
			case RS_RX_CmdID:
				pData[BufNum++] = RsData;										// 接收数据ID
				if(BufNum == 7)
				{
					RsCmdID = pData[5] | (pData[6] << 8);						// 得到数据ID号
					RXStatus = RS_RX_Data;										// 准备接收数据	
				}
				break;
				
			case RS_RX_Data:
				pData[BufNum++] = RsData;										// 接收数据
				if(BufNum == 7 + RsDataLength)
				{
					RXStatus = RS_RX_CRC16;										// 准备接收两个CRC16校验字节	
				}
				break;

			case RS_RX_CRC16:
				pData[BufNum++] = RsData;										// 接收两个CRC16校验字节
				if(BufNum == 9 + RsDataLength)
				{
					if (Verify_CRC16_Check_Sum(pData, BufNum))
					{	
						MatchRSData(RsCmdID, pData, RsDataLength);				// 裁判系统数据复制到对应结构体	（注！！:一次空闲中断会接收到多组数据（一次中断会多次解码））
						RXStatus = RS_RX_FREE;
						G_system_monitor.UART2_rx_cnt++;					// 对有效接收次数进行计数（计数周期为1s）
					}
				}
				break;

			default:
				break;
		}
	}
}

/**
  * @brief  裁判系统数据复制到对应结构体，协议为20210203 v1.0版本
  * @retval None
  * @param	cmdID:	裁判系统数据指令ID
  * 		pData:	RoboMaster裁判系统信息存储数组
  * 		Size:	数据长度
  */
void MatchRSData(u16 cmdID, u8 *pData, u16 Size)
{
	switch (cmdID)
	{
		case GameStatusID:
			G_referee_monitor.GameStatus_cnt++;
			memcpy(&G_referee.GameStatus, &pData[7], Size);
			break;
		case GameRobotHPID:
			G_referee_monitor.GameRobotHP_cnt++;
			memcpy(&G_referee.GameRobotHP, &pData[7], Size);
			break;
		case EventDataID:
			G_referee_monitor.EventData_cnt++;
			memcpy(&G_referee.EventData, &pData[7], Size);
			break;
		case RefereeWaringID:
			G_referee_monitor.RefereeWarning_cnt++;
			memcpy(&G_referee.RefereeWarning, &pData[7], Size);
			break;
		case GameRobotStatusID:
			G_referee_monitor.GameRobotStatus_cnt++;
			memcpy(&G_referee.GameRobotStatus, &pData[7], Size);
			break;
		case PowerHeatDataID:
			G_referee_monitor.PowerHeatData_cnt++;
			memcpy(&G_referee.PowerHeatData, &pData[7], Size);
			break;
		case GameRobotPosID:
			G_referee_monitor.GameRobotPos_cnt++;
			memcpy(&G_referee.GameRobotPos, &pData[7], Size);
			break;
		case BuffID:
			G_referee_monitor.Buff_cnt++;
			memcpy(&G_referee.Buff, &pData[7], Size);
			break;
		case RobotHurtID:
			G_referee_monitor.RobotHurt_cnt++;
			memcpy(&G_referee.RobotHurt, &pData[7], Size);
			break;
		case ShootDataID:
			G_referee_monitor.ShootData_cnt++;
			if(pData[7] == 1)
				memcpy(&G_referee.ShootData_1, &pData[7], Size);
			else
				memcpy(&G_referee.ShootData_2, &pData[7], Size);
			break;
		case BulletRemainingID:
			G_referee_monitor.BulletRemaining_cnt++;
			memcpy(&G_referee.BulletRemaining, &pData[7], Size);
			break;
		case RFIDStatusID:
			G_referee_monitor.RFIDStatus_cnt++;
			memcpy(&G_referee.RFIDStatus, &pData[7], Size);
			break;
		case RobotInteractiveDataID:
			G_referee_monitor.RobotInteractiveData_cnt++;
			memcpy(&G_referee.StudentInteractiveHeaderData, &pData[7], 6);
			memcpy(&G_referee.RobotInteractiveData, &pData[13], Size - 6);
			break;
		case RobotCommandID:
			G_referee_monitor.RobotCommand_cnt++;
			memcpy(&G_referee.RobotCommand, &pData[7], Size);
			G_navigation.m_data_send_frame.m_id = 0x01;
			G_navigation.m_data_send_frame.m_data[0] = G_referee.RobotCommand.target_position_x - ROBOMASTER_SITE_BIAS_X;
			G_navigation.m_data_send_frame.m_data[1] = G_referee.RobotCommand.target_position_y ;
			break;
		default:
			break;
	}
}

void UART_YawGimbalDataDecode(unsigned char* buffer){
    if(	buffer[0]==0x55 && buffer[1]==0x00 && Verify_CRC16_Check_Sum(buffer,YAW_GIMBAL_DATA_RECEIVE_SIZE)) {
		memcpy((&G_yaw_gimbal.m_data_receive_frame), (u8*)buffer, 
		sizeof(G_yaw_gimbal.m_data_receive_frame));	
    }
}

void UART_RightGimbalDataDecode(unsigned char* buffer){
	if(	buffer[0]==0x55 && buffer[1]==0x00 && Verify_CRC16_Check_Sum(buffer,RIGHT_GIMBAL_DATA_RECEIVE_SIZE)) {
		memcpy((&G_right_gimbal.m_data_receive_frame), (u8*)buffer, 
		sizeof(G_right_gimbal.m_data_receive_frame));	
    }
}

void UART_NavigationDataDecode(unsigned char* buffer){
    if(	buffer[0]==0x55 && buffer[1]==0x00) {
		memcpy(&(G_navigation.m_data_receive_frame), (u8*)buffer, 
		sizeof(G_navigation.m_data_receive_frame));	
    }	
}

void UART_LeftGimbalDataDecode(unsigned char* buffer){
    if(	buffer[0]==0x55 && buffer[1]==0x00 && Verify_CRC16_Check_Sum(buffer,LEFT_GIMBAL_DATA_RECEIVE_SIZE)) {
		memcpy(&(G_left_gimbal.m_data_receive_frame), (u8*)buffer, 
		sizeof(G_left_gimbal.m_data_receive_frame));	
    }
}



