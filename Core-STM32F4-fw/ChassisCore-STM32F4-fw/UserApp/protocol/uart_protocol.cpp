#include "uart_protocol.h"
#include "user_global.h"

static void MatchRSData(u16 cmdID, u8 *pData, u16 Size);

void UART_Decode(EventBits_t EventValue){
    static EventBits_t usart_vofa_bit;
    static EventBits_t usart_gimbal_bit;
    static EventBits_t usart_referee_bit;

    usart_referee_bit = EventValue & 0x02;
    usart_vofa_bit = EventValue & 0x04;
    usart_gimbal_bit = EventValue & 0x08;

    if(usart_referee_bit){
        UART_RSDataDecode(G_referee.usart_referee->USART_RT.pMailbox,G_referee.usart_referee->USART_RT.rxSize);
		if(G_gimbal.gimbal_send_buff_cnt + G_referee.usart_referee->USART_RT.rxSize + GIMBAL_SEND_HEAD_SIZE < GIMBAL_SEND_BUFF_SIZE){
			G_gimbal.gimbal_send_buff[G_gimbal.gimbal_send_buff_cnt++] = 0xAB;
			G_gimbal.gimbal_send_buff[G_gimbal.gimbal_send_buff_cnt++] = 0xA1;
			G_gimbal.gimbal_send_buff[G_gimbal.gimbal_send_buff_cnt++] = G_referee.usart_referee->USART_RT.rxSize;
			memcpy(&G_gimbal.gimbal_send_buff[G_gimbal.gimbal_send_buff_cnt],G_referee.usart_referee->USART_RT.pMailbox,G_referee.usart_referee->USART_RT.rxSize);
			G_gimbal.gimbal_send_buff_cnt += G_referee.usart_referee->USART_RT.rxSize;
			}
		else{
				G_gimbal.gimbal_send_buff_cnt = 0;
			}
        G_system_monitor.Referee_Receive_cnt++;
    }

    if(usart_vofa_bit){
		    G_system_monitor.Vofa_Receive_cnt++;
    }

    if(usart_gimbal_bit){
		    UART_GimbalDataDecode(G_gimbal.usart_gimbal->USART_RT.pMailbox);
        G_system_monitor.Gimbal_Receive_cnt++;
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
			memcpy(&G_referee.ShootData, &pData[7], Size);
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
			break;
		default:
			break;
	}
}



void UART_GimbalDataDecode(unsigned char* buffer){
    if(	buffer[0]==0x55 && buffer[1]==0x00 && Verify_CRC16_Check_Sum(buffer,GIMBAL_DATA_RECEIVE_SIZE)) {
		memcpy(&(G_gimbal.m_data_receive_frame), (u8*)buffer, 
		sizeof(G_gimbal.m_data_receive_frame));	
    }
}



