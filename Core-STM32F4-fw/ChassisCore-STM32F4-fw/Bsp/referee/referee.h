#pragma once
#include "rm_rs_v2023_types.h"
#include "usart.h"
#include "verify.h"
#include "usart_interface.h"

typedef unsigned char  		UCHAR8;                  /** defined for unsigned 8-bits integer variable 	    无符号8位整型变量       */
typedef signed   char  		SCHAR8;                  /** defined for signed 8-bits integer variable		    有符号8位整型变量       */
typedef unsigned short 		USHORT16;                /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量      */
typedef signed   short 		SSHORT16;                /** defined for signed 16-bits integer variable 	    有符号16位整型变量      */
typedef unsigned int   		UINT32;                  /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量      */
typedef int   				SINT32;                  /** defined for signed 32-bits integer variable         有符号32位整型变量      */
typedef float          		FP32;                    /** single precision floating point variable (32bits)   单精度浮点数（32位长度） */
typedef double         		DB64;                    /** double precision floating point variable (64bits)   双精度浮点数（64位长度） */
typedef UCHAR8            u8;                      /** defined for unsigned 8-bits integer variable 	        无符号8位整型变量  */
typedef USHORT16          u16;                     /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量 */
typedef UINT32            u32;                     /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量 */
typedef SCHAR8            s8;                      /** defined for unsigned 8-bits integer variable 	        无符号8位整型变量  */
typedef SSHORT16          s16;                     /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量 */
typedef SINT32            s32;                     /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量 */


#define REFEREE_DATA_LENGTH ((uint16_t) 103)
#define REFEREE_RECEIVE_SIZE 250
#define REFEREE_SEND_SIZE 112
#define REFEREE_MAP_SIZE 49
class Referee {
public:
    
    /**裁判系统相关**/
    ext_game_status_t						GameStatus;						// 比赛状态数据
    ext_game_result_t						GameResult;						// 比赛结果数据
    ext_game_robot_HP_t						GameRobotHP;					// 机器人血量数据
    ext_event_data_t						EventData;						// 场地事件数据
    ext_supply_projectile_action_t			SupplyProjectileAction;			// 补给站动作标识
    ext_referee_warning_t					RefereeWarning;					// 裁判警告信息
    ext_game_robot_status_t					GameRobotStatus;				// 比赛机器人状态
    ext_power_heat_data_t					PowerHeatData;					// 实时功率热量数据
    ext_game_robot_pos_t					GameRobotPos;					// 机器人位置
    ext_buff_t								Buff;							// 机器人增益
    ext_robot_hurt_t						RobotHurt = { .armor_id = 9, .hurt_type = 9};						// 伤害状态
    ext_shoot_data_t						ShootData;						// 实时射击信息
    ext_bullet_remaining_t					BulletRemaining;				// 子弹剩余发射数目
    ext_rfid_status_t						RFIDStatus;						// 机器人RFID状态
    ext_student_interactive_header_data_t	StudentInteractiveHeaderData;	// 机器人间交互信息
    ext_robot_command_t                     RobotCommand;	                // 云台手小地图指令
    robot_interactive_data_t				RobotInteractiveData;			// 交互数据       
    map_sentry_data_t                       MapSentryData;

    Usart* usart_referee;

    uint8_t m_data_send_buff[REFEREE_SEND_SIZE];
    uint8_t m_data_receive_buff[REFEREE_RECEIVE_SIZE];
    int8_t m_loc_delta_x_total;
    int8_t m_loc_delta_y_total;
    uint16_t m_loc_pos_x_pre;
    uint16_t m_loc_pos_y_pre;
    uint8_t m_loc_send_cnt;
    uint8_t m_loc_update_cnt;
    bool m_loc_send_init_flag;
    uint8_t m_loc_queue_len;
    uint16_t m_enemy_1_HP_pre;
    uint16_t m_enemy_2_HP_pre;
    uint16_t m_enemy_3_HP_pre;
    uint16_t m_enemy_4_HP_pre;
    uint16_t m_enemy_5_HP_pre;

    // TEST
    // uint8_t temp_cnt;
    // uint16_t temp_x;
    // uint16_t temp_y;
    // TEST

    Referee(void){
        m_data_send_buff[0] = 0xA5;
        m_data_send_buff[1] = (uint8_t) REFEREE_DATA_LENGTH;
        m_data_send_buff[2] = (uint8_t) (REFEREE_DATA_LENGTH >> 8);
        m_data_send_buff[3] = 0x00;
        Append_CRC8_Check_Sum(m_data_send_buff,5);

        m_data_send_buff[5] = (uint8_t) MapSentryDataID;
        m_data_send_buff[6] = (uint8_t) (MapSentryDataID >> 8);

        m_loc_delta_x_total = 0;
        m_loc_delta_y_total = 0;
        m_loc_pos_x_pre = 0;
        m_loc_pos_y_pre = 0;
        m_loc_send_cnt = 0;
        m_loc_update_cnt = 0;
        m_loc_send_init_flag = false;
        m_loc_queue_len = 0;

        m_enemy_1_HP_pre = 1;
        m_enemy_2_HP_pre = 1;
        m_enemy_3_HP_pre = 1;
        m_enemy_4_HP_pre = 1;
        m_enemy_5_HP_pre = 1;
        // TEST
        // uint8_t temp_cnt = 0;
        // uint16_t temp_x = 0;
        // uint16_t temp_y = 0;
        // TEST
        GameStatus={0};
        GameResult={0};
        GameRobotHP={0};
        EventData={0};
        SupplyProjectileAction={0};
        RefereeWarning={0};
        GameRobotStatus={0};
        PowerHeatData={0};
        GameRobotPos={0};
        Buff={0};
        ShootData={0};
        BulletRemaining={0};
        RFIDStatus={0};
        StudentInteractiveHeaderData={0};
        RobotCommand={0};
        RobotInteractiveData={0};
        MapSentryData={0};

        usart_referee = new Usart();
    }
    
    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};

/*接收到的来自裁判系统的字节在数据包中位置*/
typedef enum {
	RS_RX_FREE = 0,
	RS_RX_Length = 1,
	RS_RX_Num = 2,
	RS_RX_CRC8 = 3,
	RS_RX_CmdID = 4,
	RS_RX_Data = 5,
	RS_RX_CRC16 = 6,
}RS_RX_Status;