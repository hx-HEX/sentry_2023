#include "can_protocol.h"


static s32 GetEncoderNumber(u8* pData);
static s32 GetMotorSpeed(u8* pData);

void CAN_Decode(CANContext* context){
    if(context->CANx == CAN1){
        switch(context->CANx_RxMsg.StdId)
        {
            case (0x200 + LEFT_FRIC_MOTOR_ID):	
                G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpFB = 
                GetMotorSpeed(&(context->CANx_RxMsg.Data[0]));
                G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_state_update_times++;
                break;  
            case (0x200 + RIGHT_FRIC_MOTOR_ID):	
                G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpFB = 
                GetMotorSpeed(&(context->CANx_RxMsg.Data[0]));
                G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_state_update_times++;
                break;      
        }
    }
    else if(context->CANx == CAN2){
        switch(context->CANx_RxMsg.StdId)
        {
           case (0x204 + GIMBAL_PITCH_MOTOR_ID):	
                G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.gimbal_motor[SentryRobot::GIMBAL_PITCH_MOTOR]->m_state_update_times++;
                break;              
            case (0x204 + GIMBAL_YAW_MOTOR_ID):	
                G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.gimbal_motor[SentryRobot::GIMBAL_YAW_MOTOR]->m_state_update_times++;
                break;    
        }
    }
}

/**
 *@brief Read the motor speed returned by the encoder, the available range is C610, C620, 6020 motor
 *@param pData original data array
 *@return motor speed
*/
s32 GetMotorSpeed(u8* pData)
{
	s32 speed_temp;
	s32 base_value = 0xFFFF;
	if(pData[2] & 0x01<<7 )
	{	speed_temp = (base_value<<16 | pData[2]<<8 | pData[3]);}
	else
	{	speed_temp = pData[2]<<8 | pData[3];}
	return speed_temp;
}



/**
 *@brief Read the mechanical angle returned by the encoder, the available range is C610, C620, 6020 motor
 *@param pData original data array
 *@return mechanical angle (0~8191)
*/
s32 GetEncoderNumber(u8* pData)
{
    s32 encoder_temp;
	encoder_temp = pData[0]<<8 | pData[1];
	return encoder_temp;
}