#include "can_protocol.h"


static s32 GetEncoderNumber(u8* pData);
static s32 GetMotorSpeed(u8* pData);

void CAN_Decode(CANContext* context){
    if(context->CANx == CAN1){
        switch(context->CANx_RxMsg.StdId)
        {
            case (0x200 + CHASSIS_FRL_MOTOR_ID):	
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_FRL_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_FRL_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_FRL_MOTOR]->m_state_update_times++;
                break;
            case (0x200 + CHASSIS_FLL_MOTOR_ID):	
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_FLL_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_FLL_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_FLL_MOTOR]->m_state_update_times++;
                break;              
            case (0x200 + CHASSIS_BLL_MOTOR_ID):	
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_BLL_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_BLL_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_BLL_MOTOR]->m_state_update_times++;
                break;
            case (0x200 + CHASSIS_BRL_MOTOR_ID):	
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_BRL_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_BRL_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.chassis_line_motor[G_sentry.CHASSIS_BRL_MOTOR]->m_state_update_times++;
                break;
        }
    }
    else if(context->CANx == CAN2){
        switch(context->CANx_RxMsg.StdId)
        {
            case 0x400:  
                G_sentry.capacitor->m_cap_vol   = (float)( ( context->CANx_RxMsg.Data[0] << 8 ) | (context->CANx_RxMsg.Data[1] ) ) / 100.0f;
                G_sentry.capacitor->m_pow_in    = (float)( ( context->CANx_RxMsg.Data[2] << 8 ) | (context->CANx_RxMsg.Data[3] ) ) / 100.0f;
                G_sentry.capacitor->m_pow_out   = (float)( ( context->CANx_RxMsg.Data[4] << 8 ) | (context->CANx_RxMsg.Data[5] ) ) / 100.0f;
                G_sentry.capacitor->m_volt_out  = (float)( ( context->CANx_RxMsg.Data[6] << 8 ) | (context->CANx_RxMsg.Data[7] ) ) / 100.0f;	
                G_sentry.capacitor->m_state_update_times++;
                break;
            case (0x200 + YAW_GIMBAL_LEFT_MOTOR_ID):	
                G_sentry.yaw_gimbal_motor[G_sentry.LEFT_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.yaw_gimbal_motor[G_sentry.LEFT_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.yaw_gimbal_motor[G_sentry.LEFT_MOTOR]->m_state_update_times++;
                break;
            case (0x200 + YAW_GIMBAL_RIGHT_MOTOR_ID):	
                G_sentry.yaw_gimbal_motor[G_sentry.RIGHT_MOTOR]->
                EncoderAngleUpdate(GetEncoderNumber(&(context->CANx_RxMsg.Data[0])));
                G_sentry.yaw_gimbal_motor[G_sentry.RIGHT_MOTOR]->
                EncoderSpeedUpdate(GetMotorSpeed(&(context->CANx_RxMsg.Data[0])));
                G_sentry.yaw_gimbal_motor[G_sentry.RIGHT_MOTOR]->m_state_update_times++;
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
