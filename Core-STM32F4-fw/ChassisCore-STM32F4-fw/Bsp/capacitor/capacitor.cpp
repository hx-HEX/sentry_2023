#include "capacitor.h"

/*----------------------------------------------------------------------------------------
函数名：void UpdateChargingPower(float power_now, float power_buffer_now, float power_limit)
功  能：充电策略
----------------------------------------------------------------------------------------*/
void Capacitor::UpdateChargingPower(float power_now, float power_buffer_now, float power_limit)
{
    m_power_buffer_now = power_buffer_now;
    m_power_limit = power_limit;
    m_power_now = power_now;

	if( m_power_now > m_power_limit )
	{
		m_power_buffer_next = m_power_buffer_now - ( m_power_now - m_power_limit ) * 0.001f ; 
	}
	else
	{
		m_power_buffer_next = m_power_buffer_now ;                                   
	}
	
	if( m_power_buffer_next < 45 )
	{
		m_power_warning = true ;
	}
	else if( m_power_buffer_next >= 60 )
	{
		m_power_warning = false ;
	}	

    if ( m_power_warning ) {
        m_power_charging = m_power_limit - 8.0f;
    } else {
        m_power_charging = m_power_limit + 5.0f;
    }
}



/*----------------------------------------------------------------------------------------
函数名：void PowerJudgeByVol(void)
功  能：通过超级电容电压值来判断放电状态,保护电容
----------------------------------------------------------------------------------------*/
void Capacitor::PowerJudgeByVol(void)
{
	CAP_STATE = Pow_Unlimited;
	if(m_cap_vol < 19.8f)
	{
		if(m_cap_vol <=14.2f || FLAG_CAP_Vol_1 == true)
		{
			CAP_Vol_cnt++;
			if(CAP_Vol_cnt > 50)
			{
				FLAG_CAP_Vol_1 = true;
				CAP_STATE = Pow_Limited;
			}
		}
		else CAP_Vol_cnt = 0;
	}
	else FLAG_CAP_Vol_1 = false;

	if(m_cap_vol < 14.2f)
	{
		if(m_cap_vol <= 8.0f || FLAG_CAP_Vol_2 == true)
		{
			FLAG_CAP_Vol_2 = true;
			CAP_STATE = Pow_Stop;
		}
	}
	else FLAG_CAP_Vol_2 = false;

	if(CAP_STATE == Pow_Unlimited) Cap_Warning = false;
	else Cap_Warning = true;
}



/*----------------------------------------------------------------------------------------
函数名：void PowerDesUpdate(float power_limit)
功  能：功率限制状态下的功率输出更新
----------------------------------------------------------------------------------------*/
 void Capacitor::PowerDesUpdate(float power_limit)
 {
	CAP_POW_USE = m_pow_out/(power_limit*0.95f);

	if(CAP_POW_USE == 0) CAP_POW_USE = 0.0001;
 }



/*----------------------------------------------------------------------------------------
函数名：void CapVolLimit(void)
功  能：检录使用,防止电容电压超出27V
----------------------------------------------------------------------------------------*/
void Capacitor::CapVolLimit(void)
{
	if(m_cap_vol > 26.5f)
	{
		if(m_cap_vol > 27.0f || Cap_Vol_Pow_Limit_Flag == 1)
		{
			Cap_Vol_Pow_Limit_Flag = 1;
			m_power_charging = 0;
		}
	}
	else{
		Cap_Vol_Pow_Limit_Flag = 0;
	}
}