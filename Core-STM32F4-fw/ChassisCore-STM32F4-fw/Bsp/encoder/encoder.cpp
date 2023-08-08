#include "encoder.h"



/**
 * @brief absolute encoder data processing
 *
 * @param 
 *
 * @note 0~8192 is transformed into negative infinity ~ positive infinity
 *       Jump for the case of coded value "8191 (previous) -2 (next)"
 *       (The actual change is 3, but the difference before and after the return value is -8189)
**/
void AbsEncoder::EncodeValueUpdate(unsigned int value)
{
	m_raw_value = value;
	if (!m_init_flag)
	{
		m_sum_value = 0;
		m_init_flag = true;
		m_value_diff = m_raw_value - m_zero_value;
	}
	else
		m_value_diff = m_raw_value - m_pre_raw_value;
	if(m_value_diff < -m_resolution/2)
	{
		m_value_diff += m_resolution;
	}
	else if(m_value_diff > m_resolution/2)
	{
		m_value_diff -= m_resolution;
	}
	m_sum_value += m_value_diff;
	m_pre_raw_value = m_raw_value;
}
