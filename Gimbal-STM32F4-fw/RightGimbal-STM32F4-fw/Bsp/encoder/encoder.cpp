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
	if (!m_init_flag) {
		m_raw_value = value;
		m_pre_raw_value = value;
		m_sum_value = (float)value;
		m_init_flag = true;
	} else {
		m_pre_raw_value = m_raw_value;
		m_raw_value = value;
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
	}
}