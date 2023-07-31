#include "smc.h"
#include "math.h"
#include "common_math.h"



/**
*@brief Sat function
* 
*@param 
*/
float SMC_SatFunc(float in, float d)
{
	if(fabs(in) >= d)
		return Sgn(in);
	else
		return in / d;
}



/**
*@brief Smc constructor
* 
*@param 
*/
Smc::Smc(float b, float eps, float gain, float dead, float output_max)
{
    m_b = b;
    m_eps = eps;
    m_gain = gain;
    m_dead = dead;
	m_fpUMax = output_max;

	m_fpDes = 0;
	m_fpFB = 0;
	m_fpU = 0;
}



/**
*@brief Smc constructor
* 
*@param 
*/
void Smc::CalSMC(void)
{
	m_TD->CalAdrcTD(m_fpDes);
	m_fpE = m_TD->m_x1 - m_fpFB;
	m_fpU = 1 / m_b * (m_TD->m_x2 + m_eps * SMC_SatFunc(m_fpE, m_dead) + m_gain * m_fpE);
	m_fpU = Clip(m_fpU, -m_fpUMax, m_fpUMax);
}