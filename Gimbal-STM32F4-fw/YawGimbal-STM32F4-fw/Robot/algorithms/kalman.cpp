#include "kalman.h"



/**
*@brief construct kalman filter
* 
*@param 
*/
Kalman::Kalman (unsigned char order, float T, float P_init, float Q, float R) 
{
    m_order = order;
    m_T = T;
    m_P_init = P_init;
    m_Q = Q;
    m_R = R;

    m_X_prediction = 0;
    m_X_update = 0;
    m_X_update_pre = 0;
    m_P_prediction = 0;
    m_P_update = 0;
    m_P_update_pre = 0;
    m_K = 0;
    m_first_flag = true;
}

/**
*@brief pass new origin data to update the kalman filter 
* 
*@param 
*/
void Kalman::UpdateFilter (float origin)
{
    if (m_first_flag) {
        m_first_flag = false;
        m_X_update = origin;
        m_P_update = m_P_init;
    }

    m_X_origin = origin;
    m_X_update_pre = m_X_update;
    m_P_update_pre = m_P_update;
    CalPrediction();
    CalUpdate();
}



/**
*@brief calculate the prediction part of the kalman filter 
* 
*@param 
*/
void Kalman::CalPrediction(void)
{
    // First-order Kalman filter
    if (m_order == 1) {
        m_X_prediction = m_X_update_pre;
        m_P_prediction = m_P_update_pre + m_Q;
    } 
    // Second-order Kalman filter
    else if (m_order == 2) {
    }
}



/**
*@brief calculate the update part of the kalman filter 
* 
*@param 
*/
void Kalman::CalUpdate(void)
{
    // First-order Kalman filter
    if (m_order == 1) {
        m_K = m_P_prediction * (m_P_prediction + m_R);
        m_X_update = m_X_prediction + m_K * (m_X_origin - m_X_prediction);
        m_P_update = (1 - m_K) * m_P_prediction;
    } 
    // Second-order Kalman filter
    else if (m_order == 2) {
    }
}