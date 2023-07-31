#include "ewma.h"
#include "math.h"

void EWMA::Update(float value){
    if(m_init_flag == false){
        m_avg = value;
        m_init_flag = true;
    }else{
        m_avg = (m_beta * m_avg + (1 - m_beta) * value);
    }
}