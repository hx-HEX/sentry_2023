#pragma once

class EWMA{
public:
    float m_avg;
    float m_beta;
    bool m_init_flag;
    EWMA(float beta){
        m_beta = (beta > 1.0)?1.0:(beta < 0.0)?0.0:beta;
        m_avg = 0.0;
        m_init_flag = false;
    }
    void Update(float value);
};