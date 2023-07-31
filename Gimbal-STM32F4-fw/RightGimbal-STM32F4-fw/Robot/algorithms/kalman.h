#pragma once

class Kalman {
public:
    Kalman(unsigned char order, float T, float P_init, float Q, float R);

    void UpdateFilter (float origin);
    float GetFilterOutput(void) { return m_X_update; }
private:
    unsigned char m_order;
    float m_T;
    float m_Q;
    float m_R;
    float m_K;
    float m_X_origin;
    float m_P_init;
    float m_X_prediction;
    float m_P_prediction;
    float m_X_update;
    float m_P_update;
    float m_X_update_pre;
    float m_P_update_pre;
    bool m_first_flag;

    void CalPrediction(void);
    void CalUpdate(void);
};