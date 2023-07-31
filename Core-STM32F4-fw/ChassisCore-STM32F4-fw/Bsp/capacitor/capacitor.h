#pragma once
#include "pid.h"

#define POWER_CHARGING_DEFAULT  ( (float)100.0f )

class Capacitor{
public:

    typedef enum 
    {
        Pow_Unlimited = 0,
        Pow_Limited,
        Pow_Stop
    }Power_Control_State;

    Power_Control_State CAP_STATE;
    Power_Control_State Pre_CAP_STATE;
    
    /*receive message, id 0x400*/
    float m_cap_vol;
    float m_pow_in;
    float m_pow_out;
	float m_volt_out;
	unsigned char m_cap_error;

    /*send message, id 0x220*/
    float m_power_charging;

    float m_power_now;
    float m_power_buffer_next;
    float m_power_buffer_now;
    float m_power_limit;
    bool m_power_warning;
    unsigned int m_state_update_times;

    /*Discharge strategy variable*/
    bool FLAG_CAP_Vol_1;
    bool FLAG_CAP_Vol_2;
    int CAP_Vol_cnt;
    bool Cap_Warning;
    double CAP_POW_USE;
    double CAP_Cur_Coe;
    bool FLAG_CAP_Pow;
    unsigned char Cap_Vol_Pow_Limit_Flag;

    Pid *m_power_control_pid;

    void UpdateChargingPower(float power_now, float power_buffer_now, float power_limit);

    void PowerJudgeByVol(void);
    void PowerDesUpdate(float power_limit);
    void CapVolLimit(void);

    Capacitor() {
        m_power_charging = POWER_CHARGING_DEFAULT;
        m_power_warning = false;
        m_state_update_times = 0;

        CAP_Vol_cnt=0;

        FLAG_CAP_Vol_1 = false;
        FLAG_CAP_Vol_2 = false;
        FLAG_CAP_Pow = false;

        CAP_POW_USE = 1;
        CAP_Cur_Coe = 1;

        Cap_Vol_Pow_Limit_Flag = 0;
        m_cap_vol = 0;
    };
};