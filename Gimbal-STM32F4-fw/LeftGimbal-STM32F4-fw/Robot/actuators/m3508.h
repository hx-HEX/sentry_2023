#pragma once

#include "pid.h"
#include "adrc.h"
#include "can.h"
#include "encoder.h"
#include "smc.h"

class M3508 {
public:
    float m_speed_target;
    float m_speed_current;
    float m_angle_current;
    float m_angle_target;

	unsigned int m_reduction_ratio; // motor reducer reduction ratio

    Adrc_TD* m_angle_td;
    Pid* m_speed_pid;
    Pid* m_angle_pid;
    Smc* m_smc;

    /* add other control algorithms */

    /* add other control algorithms */

    CAN_TypeDef* m_CANx;
    AbsEncoder* m_encoder;
    uint32_t m_id;
    uint32_t m_state_update_times;

    void SpeedUpdate(float speed);
    void AngleUpdate(uint32_t value);
    void SpeedControl(void);
    void AngleControl(void);

    M3508(CAN_TypeDef* CANx, uint32_t id, uint32_t ration) { 
        m_CANx = CANx;
        m_id = id;
        m_reduction_ratio = ration;
        m_state_update_times = 0;
        m_speed_target = 0;
        m_speed_current = 0;
        m_angle_current = 0;
        m_angle_target = 0;

    };
    ~M3508();
};