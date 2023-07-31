#pragma once

#include "pid.h"
#include "adrc.h"
#include "can.h"
#include "encoder.h"
#include "kalman.h"


class M2006 {
public:
    float m_speed_target;
    float m_speed_current;
    float m_angle_current;
    float m_angle_target;
    float m_angle_current_encoder;
    float m_speed_current_encoder;
    float m_angle_current_encoder_filter;
    float m_speed_current_encoder_filter;
    float m_angle_current_external;
    float m_speed_current_external;

    // motor reducer reduction ratio
	unsigned int m_reduction_ratio; 

    // control algorithms
    Adrc_TD* m_angle_td;
    Pid* m_speed_pid;
    Pid* m_angle_pid;

    // filer algorithms
    Kalman* m_kalman_filter_angle;
    Kalman* m_kalman_filter_speed;

    CAN_TypeDef* m_CANx;
    AbsEncoder* m_encoder;
    uint32_t m_id;
    uint32_t m_state_update_times;

    void SpeedUpdate(uint8_t type, float input);
    void EncoderSpeedUpdate(float speed);
    void AngleUpdate(uint8_t type, float input);
    void EncoderAngleUpdate(uint32_t value);
    void SpeedControl(void);
    void AngleControl(void);

    M2006(CAN_TypeDef* CANx, uint32_t id, uint32_t ration) { 
        m_CANx = CANx;
        m_id = id;
        m_reduction_ratio = ration;
        m_state_update_times = 0;
        m_speed_target = 0;
        m_angle_target = 0;
        m_speed_current = 0;
        m_angle_current = 0;
        m_angle_current_encoder = 0;
        m_speed_current_encoder = 0;
        m_angle_current_encoder_filter = 0;
        m_speed_current_encoder_filter = 0;
        m_angle_current_external = 0;
        m_speed_current_external = 0;
    };
    ~M2006();
};
