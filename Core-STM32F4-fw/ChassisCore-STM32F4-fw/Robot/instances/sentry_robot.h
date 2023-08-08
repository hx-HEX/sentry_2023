#pragma once
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "RoboMaster.h"
#include "capacitor.h"
#include "kalman.h"
#include "ewma.h"


// The encoder value corresponding to mechanical zero of the chassis motor
#define CHASSIS_FLL_ENCODER_ZERO_VALUE ( (int32_t) 789 )
#define CHASSIS_FRL_ENCODER_ZERO_VALUE ( (int32_t) 890 )
#define CHASSIS_BLL_ENCODER_ZERO_VALUE ( (int32_t) 901 )
#define CHASSIS_BRL_ENCODER_ZERO_VALUE ( (int32_t) 1234 )
#define YAW_GIMBAL_LEFT_ENCODER_ZERO_VALUE ( (int32_t) 0 )
#define YAW_GIMBAL_RIGHT_ENCODER_ZERO_VALUE ( (int32_t) 0 )

// The resolution of encoder
#define ENCODER_RESOLUTION ( (uint32_t) 8192)

// The reduction ratio of chassis motor
#define CHASSIS_SPEED_MOTOR_REDUCTION_RATIO ( (float) 115.853271 )
#define YAW_GIMBAL_MOTOR_REDUCTION_RAITO ((float) 115.846558)

// Motor ID(same type motors should have different ID)
#define CHASSIS_FRL_MOTOR_ID ( (uint8_t) 3)
#define CHASSIS_BRL_MOTOR_ID ( (uint8_t) 2 )
#define CHASSIS_BLL_MOTOR_ID ( (uint8_t) 1 )
#define CHASSIS_FLL_MOTOR_ID ( (uint8_t) 4 )
#define YAW_GIMBAL_LEFT_MOTOR_ID ( (uint8_t) 1)
#define YAW_GIMBAL_RIGHT_MOTOR_ID ( (uint8_t) 3)

// Motor number
#define CHASSIS_LINE_MOTOR_NUM ( (uint8_t) 4 )
#define YAW_GIMBAL_NUM         ( (uint8_t) 2 )

// Robot Initial State
#define CHASSIS_INIT_SPEED ( (float) 0.00f )
#define CHASSIS_CALIBRATE_ENCODE_RANGE ( (uint32_t) 200 )

// Robot mannal control sensitivity
#define CHASSIS_SPEED_MANNAL_CONTROL_SENSITIVITY            ( (float) 100.0f )

// Chassis driving mode
#define CHASSIS_OMNIDIRECTIONAL_MODE
                                  

#ifdef CHASSIS_OMNIDIRECTIONAL_MODE
#define CHASSIS_MANNAL_LINE_SPEED_MAX                              ( (float)(2.0) )
#define CHASSIS_MANNAL_SPIN_SPEED_MAX                                ( (float)(1.0) )
#define CHASSIS_LINE_SPEED_MAX                              ( (float)(2.0) )
#define CHASSIS_SPIN_SPEED_MAX                              ( (float)(1.0) )
#endif


#define FORWARD_DAMP_FACTOR                                     ((float)(800))
#define WHEEL_CURRENT_DEAD_AREA                                   ((float)(1000))

#define PRESS_C                                       67
#define PRESS_F                                       70
#define PRESS_G                                       71
#define PRESS_V                                       86                                      
#define PRESS_X                                       88
#define PRESS_Z                                       90

#define CHASSIS_WHEEL_RADIUS                                (float)(0.078f)

#define CHASSIS_ZERO_DIRECTION_ERR                           (float)(-157.5f)

#define CHASSIS_HEAD_FRONT         0
#define CHASSIS_HEAD_RIGHT        -90
#define CHASSIS_HEAD_LEFT          90
#define CHASSIS_HEAD_BACK         -180


class SentryRobot {
public:
    enum ChassisMode {
        CHASSIS_SAFE = 0,
        CHASSIS_MANNAL,
        CHASSIS_AUTO
    };
    ChassisMode chassis_mode;
    ChassisMode chassis_mode_pre;

    enum YawGimbalMode{
        YAWGIMBAL_SAFE = 0,
        YAWGIMBAL_CALIBRATE,
        YAWGIMBAL_MOVE,
        YAWGIMBAL_STATIC
    };
    YawGimbalMode yawgimbal_mode;
    YawGimbalMode yawgimbal_mode_pre;

    enum ChassisLineMotor {
        CHASSIS_FRL_MOTOR = 0,
        CHASSIS_FLL_MOTOR,
        CHASSIS_BLL_MOTOR,
        CHASSIS_BRL_MOTOR
    };

    enum YawGimbal{
        LEFT_MOTOR = 0,
        RIGHT_MOTOR
    };

    M3508* yaw_gimbal_motor[YAW_GIMBAL_NUM];
    M3508* chassis_line_motor[CHASSIS_LINE_MOTOR_NUM];
    Capacitor* capacitor;

    float m_yaw_angle;
    float m_speed_angle;

    uint8_t spinning_flag;
    uint8_t ctrl_spinning_flag;
    uint8_t vision_flag;
    uint8_t safe_flag;

    uint8_t yaw_calibrate_flag;
    uint8_t yaw_calibrate_flag_pre;

    uint8_t gimbal_test_flag;

    float chassis_speed;

// steer drive value

#ifdef  CHASSIS_OMNIDIRECTIONAL_MODE
    float m_world2chassis_angle;
    float m_world_vx;
    float m_world_vy;
    float m_chassis_w;
#endif

    void Init(void);
    void ExecuteYawGimbakAlgorithm(void);
    void ExecuteChassisAlgorithm(void);
    void SetChassisSpeedTarget(float fll_motor, float frl_motor, float bll_motor, float brl_motor);
    void SendControlCommand(void);
    inline void SetChassisMode(ChassisMode mode) {chassis_mode_pre = chassis_mode; chassis_mode = mode;};
    inline void SetYawGimbalMode(YawGimbalMode mode) {yawgimbal_mode_pre = yawgimbal_mode; yawgimbal_mode = mode;};
    void Power_Control(void);
    
    SentryRobot(void);
    ~SentryRobot(){};

private:
    void InitAllActuators(void);
    void InitAllSensors(void);
};
