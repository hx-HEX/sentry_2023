#pragma once
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "bmi088.h"
#include "ewma.h"

// The resolution of encoder
#define ENCODER_RESOLUTION ( (uint32_t) 8192)

// Motor number
#define YAW_GIMBAL_MOTOR_NUM ( (uint8_t) 2 )

// Robot mannal control sensitivity
#define GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY         ( (float) 100.0f )                                 

// IMU number
#define BMI088_IMU_NUM ( (uint8_t) 1 )

#define IMU_CALIBRATE                                       ((bool)false)

#define YAW_GIMBAL_LEFT_MOTOR_ID ( (uint8_t) 1)
#define YAW_GIMBAL_RIGHT_MOTOR_ID ( (uint8_t) 2)

#define YAW_GIMBAL_MOTOR_REDUCTION_RAITO ((uint32_t) 19)

#define YAW_GIMBAL_LEFT_ENCODER_ZERO_VALUE ( (int32_t) 0 )
#define YAW_GIMBAL_RIGHT_ENCODER_ZERO_VALUE ( (int32_t) 0 )


class SentryRobot {
public:
    enum GimbalMode {
        YAW_GIMBAL_SAFE = 0,
        YAW_GIMBAL_MOVE,
        YAW_GIMBAL_STATIC
    };
    GimbalMode gimbal_mode;

    enum GimbalMotor {
        LEFT_YAW_MOTOR = 0,
        RIGHT_YAW_MOTOR
    };
    M3508* yaw_gimbal_motor[YAW_GIMBAL_MOTOR_NUM];

    enum BMI088IMU {
        GIMBAL_FIRST_IMU = 0,
    };
    BMI088* gimbal_imu[BMI088_IMU_NUM];

    float yaw_angle_des;

    void Init(void);

    void SetGimbalAngleTarget(float target_p , float target_y);
    void ExecuteGimbalAlgorithm(void);
    inline void SetGimbalMode(GimbalMode mode) {gimbal_mode = mode;};

    void SendControlCommand(void);

    SentryRobot()
    {    
        yaw_angle_des = 0;
    };
    ~SentryRobot(){};

private:
    void InitAllActuators(void);
    void InitAllSensors(void);
};