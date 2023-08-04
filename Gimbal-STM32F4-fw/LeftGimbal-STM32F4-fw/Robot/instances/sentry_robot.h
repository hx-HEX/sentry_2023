#pragma once
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "bmi088.h"
#include "ewma.h"

// The encoder value corresponding to mechanical zero of the gimbal motor
#define GIMBAL_PITCH_ENCODER_ZERO_VALUE    ( (uint32_t) 3380 )
#define GIMBAL_YAW_ENCODER_ZERO_VALUE    ( (uint32_t) 3236 )

// The encoder value corresponding to mechanical zero of the fric motor
#define LEFT_FRIC_ENCODER_ZERO_VALUE            ( (uint32_t) 123 )
#define RIGHT_FRIC_ENCODER_ZERO_VALUE           ( (uint32_t) 234 )

// The resolution of encoder
#define ENCODER_RESOLUTION ( (uint32_t) 8192)

// The reduction ratio of fric motor
#define FRIC_MOTOR_REDUCTION_RATIO ( (uint32_t) 1 )

// The reduction ratio of gimbal motor
#define GIMBAL_MOTOR_REDUCTION_RATIO ( (uint32_t) 1 )

// Motor ID(same type motors should have different ID)
#define GIMBAL_PITCH_MOTOR_ID             ( (uint8_t) 1 )
#define GIMBAL_YAW_MOTOR_ID              ( (uint8_t) 2 )
#define LEFT_FRIC_MOTOR_ID                      ( (uint8_t) 1 )
#define RIGHT_FRIC_MOTOR_ID                     ( (uint8_t) 2 )

// Motor number
#define SHOOT_MOTOR_NUM ( (uint8_t) 2 )
#define GIMBAL_MOTOR_NUM ( (uint8_t) 2 )

// Robot mannal control sensitivity
#define GIMBAL_PITCH_ANGLE_MANNAL_CONTROL_SENSITIVITY       ( (float) 100.0f )
#define GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY         ( (float) 100.0f )                                 

// IMU number
#define BMI088_IMU_NUM ( (uint8_t) 1 )

#define IMU_CALIBRATE                                       ((bool)false)

#define FRIC_WHEEL_SPEED                ((float)6800.0f)
#define FRIC_WHEEL_SPEED_SAFE           ((float)6500.0f)

#define FITCH_SPEED_IMU_FEEDBACK
// #define FITCH_SPEED_ENCODER_FEEDBACK
// #define FITCH_ANGLE_IMU_FEEDBACK
#define FITCH_ANGLE_ENCODER_FEEDBACK

#define YAW_SPEED_IMU_FEEDBACK
// #define YAW_SPEED_ENCODER_FEEDBACK
// #define YAW_ANGLE_IMU_FEEDBACK
#define YAW_ANGLE_ENCODER_FEEDBACK

#define PITCH_ANGLE_MAX              30
#define PITCH_ANGLE_MIN             -30
#define YAW_ANGLE_MAX                196
#define YAW_ANGLE_MIN               -23

#define GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MIN                   ( (float) -15.0f )
#define GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MAX                   ( (float) 15.0f )
#define GIMBAL_YAW_ANGLE_SCAN_ANGLE_MIN                     ( (float) -15.0f )
#define GIMBAL_YAW_ANGLE_SCAN_ANGLE_MAX                     ( (float) 180.0f )

#define GIMBAL_PITCH_ANGLE_SCAN_SPEED                       ( (float) 0.04f )
#define GIMBAL_YAW_ANGLE_SCAN_SPEED                         ( (float) 0.1f )

#define Pitch_Vison_Error                          ((float) 1.0f)
#define Yaw_Vision_Error                           ((float) 2.0f)


class SentryRobot {
public:
    enum GimbalMode {
        GIMBAL_SAFE = 0,
        GIMBAL_MOVE,
        GIMBAL_AUTO
    };
    GimbalMode gimbal_mode;
    GimbalMode gimbal_mode_pre;

    enum ShootMode {
        SHOOT_SAFE = 0,
        SHOOT_MOVE
    };
    ShootMode shoot_mode;

    enum ShootMotor {
        LEFT_FRIC_MOTOR = 0,
        RIGHT_FRIC_MOTOR,
    };
    M3508* shoot_motor[SHOOT_MOTOR_NUM];

    enum GimbalMotor {
        GIMBAL_PITCH_MOTOR = 0,
        GIMBAL_YAW_MOTOR
    };
    GM6020* gimbal_motor[GIMBAL_MOTOR_NUM];

    enum BMI088IMU {
        GIMBAL_FIRST_IMU = 0,
    };
    BMI088* gimbal_imu[BMI088_IMU_NUM];

    EWMA* m_shoot_speed_filter;

    uint8_t enemy_find_flag;
    uint8_t change_yaw_flag;
    uint8_t change_pitch_flag;
    uint8_t change_scan_flag;
    uint8_t yaw_scan_dir;
    uint8_t pitch_scan_dir;
    uint8_t direction_flag;
    
    //主控通讯
    //接收
    uint8_t shoot_flag;
    uint8_t run_flag;
    uint8_t balance_infantry_num;
    uint8_t enemy_color;
    uint8_t m_scan_flag;

    float pitch_rc_des;
    float yaw_rc_des;
    float bullet_speed;

    //发送
    uint8_t supply_bullet_flag;
    uint8_t enemy_number;


    uint16_t m_vision_delay_cnt;

    float yaw_angle_des;
    float pitch_angle_des;

    uint8_t test_flag;
    float test_pitch_des;
    float test_yaw_des;

    void Init(void);

    void MoveShoot(void);
    void MoveGimbal(void);
    void SetGimbalAngleTarget(float target_p , float target_y);
    void SetFricWheelSpeedTarget(float l_target, float r_target);
    void UpdateGimbalPitchState(float angle,float speed);
    void UpdateGimbalYawState(float angle,float speed);
    void UpdateFricWheelState(float l_speed,float r_speed);
    void ExecuteGimbalAlgorithm(void);
    void ExecuteShootAlgorithm(void);
    inline void SetGimbalMode(GimbalMode mode) {gimbal_mode_pre = gimbal_mode;gimbal_mode = mode;};
    inline void SetShootMode(ShootMode mode) {shoot_mode = mode;};

    void SendControlCommand(void);

    SentryRobot()
    {
        m_scan_flag = 1;
        enemy_find_flag = 0;
        change_yaw_flag = 0;
        change_pitch_flag = 0;
        change_scan_flag = 0;

        m_vision_delay_cnt = 0;
    
        yaw_angle_des = 0;
        pitch_angle_des = 0;
    };
    ~SentryRobot(){};

private:
    void InitAllActuators(void);
    void InitAllSensors(void);
};