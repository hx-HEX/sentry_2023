#pragma once
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "RoboMaster.h"
#include "kalman.h"
#include "ewma.h"
#include "photogate.h"

// The resolution of encoder
#define ENCODER_RESOLUTION ( (uint32_t) 8192)

// The reduction ratio of shoot motor
#define SHOOT_MOTOR_REDUCTION_RATIO ( (uint32_t) 36 )

// Motor ID(same type motors should have different ID)
#define LEFT_SHOOT_DRIVE_MOTOR_ID ( (uint8_t) 1 )
#define RIGHT_SHOOT_DRIVE_MOTOR_ID ( (uint8_t) 2 )

// Motor number
#define SHOOT_MOTOR_NUM ( (uint8_t) 2 )

// Robot mannal control sensitivity
#define CHASSIS_SPEED_MANNAL_CONTROL_SENSITIVITY            ( (float) 100.0f )
#define CHASSIS_ANGLE_MANNAL_CONTROL_SENSITIVITY            ( (float) 100.0f )
#define GIMBAL_PITCH_ANGLE_MANNAL_CONTROL_SENSITIVITY       ( (float) 0.05f )
#define GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY         ( (float) 0.05f )
#define GIMBAL_PITCH_ANGLE_SCAN_SPEED                       ( (float) 0.04f )
#define GIMBAL_YAW_ANGLE_SCAN_SPEED                         ( (float) 0.1f )
#define GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MIN                   ( (float) -10.0f )
#define GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MAX                   ( (float) 8.0f )
#define GIMBAL_YAW_ANGLE_SCAN_ANGLE_MIN                     ( (float) -180.0f )
#define GIMBAL_YAW_ANGLE_SCAN_ANGLE_MAX                     ( (float) 180.0f )
#define GIMBAL_PITCH_MAX                                    ( (float) 20.0f )
#define GIMBAL_PITCH_MIN                                    ( (float) -10.0f )
#define GIMBAL_YAW_MAX                                      ( (float) 180.0f )
#define GIMBAL_YAW_MIN                                      ( (float) -180.0f )

#define SHOOT_BULLET_FREQUENCY_TEST_0   0
#define SHOOT_BULLET_FREQUENCY_TEST_1   1
#define SHOOT_BULLET_FREQUENCY_TEST_2   8
#define SHOOT_BULLET_FREQUENCY_TEST_3   13
#define SHOOT_BULLET_FREQUENCY_TEST_4   17

#define SHOOT_DRIVER_SUPPLY_ANGLE_STEP  ((float)(29491 * 5 / 3))

#define SHOOT_SPEED_FILTER_BETA ((float)0.9)
                     
#define PRESS_C                                       67
#define PRESS_F                                       70
#define PRESS_G                                       71
#define PRESS_V                                       86                                      
#define PRESS_X                                       88
#define PRESS_Z                                       90

#define CHASSIS_MANNAL_SPIN_SPEED_MAX                                ( (float)(2.0) )
#define CHASSIS_SPIN_SPEED_MAX                                ( (float)(1.0) )

class SentryRobot {
public:
    enum ChassisMode {
        CHASSIS_SAFE = 0,
        CHASSIS_MANNAL,
        CHASSIS_AUTO
    };
    ChassisMode chassis_mode;
    ChassisMode chassis_mode_pre;

    enum GimbalMode {
        GIMBAL_SAFE = 0,
        GIMBAL_MANNAL,
        GIMBAL_AUTO
    };
    GimbalMode gimbal_mode;
    GimbalMode gimbal_mode_pre;

    enum ShootMode {
        SHOOT_SAFE = 0,
        SHOOT_MANNAL,
        SHOOT_AUTO
    };
    ShootMode shoot_mode;
    ShootMode shoot_mode_pre;

    enum YawGimbalMode {
        YAW_SAFE = 0,
        YAW_MOVE,
        YAW_STATIC
    };
    YawGimbalMode yawgimbal_mode;
    YawGimbalMode yawgimbal_mode_pre;

    enum GimbalControlMode {
        LEFT_MANNAL,
        RIGHT_MANNAL,
        LEFT_AUTO,
        RIGHT_AUTO,
        ALL_AUTO
    };
    GimbalControlMode gimbalrc_mode;
    GimbalControlMode gimbalauto_mode;

    enum ShootMotor {
        LEFT_SHOOT_DRIVE_MOTOR = 0,
        RIGHT_SHOOT_DRIVE_MOTOR
    };
    M2006* shoot_motor[SHOOT_MOTOR_NUM];

    Photogate* photogate;

    EWMA* m_shoot_speed_filter;

    //shoot
    bool left_shoot_insurance;
    uint32_t m_left_shoot_bullet_cnt_target;
    uint8_t m_left_shoot_bullet_fps;
    uint32_t m_left_shoot_bullet_last_increase_time;
    float left_shoot_driver_angle_target;

    bool right_shoot_insurance;
    uint32_t m_right_shoot_bullet_cnt_target;
    uint8_t m_right_shoot_bullet_fps;
    uint32_t m_right_shoot_bullet_last_increase_time;
    float right_shoot_driver_angle_target;

    uint8_t left_stop_shoot_flag;
    uint16_t m_left_stop_shoot_cnt;

    uint8_t right_stop_shoot_flag;
    uint16_t m_right_stop_shoot_cnt;

    float m_shoot_speed_pre_1;
    float m_shoot_speed_pre_2;

    //gimbal
    float left_pitch_des;
    float left_pitch_fb;
    float left_yaw_des;
    float left_yaw_fb;
    float right_pitch_des;
    float right_pitch_fb;
    float right_yaw_des;
    float right_yaw_fb;
    float yaw_des;
    float yaw_fb;
    uint8_t left_run_flag;
    uint8_t right_run_flag;
    uint8_t m_scan_flag;

    //chassis
    float m_world_vx;
    float m_world_vy;
    float m_chassis_w;

    float m_radar_world_yaw_angle;
    float m_navigation_x;
    float m_navigation_y;

    int16_t leftyaw_output;
    int16_t rightyaw_output;

    float m_robot_chassis_speed;
    float m_robot_wheel_world_yaw;

    uint8_t spinning_flag;

    //competition
    uint8_t vision_flag;
    uint8_t safe_flag;
    uint16_t m_shoot_cnt;

    bool balance_infantry_flag;
    float balance_infantry_num;
    bool m_enemy_1_res_flag;
    bool m_enemy_2_res_flag;
    bool m_enemy_3_res_flag;
    bool m_enemy_4_res_flag;
    bool m_enemy_5_res_flag;
    bool m_enemy_7_res_flag;
  
    uint16_t m_enemy_1_res_cnt;
    uint16_t m_enemy_2_res_cnt;
    uint16_t m_enemy_3_res_cnt;
    uint16_t m_enemy_4_res_cnt;
    uint16_t m_enemy_5_res_cnt;

    void Init(void);

    void ExecuteShootAlgorithm(void);
    void SetShootDriverTarget(float left_angle, float right_angle);
    void SendControlCommand(void);
    void IncreaseLeftShootBulletTarget(uint32_t time_now);
    void IncreaseRightShootBulletTarget(uint32_t time_now);
    inline void SetChassisMode(ChassisMode mode) {chassis_mode_pre = chassis_mode; chassis_mode = mode;};
    inline void SetGimbalMode(GimbalMode mode) {gimbal_mode_pre = gimbal_mode; gimbal_mode = mode;};
    inline void SetShootMode(ShootMode mode) {shoot_mode_pre = shoot_mode; shoot_mode = mode;};
    inline void SetYawGimbalMode(YawGimbalMode mode) {yawgimbal_mode_pre = yawgimbal_mode; yawgimbal_mode = mode;};
    
    SentryRobot(void);
    ~SentryRobot(){};

private:
    void InitAllActuators(void);
    void InitAllSensors(void);
};
