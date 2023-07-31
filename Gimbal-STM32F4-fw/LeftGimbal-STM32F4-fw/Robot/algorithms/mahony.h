#pragma once

#ifndef RADIANODEGREES         
#define RADIANODEGREES     57.29577951308232f  
#endif



class Mahony {
public:
    struct MahonyEulerAngle {
        float yaw;
        float pitch;
        float roll;
    };
    MahonyEulerAngle m_eular_angle;

    struct MahonyComGyro {
        float x;
        float y;
        float z;
    };
    MahonyComGyro m_com_gyro;

    struct MahonyEstiAcc {
        float x;
        float y;
        float z;
    };
    MahonyEstiAcc m_esti_acc;

    struct MahonyQuaternion {
        float q0;
        float q1;
        float q2;
        float q3;
    };
    MahonyQuaternion m_quaternion;

    void AngleUpdate(const float acc[3], const float gyro[3]);

    Mahony(float dt, float kp, float ki);
private:
    bool m_first_in_flag;
    float m_dt;
    float m_acc_error_kp;
    float m_acc_error_ki;
    float m_acc_error_integral_x;
    float m_acc_error_integral_y;
    float m_acc_error_integral_z;
};
