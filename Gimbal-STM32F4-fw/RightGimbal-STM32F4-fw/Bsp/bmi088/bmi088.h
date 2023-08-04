#pragma once

#include "gpio.h"
#include "spi.h"
#include "mahony.h"
#include "kalman.h"


#define  GYRO_DISABLE    GPIO_SetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12
#define  GYRO_ENABLE    GPIO_ResetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12

#define  ACC_DISABLE    GPIO_SetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8
#define  ACC_ENABLE    GPIO_ResetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8

#define BMI088_LONG_DELAY_TIME              80
#define BMI088_COM_WAIT_SENSOR_TIME         150

#define BMI088_Write_ACCEL_Reg_Num          6
#define BMI088_Write_GYRO_Reg_Num           6

// m/(s^2)
// (1 / 32768 * n * g)
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

// rad/s
// (1 / 32767 * n / 180 * pi)
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN  0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN  0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN  0.000066579027251980956150958662738366f

// sentry robot
#define GYRO_X_OFFSET          0.000392445538
#define GYRO_Y_OFFSET          -0.000426955899
#define GYRO_Z_OFFSET          0.000955480093

// temp 
// #define GYRO_X_OFFSET             -0.000506517827
// #define GYRO_Y_OFFSET             0.00216058665
// #define GYRO_Z_OFFSET             -0.00288705388

#define OFFSET_CALIBRATE_TIMES    2500000



class BMI088 {
public:
    enum {
        BMI088_NO_Error = 0x00,
        BMI088_ACC_PWR_CTRL_Error = 0x01,
        BMI088_ACC_PWR_CONF_Error = 0x02,
        BMI088_ACC_CONF_Error = 0x03,
        BMI088_ACC_SELF_TEST_Error = 0x04,
        BMI088_ACC_RANGE_Error = 0x05,
        BMI088_INT1_IO_CTRL_Error = 0x06,
        BMI088_INT_MAP_DATA_Error = 0x07,
        BMI088_GYRO_RANGE_Error = 0x08,
        BMI088_GYRO_BANDWIDTH_Error = 0x09,
        BMI088_GYRO_LPM1_Error = 0x0A,
        BMI088_GYRO_CTRL_Error = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_Error = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_Error = 0x0D,

        BMI088_SELF_TEST_ACCEL_Error = 0x80,
        BMI088_SELF_TEST_GYRO_Error = 0x40,
        BMI088_NO_Sensor = 0xFF,
    };

    struct Bmi088AccData {
        float x;
        float y;
        float z;
    };
    struct Bmi088GyroData {
        float x;
        float y;
        float z;
    };

    Bmi088AccData m_acc_data;
    Bmi088GyroData m_gyro_data;
    Mahony* m_mahony_filter;
    Kalman* m_kalman_filter_gyro_x;
    Kalman* m_kalman_filter_gyro_z;
    Kalman* m_kalman_filter_gyro_y;
    float m_gyro_offset_x;
    float m_gyro_offset_y;
    float m_gyro_offset_z;

    BMI088(){}

    unsigned char Init(bool calibrate_flag);
    void GetGyroOffset(bool reset);
    void UpdataAccData(void);
    void UpdataGyroData(void);
    void UpdataAngleData(void);

private:

    unsigned char AccInit(void);
    unsigned char AccSelfTest(void);
    unsigned char GyroInit(void);
    unsigned char GyroSelfTest(void);
    void ReadAccSingleData(unsigned char reg_id, unsigned char *pData);
    void ReadGyroSingleData(unsigned char reg_id, unsigned char *pData);
    void ReadAccMultiplyData(unsigned char reg_id, unsigned char *pData, unsigned char size);
    void ReadGyroMultiplyData(unsigned char reg_id, unsigned char *pData, unsigned char size);
    void WriteAccSingleData(unsigned char reg_id, unsigned char data);
    void WriteGyroSingleData(unsigned char reg_id, unsigned char data);

    void WriteSingleReg(unsigned char reg_id, unsigned char data);
    void ReadSingleReg(unsigned char reg_id, unsigned char* pData);
    void ReadMultiplyReg(unsigned char reg_id, unsigned char* pData, unsigned char size);
    unsigned char WriteReadByte(unsigned char txdata);
};
