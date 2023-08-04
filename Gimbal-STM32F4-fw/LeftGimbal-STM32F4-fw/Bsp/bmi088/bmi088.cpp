#include "bmi088.h"
#include "bmi088reg.h"
#include "spi.h"
#include "delay.h"
#ifdef IMU_DEBUG
#include "stdio.h"
#endif

// configuration of ACC
static unsigned char write_BMI088_ACCEL_Reg_Data_Error[BMI088_Write_ACCEL_Reg_Num][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088::BMI088_ACC_PWR_CTRL_Error},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088::BMI088_ACC_PWR_CONF_Error},
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088::BMI088_ACC_CONF_Error},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088::BMI088_ACC_RANGE_Error},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088::BMI088_INT1_IO_CTRL_Error},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088::BMI088_INT_MAP_DATA_Error}
};
// configuration of GYRO
static unsigned char  write_BMI088_GYRO_Reg_Data_Error[BMI088_Write_GYRO_Reg_Num][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_1000, BMI088::BMI088_GYRO_RANGE_Error},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088::BMI088_GYRO_BANDWIDTH_Error},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088::BMI088_GYRO_LPM1_Error},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088::BMI088_GYRO_CTRL_Error},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088::BMI088_GYRO_INT3_INT4_IO_CONF_Error},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088::BMI088_GYRO_INT3_INT4_IO_MAP_Error}

};



/**
 * @brief Init the bmi088 IMU
 * 
 * @note 
 **/
unsigned char BMI088::Init(bool calibrate_flag) 
{
    unsigned char error = BMI088_NO_Error;

    // self test pass and init
    if (AccSelfTest() != BMI088_NO_Error)
    {
        error |= BMI088_SELF_TEST_ACCEL_Error;
        #ifdef IMU_DEBUG
        printf("AccSelfTest error: %x\n", error);
        #endif
    } else {
        error |= AccInit();
        #ifdef IMU_DEBUG
        if (error != BMI088_NO_Error) {
            printf("AccInit error: %x\n", error);
        } else {
            printf("AccInit successfully!\n");
        }
        #endif
    }

    if (GyroSelfTest() != BMI088_NO_Error)
    {
        error |= BMI088_SELF_TEST_GYRO_Error;
        #ifdef IMU_DEBUG
        printf("GyroSelfTest error: %x\n", error);
        #endif
    } else {
        error |= GyroInit();
        #ifdef IMU_DEBUG
        if (error != BMI088_NO_Error) {
            printf("GyroInit error: %x\n", error);
        } else {
            printf("GyroInit successfully!\n");
        }
        #endif
    }

    if (error == BMI088_NO_Error) {
        GetGyroOffset(calibrate_flag);
    }
    return error;
}
/**
 * @brief Get bmi088 IMU gyroscope offset
 * 
 * @note 
 **/
void BMI088::GetGyroOffset(bool reset) {
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    m_gyro_offset_x = 0;
    m_gyro_offset_y = 0;
    m_gyro_offset_z = 0;

    if (reset) {
        for (int i = 0; i < OFFSET_CALIBRATE_TIMES; i++) {
            UpdataGyroData();
            sum_x += m_gyro_data.x;
            sum_y += m_gyro_data.y;
            sum_z += m_gyro_data.z;
        }
        m_gyro_offset_x = sum_x / OFFSET_CALIBRATE_TIMES;
        m_gyro_offset_y = sum_y / OFFSET_CALIBRATE_TIMES;
        m_gyro_offset_z = sum_z / OFFSET_CALIBRATE_TIMES;
    } else {
        m_gyro_offset_x = GYRO_X_OFFSET;
        m_gyro_offset_y = GYRO_Y_OFFSET;
        m_gyro_offset_z = GYRO_Z_OFFSET;
    }
}



/**
 * @brief Init the accelerometer
 * 
 * @note 
 **/
unsigned char BMI088::AccInit(void)
{
    unsigned char res = 0;
    unsigned char write_reg_num = 0;

    //check commiunication
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    WriteAccSingleData(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_Sensor;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_Write_ACCEL_Reg_Num; write_reg_num++)
    {

        WriteAccSingleData(write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        ReadAccSingleData(write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][2];
        }
    }
    return BMI088_NO_Error;
}



/**
 * @brief accelerometer self test
 * 
 * @note 
 **/
unsigned char BMI088::AccSelfTest(void) 
{
    int16_t self_test_accel[2][3];

    unsigned char buf[6] = {0, 0, 0, 0, 0, 0};
    unsigned char res = 0;

    unsigned char write_reg_num = 0;

    static const unsigned char write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
    {
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_Error},
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_Error},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_Error},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_Error},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_Error},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_Error}
    };

    //check commiunication is normal
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    WriteAccSingleData(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    
    delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_Sensor;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
    {

        WriteAccSingleData(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        ReadAccSingleData(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        delay_ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
    {

        WriteAccSingleData(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        ReadAccSingleData(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        delay_ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        ReadAccMultiplyData(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    //set self test off
    WriteAccSingleData(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadAccSingleData(BMI088_ACC_SELF_TEST, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
    {
        return BMI088_ACC_SELF_TEST_Error;
    }

    //reset the accel sensor
    WriteAccSingleData(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
    {
        return BMI088_SELF_TEST_ACCEL_Error;
    }

    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadAccSingleData(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_Error;
}



/**
 * @brief Init the gyroscope
 * 
 * @note 
 **/
unsigned char BMI088::GyroInit(void)
{
    unsigned char write_reg_num = 0;
    unsigned char res = 0;

    //check commiunication
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    WriteGyroSingleData(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_Sensor;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_Write_GYRO_Reg_Num; write_reg_num++)
    {

        WriteGyroSingleData(write_BMI088_GYRO_Reg_Data_Error[write_reg_num][0], write_BMI088_GYRO_Reg_Data_Error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        ReadGyroSingleData(write_BMI088_GYRO_Reg_Data_Error[write_reg_num][0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_GYRO_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_GYRO_Reg_Data_Error[write_reg_num][2];
        }
    }

    return BMI088_NO_Error;
}



/**
 * @brief gyroscope self test
 * 
 * @note 
 **/
unsigned char BMI088::GyroSelfTest(void)
{
    unsigned char res = 0;
    unsigned char retry = 0;
    //check commiunication is normal
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    WriteGyroSingleData(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ReadGyroSingleData(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    WriteGyroSingleData(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    delay_ms(BMI088_LONG_DELAY_TIME);

    do
    {
        ReadGyroSingleData(BMI088_GYRO_SELF_TEST, &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    }
    while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
    {
        return BMI088_SELF_TEST_GYRO_Error;
    }

    if (res & BMI088_GYRO_BIST_FAIL)
    {
        return BMI088_SELF_TEST_GYRO_Error;
    }

    return BMI088_NO_Error;
}



/**
 * @brief Update the Bmi088 IMU accelerometer data
 * 
 * @note 
 **/
void BMI088::UpdataAccData(void)
{
    uint8_t buff[6] = {0, 0, 0, 0, 0, 0};
    int16_t acc_raw_temp = 0;

    ReadAccMultiplyData(BMI088_ACCEL_XOUT_L, buff, 6);

    acc_raw_temp = (int16_t)(((buff[1]) << 8) | buff[0]);
    m_acc_data.x = acc_raw_temp * BMI088_ACCEL_6G_SEN;
    acc_raw_temp = (int16_t)(((buff[3]) << 8) | buff[2]);
    m_acc_data.y = acc_raw_temp * BMI088_ACCEL_6G_SEN;
    acc_raw_temp = (int16_t)(((buff[5]) << 8) | buff[4]);
    m_acc_data.z = acc_raw_temp * BMI088_ACCEL_6G_SEN;
}



/**
 * @brief Update the Bmi088 IMU gyro data
 * 
 * @note 
 **/
void BMI088::UpdataGyroData(void)
{
    uint8_t buff[6] = {0, 0, 0, 0, 0, 0};
    int16_t gyro_raw_temp = 0;

    ReadGyroMultiplyData(BMI088_GYRO_X_L, buff, 6);

    gyro_raw_temp = (int16_t)(((buff[1]) << 8) | buff[0]);
    m_gyro_data.x = gyro_raw_temp * BMI088_GYRO_1000_SEN - m_gyro_offset_x;
    gyro_raw_temp = (int16_t)(((buff[3]) << 8) | buff[2]);
    m_gyro_data.y = gyro_raw_temp * BMI088_GYRO_1000_SEN - m_gyro_offset_y;
    gyro_raw_temp = (int16_t)(((buff[5]) << 8) | buff[4]);
    m_gyro_data.z = gyro_raw_temp * BMI088_GYRO_1000_SEN - m_gyro_offset_z;

    m_kalman_filter_gyro_x->UpdateFilter(m_gyro_data.x);
    m_kalman_filter_gyro_z->UpdateFilter(m_gyro_data.z);
    m_kalman_filter_gyro_y->UpdateFilter(m_gyro_data.y);
}



/**
 * @brief Update the Bmi088 IMU angle data
 * 
 * @note 
 **/
void BMI088::UpdataAngleData(void)
{
    float acc[3] = {m_acc_data.x, m_acc_data.y, m_acc_data.z};
    float gyro[3] = {m_gyro_data.x, m_gyro_data.y, m_gyro_data.z};
    m_mahony_filter->AngleUpdate(acc, gyro);
}



/**
 * @brief Read the acceleration data from specified register
 * 
 * @note 
 **/
void BMI088::ReadAccSingleData(unsigned char reg_id, unsigned char *pData)
{
    ACC_ENABLE;
    WriteReadByte((reg_id) | 0x80); 
    WriteReadByte(0x55);          
    *pData = WriteReadByte(0x55); 
    ACC_DISABLE;
}



/**
 * @brief Read the acceleration multiply data from specified register
 * 
 * @note 
 **/
void BMI088::ReadAccMultiplyData(unsigned char reg_id, unsigned char *pData, unsigned char size)
{
    ACC_ENABLE;
    WriteReadByte((reg_id) | 0x80);      \
    ReadMultiplyReg(reg_id, pData, size);
    ACC_DISABLE;
}



/**
 * @brief Read the gyroscope data from specified register
 * 
 * @note 
 **/
void BMI088::ReadGyroSingleData(unsigned char reg_id, unsigned char *pData)
{
    GYRO_ENABLE;
    ReadSingleReg(reg_id, pData);
    GYRO_DISABLE;
}



/**
 * @brief Read the gyroscope multiply data from specified register
 * 
 * @note 
 **/
void BMI088::ReadGyroMultiplyData(unsigned char reg_id, unsigned char *pData, unsigned char size)
{
    GYRO_ENABLE;
    ReadMultiplyReg(reg_id, pData, size);
    GYRO_DISABLE;
}



/**
 * @brief write the acceleration data to specified address
 * 
 * @note 
 **/
void BMI088::WriteAccSingleData(unsigned char reg_id, unsigned char data)
{
    ACC_ENABLE;
    WriteSingleReg(reg_id, data);
    ACC_DISABLE;
}



/**
 * @brief write the gyroscope data to specified register
 * 
 * @note 
 **/
void BMI088::WriteGyroSingleData(unsigned char reg_id, unsigned char data)
{
    GYRO_ENABLE;
    WriteSingleReg(reg_id, data);
    GYRO_DISABLE;
}



/**
 * @brief write and read register
 * 
 * @note  use spi to send a byte and receive a received byte
 **/
unsigned char BMI088::WriteReadByte(unsigned char txdata)
{
    while((SPI1->SR&SPI_SR_TXE)==0);    //等待发送结束
    SPI1->DR = txdata;
    while((SPI1->SR&SPI_SR_RXNE)==0);   //等待接收结束
    return SPI1->DR;
}



/**
 * @brief read single register
 * 
 * @note 
 **/
void BMI088::ReadSingleReg(unsigned char reg_id, unsigned char* pData)
{
    WriteReadByte(reg_id | 0x80);
    *pData = WriteReadByte(0x55);
}



/**
 * @brief write single register
 * 
 * @note 
 **/
void BMI088::WriteSingleReg(unsigned char reg_id, unsigned char data)
{
    WriteReadByte(reg_id);
    WriteReadByte(data);
}



/**
 * @brief read multiply register
 * 
 * @note 
 **/
void BMI088::ReadMultiplyReg(unsigned char reg_id, unsigned char* pData, unsigned char size)
{
    WriteReadByte(reg_id | 0x80);

    while (size != 0)
    {

        *pData = WriteReadByte(0x55);
        pData++;
        size--;
    }
}