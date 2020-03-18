#include "imu.h"
#include "spi.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "delay.h"
#include "usart3.h"
#include "timer.h"
#include <math.h>
#include <string.h>
#include "adc.h"
#include "main.h"
#include "pid_regulator.h"
#include "stdlib.h"

#define MPU_ID  0x70

float imu_temp = 0;
float imu_yaw_angle=0;
float imu_yaw_angular_speed=0;
float imu_pitch_angular_speed=0;
//---------------------------------------------------------------------------------------------------
// Variable definitions
static volatile float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
static volatile float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
volatile float        q0 = 1.0f;
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;

static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  
uint8_t mpu_buff[14];  /* buffer to save imu raw data */
uint8_t ist_buff[6];   /* buffer to save IST8310 raw data */

static imu_t imu = {
                    {0,0,0},                     //atti
                    {0,0,0,0,0,0,0,0,0,0},       //rip
                    {0,0,0,0,0,0,0,0,0,0},       //raw
                    {ACC_X_OFFSET,ACC_Y_OFFSET,ACC_Z_OFFSET,GYRO_X_OFFSET,GYRO_Y_OFFSET,GYRO_Z_OFFSET,MAG_X_OFFSET,MAG_Y_OFFSET,MAG_Z_OFFSET},         //offset
                    };

volatile float IST8310_FIFO[3][6] = {0};    //[0]-[4]为最近5次数据 [5]为5次数据的平均值 
                                     //注：磁传感器的采样频率慢，所以单独列出
PID_Regulator_t IMUTemperaturePID = IMU_Temperature_PID_DEFAULT;

void init_quaternion(float hx, float hy);

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data){
    static uint8_t MPU_Tx;
    MPU6500_NSS_Low();
    MPU_Tx = reg&0x7f;
    SPI_ReadWriteByte(MPU_Tx);
    MPU_Tx = data;
    SPI_ReadWriteByte(MPU_Tx);
    MPU6500_NSS_High();
    return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg){
  static uint8_t MPU_Rx, MPU_Tx; 
  MPU6500_NSS_Low();
  MPU_Tx = reg|0x80;
  SPI_ReadWriteByte(MPU_Tx);
    MPU_Rx = SPI_ReadWriteByte(0x00);//0xff
  MPU6500_NSS_High();
  return MPU_Rx;
}
//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const reg, uint8_t *pData, uint8_t len){
    static uint8_t  MPU_Tx;//MPU_Rx;//
    MPU6500_NSS_Low();
    MPU_Tx = reg|0x80;
    SPI_ReadWriteByte(MPU_Tx);
    for(int i = 0;i<len;i++)
    {
        pData[i] = SPI_ReadWriteByte(0x00);//0xff
    }
    MPU6500_NSS_High();
    return 0;
}

uint8_t MPU6500_Read_Regss(uint8_t const addr,uint8_t reg,uint8_t len,uint8_t *buf){
    MPU6500_Read_Regs(reg, buf, len);
    return 0;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data){
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(10);
}
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr){
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_ms(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  return data;
}

static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num){
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  delay_ms(2);
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr){
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr){
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}
/**
  * @brief          初始化mpu6500
  * @author         金双平
  * @param[in]      NULL
  * @retval         返回空
  */
uint8_t MPU6500_Init(void){
    MPU6500_NSS_High();
    
    delay_ms(100);
    
    if(MPU_ID  ==  MPU6500_Read_Reg(MPU6500_WHO_AM_I)){
    }
    else{ 
        return 1; //error
    }
    
     uint8_t index = 0;
     uint8_t MPU6500_Init_Data[10][2] = {
        {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
        {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
        {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
        {MPU6500_CONFIG,        0x04},      // LPF 98Hz
        {MPU6500_GYRO_CONFIG,   0x10},      // +-1000dps
        {MPU6500_ACCEL_CONFIG,  0x00},      // +-2G
        {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
        {MPU6500_USER_CTRL,     0x20},      // Enable AUX
    };
  
    for(index = 0; index < 10; index++){
        MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
        delay_ms(1);
    }
    
    MPU6500_Set_Gyro_Fsr(2);
    MPU6500_Set_Accel_Fsr(0);

    return 0;
}
/**
  * @brief          初始化ist8310
  * @author         金双平
  * @param[in]      NULL
  * @retval         返回空
  */
uint8_t IST8310_Init(void){
    MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
    delay_ms(10);
    MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
    delay_ms(10);

    MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    delay_ms(10);
    MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    delay_ms(10);

    IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
    delay_ms(10);
    if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error

    IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01); 
    delay_ms(10);

    IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
    if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) !=   0x00)
    return 2;
    delay_ms(10);

    IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
    if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) !=   0x00)
    return 3;
    delay_ms(10);

    IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
    if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) !=   0x24)
    return 4;
    delay_ms(10);

    IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
    if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) !=   0xc0)
    return 5;
    delay_ms(10);

    MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
    delay_ms(10);
    MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
    delay_ms(10);

    MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    delay_ms(100);
    return 0;
}
/**
  * @brief          读取ist8310的磁力值
  * @author         金双平
  * @param[in]      存储ist8310寄存器值的数组起始地址
  * @retval         返回空
  */
void GetIST8310_RawValues(uint8_t* buff)
{
    MPU6500_Read_Regs(MPU6500_EXT_SENS_DATA_00,buff,6);
}
/**
  * @brief          磁力值滑动平均滤波算法
  * @author         李运环
  * @param[in]      转换为国际单位的磁力值的结构体指针
  * @retval         返回空
  */
static void ist8310_moving_average_filter(imu_ripdata_t * mpudata){
    static uint8_t count = 0;

    for(uint8_t i = 1;i<5;i++){
        IST8310_FIFO[0][i-1] = IST8310_FIFO[0][i];
        IST8310_FIFO[1][i-1] = IST8310_FIFO[1][i];
        IST8310_FIFO[2][i-1] = IST8310_FIFO[2][i];
    }
    IST8310_FIFO[0][4] = mpudata->mx;
    IST8310_FIFO[1][4] = mpudata->my;
    IST8310_FIFO[2][4] = mpudata->mz;

    if(count >= 4){
        for(uint8_t j = 0;j<3;j++){
            for(uint8_t i = 0;i<5;i++){
                IST8310_FIFO[j][5] += IST8310_FIFO[j][i];
            }
            IST8310_FIFO[j][5] = IST8310_FIFO[j][5]/5;
        }
        mpudata->mx = IST8310_FIFO[0][5];
        mpudata->my = IST8310_FIFO[1][5];
        mpudata->mz = IST8310_FIFO[2][5];
    }
    else{
        count++;
    }
}

/**
  * @brief          mpu和ist数据减去零漂，并转换为国际单位, 磁力值滑动平均滤波
  * @author         李运环
  * @param[in]      NULL
  * @retval         返回空
  */
static void imu_calibrate_unit_convert(void){
//unit:m/s2
    imu.rip.ax = (float)((imu.raw.ax - imu.offset.ax) * 9.80665f / 16384.0f);
    imu.rip.ay = (float)((imu.raw.ay - imu.offset.ay) * 9.80665f / 16384.0f);
    imu.rip.az = (float)((imu.raw.az - imu.offset.az) * 9.80665f / 16384.0f);

/* +-1000dps -> rad/s */
    imu.rip.gx = (float)((imu.raw.gx - imu.offset.gx) /32.8f /57.3f);
    imu.rip.gy = (float)((imu.raw.gy - imu.offset.gy) /32.8f /57.3f);
    imu.rip.gz = (float)((imu.raw.gz - imu.offset.gz) /32.8f /57.3f);

/* uT */
    imu.rip.mx = (float)((imu.raw.mx - imu.offset.mx) * MAG_SEN);
    imu.rip.my = (float)((imu.raw.my - imu.offset.my) * MAG_SEN);
    imu.rip.mz = (float)((imu.raw.mz - imu.offset.mz) * MAG_SEN);

    ist8310_moving_average_filter(&imu.rip);
//    gyro_moving_average_filter(&imu.rip);
/* 摄氏度 */
    imu.rip.temp = imu.raw.temp * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;
}
/**
  * @brief          加速度计低通滤波
  * @author         李运环
  * @param[in]      转化为国际单位的加速度值的结构体指针
  * @retval         返回空
  */
static void accel_low_pass_filter(imu_ripdata_t * mpudata){
    static uint8_t updata_count=0;

    static double accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
    static double accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
    static double accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
    static const double fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

    if(updata_count==0){
        accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = mpudata->ax;
        accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = mpudata->ay;
        accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = mpudata->az;
        updata_count++;
    }
    else{
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + mpudata->ax * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + mpudata->ay * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + mpudata->az * fliter_num[2];
    }
    
    mpudata->ax = accel_fliter_3[0];
    mpudata->ay = accel_fliter_3[1];
    mpudata->az = accel_fliter_3[2];
}
/**
  * @brief          更新加速度、角速度、磁力值
  * @author         李运环
  * @retval         返回空
  */
static void imu_get_data(void)
{
    static uint8_t init_quaternion_flag=0;
    
    MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    imu.raw.ax = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]);
    imu.raw.ay = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]);
    imu.raw.az = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]);

    imu.raw.temp = (int16_t)(mpu_buff[6] << 8 | mpu_buff[7]);

    imu.raw.gx = (int16_t)(mpu_buff[8]  << 8 | mpu_buff[9]);
    imu.raw.gy = (int16_t)(mpu_buff[10] << 8 | mpu_buff[11]);
    imu.raw.gz = (int16_t)(mpu_buff[12] << 8 | mpu_buff[13]);

    GetIST8310_RawValues(ist_buff);
    memcpy(&imu.raw.mx, ist_buff, 6);

    imu_calibrate_unit_convert();

    accel_low_pass_filter(&imu.rip);

    if(init_quaternion_flag==0){
        init_quaternion(imu.rip.mx, imu.rip.my);
        init_quaternion_flag=1;
    }
}
/**
  * @brief          获取加速度、角速度的零漂
  * @author         李运环
  * @retval         返回空
  */
void get_mpu_accel_gyro_offset(void){
    for (int i = 0; i<300;i++){
        MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

        imu.offset.ax +=  mpu_buff[0] << 8 | mpu_buff[1];
        imu.offset.ay +=  mpu_buff[2] << 8 | mpu_buff[3];
        imu.offset.az +=  mpu_buff[4] << 8 | mpu_buff[5];

        imu.offset.gx +=  mpu_buff[8]  << 8 | mpu_buff[9];
        imu.offset.gy +=  mpu_buff[10] << 8 | mpu_buff[11];
        imu.offset.gz +=  mpu_buff[12] << 8 | mpu_buff[13];

        delay_ms(2);
    }
    imu.offset.ax = imu.offset.ax / 300;
    imu.offset.ay = imu.offset.ay / 300;
    imu.offset.az = imu.offset.az / 300;
    imu.offset.gx = imu.offset.gx / 300;
    imu.offset.gy = imu.offset.gy / 300;
    imu.offset.gz = imu.offset.gz / 300;
}

void get_mpu_accel_gyro_offset_test(void){
    for (int i = 0; i<300;i++){
        MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

        imu.offset.ax +=  mpu_buff[0] << 8 | mpu_buff[1];
        imu.offset.ay +=  mpu_buff[2] << 8 | mpu_buff[3];
        imu.offset.az +=  mpu_buff[4] << 8 | mpu_buff[5];

        imu.offset.gx +=  mpu_buff[8]  << 8 | mpu_buff[9];
        imu.offset.gy +=  mpu_buff[10] << 8 | mpu_buff[11];
        imu.offset.gz +=  mpu_buff[12] << 8 | mpu_buff[13];

        delay_ms(2);
    }
    imu.offset.ax = imu.offset.ax / 300;
    imu.offset.ay = imu.offset.ay / 300;
    imu.offset.az = imu.offset.az / 300;
    imu.offset.gx = imu.offset.gx / 300;
    imu.offset.gy = imu.offset.gy / 300;
    imu.offset.gz = imu.offset.gz / 300;
    printf("%d,%d,%d,%d,%d,%d\r\n",imu.offset.ax,imu.offset.ay,imu.offset.az,imu.offset.gx,imu.offset.ay,imu.offset.gz);
}
/**
  * @brief          获取磁力计的偏移（不理解，不知道能否使用，本程序未使用）
  * @author         DJI_RM
  * @retval         返回空
  */
void get_ist_mag_offset(void){
    int16_t mag_max[3], mag_min[3];
    for (int i = 0; i < 1000; i++){
        GetIST8310_RawValues(ist_buff);
        memcpy(&imu.raw.mx, ist_buff, 6);
        if ((abs(imu.raw.mx) < 400) && (abs(imu.raw.my) < 400) && (abs(imu.raw.mz) < 400)){
            mag_max[0] = VAL_MAX(mag_max[0], imu.raw.mx);
            mag_min[0] = VAL_MIN(mag_min[0], imu.raw.mx);

            mag_max[1] = VAL_MAX(mag_max[1], imu.raw.my);
            mag_min[1] = VAL_MIN(mag_min[1], imu.raw.my);

            mag_max[2] = VAL_MAX(mag_max[2], imu.raw.mz);
            mag_min[2] = VAL_MIN(mag_min[2], imu.raw.mz);
        }
    delay_ms(2);
    }
    imu.offset.mx = (int16_t)((mag_max[0] + mag_min[0]) * 0.5f);
    imu.offset.my = (int16_t)((mag_max[1] + mag_min[1]) * 0.5f);
    imu.offset.mz = (int16_t)((mag_max[2] + mag_min[2]) * 0.5f);
}
/**
  * @brief          快速计算方根
  * @author         open source
  * @param[in]      需要计算方根的单精度浮点数
  * @retval         返回空
  */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/**
  * @brief          姿态解算，更新四元数
  * @author         http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
  * @param[in]      转换成国际单位的加速度、角速度的结构体指针
  * @retval         返回空
  */
static void mahony_ahrs_updateIMU(imu_ripdata_t const *mpudata){
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx = mpudata->gx;
    gy = mpudata->gy;
    gz = mpudata->gz;
    ax = mpudata->ax;
    ay = mpudata->ay;
    az = mpudata->az;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f){
          integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
          integralFBy += twoKi * halfey * (1.0f / sampleFreq);
          integralFBz += twoKi * halfez * (1.0f / sampleFreq);
          gx += integralFBx; // apply integral feedback
          gy += integralFBy;
          gz += integralFBz;
        }
        else{
          integralFBx = 0.0f; // prevent integral windup
          integralFBy = 0.0f;
          integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
/**
  * @brief          姿态解算, 更新四元数, this function takes 56.8us.(168M)
  * @author         http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
  * @param[in]      转换成国际单位的加速度、角速度、磁力值的结构体指针
  * @retval         返回空
  */
static void mahony_ahrs_update(imu_ripdata_t const *mpudata){
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx = mpudata->gx;
    gy = mpudata->gy;
    gz = mpudata->gz;
    ax = mpudata->ax;
    ay = mpudata->ay;
    az = mpudata->az;
    mx = mpudata->mx;
    my = mpudata->my;
    mz = mpudata->mz;

#if AXIS_6 == 1
    mx = 0;
    my = 0;
    mz = 0;
#endif

    if((mx == 0) && (my == 0) && (mz == 0)){
        mahony_ahrs_updateIMU(mpudata);
        return;
    }
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
/**
  * @brief          根据磁力计的值初始化四元数
  * @author         DJI_RM
  * @param[in]      读取磁力计的磁力值的结构体
  * @retval         返回空
  */
void init_quaternion(float hx, float hy){

#if BOARD_DOWN
    if (hx < 0 && hy < 0) 
    {
        if (fabs(hx / hy) >=  1)
        {
            q0 = -0.005;
            q1 = -0.199;
            q2 = 0.979;
            q3 = -0.0089;
        }
        else
        {
            q0 = -0.008;
            q1 = -0.555;
            q2 = 0.83;
            q3 = -0.002;
        }
        
    }
    else if (hx < 0 && hy > 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.005;
            q1 = -0.199;
            q2 = -0.978;
            q3 = 0.012;
        }
        else
        {
            q0 = 0.005;
            q1 = -0.553;
            q2 = -0.83;
            q3 = -0.0023;
        }
        
    }
    else if (hx > 0 && hy > 0)
    {
        if (fabs(hx / hy) >=  1)
        {
            q0 = 0.0012;
            q1 = -0.978;
            q2 = -0.199;
            q3 = -0.005;
        }
        else
        {
            q0 = 0.0023;
            q1 = -0.83;
            q2 = -0.553;
            q3 = 0.0023;
        }
        
    }
    else if (hx > 0 && hy < 0)
    {
        if (fabs(hx / hy) >=  1)
        {
            q0 = 0.0025;
            q1 = 0.978;
            q2 = -0.199;
            q3 = 0.008;            
        }
        else
        {
            q0 = 0.0025;
            q1 = 0.83;
            q2 = -0.56;
            q3 = 0.0045;
        }        
    }
#else
        if (hx < 0 && hy < 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.195;
            q1 = -0.015;
            q2 = 0.0043;
            q3 = 0.979;
        }
        else
        {
            q0 = 0.555;
            q1 = -0.015;
            q2 = 0.006;
            q3 = 0.829;
        }
        
    }
    else if (hx < 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.193;
            q1 = -0.009;
            q2 = -0.006;
            q3 = 0.979;
        }
        else
        {
            q0 = -0.552;
            q1 = -0.0048;
            q2 = -0.0115;
            q3 = 0.8313;
        }
        
    }
    else if (hx > 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.9785;
            q1 = 0.008;
            q2 = -0.02;
            q3 = 0.195;
        }
        else
        {
            q0 = -0.9828;
            q1 = 0.002;
            q2 = -0.0167;
            q3 = 0.5557;
        }
        
    }
    else if (hx > 0 && hy < 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.979;
            q1 = 0.0116;
            q2 = -0.0167;
            q3 = -0.195;            
        }
        else
        {
            q0 = -0.83;
            q1 = 0.014;
            q2 = -0.012;
            q3 = -0.556;
        }        
    }
#endif
}
/**
  * @brief          根据四元数更新姿态角，并对yaw进行连续化处理
  * @author         李运环
  * @param[in]      NULL
  * @retval         返回空
  */

static void IMU_getYawPitchRoll(attitude_angle_t * atti)
{
    static float yaw_temp = 0,last_yaw_temp = 0;
    static int   yaw_count = 0;
    // yaw    -pi----pi
    atti->yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)* 57.3;
    // pitch  -pi/2--- pi/2
    atti->pit = -asin(2.0f * (q1 * q3 - q0 * q2))* 57.3;
    // roll   -pi-----pi
    atti->rol = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)* 57.3;

    //yaw数据连续化
    last_yaw_temp = yaw_temp;
    yaw_temp = atti->yaw; 
    if(yaw_temp - last_yaw_temp>= 300){
        yaw_count--;
    } 
    else if (yaw_temp - last_yaw_temp <= -300){
        yaw_count++;
    }
    atti->yaw = yaw_temp + yaw_count*360;

}
/**
  * @brief          获取IMU控制温度，为开机时adc读取的板子温度+10
  * @author         李运环
  * @retval         返回空
  */
//static int8_t IMU_GET_CONTROL_TEMPERATURE(void){
//    static int8_t control_temperature = 0;
//    static uint8_t count=0;
//    if(count==0){
//        control_temperature = (int8_t)(get_temprate()) + 10;
//        if (control_temperature > (int8_t)(GYRO_CONST_MAX_TEMP)){
//            control_temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
//        }
//        count=1;
//    }
////    printf("control_temperature:%5d，imu_temperature:%5.3f\r\n", control_temperature,imu.atti.temp);
//    printf("%8.3lf,%5d，%5.3f\r\n",imu.atti.yaw, control_temperature,imu.rip.temp);
//    delay_ms(5);
//    return control_temperature;
//}
/**
  * @brief          IMU温度闭环控制
  * @author         李运环
   *@param[in]      读取的mpu6500的温度值，单位：摄氏度
  * @retval         返回空
  */
void IMU_temp_Control(double temp){
    uint16_t tempPWM;
    static uint8_t first_temperature = 0;
    static uint8_t temp_constant_time = 0 ;
    if (first_temperature){
        IMUTemperaturePID.ref = IMU_CONTROL_TEMPERATURE;
        IMUTemperaturePID.fdb = temp;
        IMUTemperaturePID.Calc(&IMUTemperaturePID);
        
        if (IMUTemperaturePID.output < 0.0f){
            IMUTemperaturePID.output = 0.0f;
        }
        tempPWM = (uint16_t)IMUTemperaturePID.output;
        TIM_SetCompare2(TIM3, tempPWM);
    }
    else{
        //在没有达到设置的温度，一直最大功率加热
        if (temp > IMU_CONTROL_TEMPERATURE){
            temp_constant_time ++;
            if(temp_constant_time > 50){
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperature = 1;
                //imuTempPid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        TIM_SetCompare2(TIM3, MPU6500_TEMP_PWM_MAX - 1);
    }
}
/**
  * @brief          mpu6500和ist8310的初始化以及角速度、加速度、磁力值的零漂，通过宏定义决定是否开启IMU温控
  * @author         李运环
  * @retval         返回空
  */
void imu_init(void){
    while(MPU6500_Init()){
        printf("MPU6500 init error！\r\n");//失败进入死循环
        delay_ms(500);
    }
    printf("MPU6500 init done! \r\n");
    delay_ms(5);
    IST8310_Init();
    delay_ms(5);
//    get_mpu_accel_gyro_offset();
    delay_ms(5);
    get_ist_mag_offset();
#if IMU_TEMPERATURE_CONTROL == 1
//    temperature_ADC_init();
    TIM3_Init(MPU6500_TEMP_PWM_MAX, 1); //陀螺仪温度控制PWM初始化
#endif
}
/**
  * @brief          IMU主函数
  * @author         李运环
  * @retval         返回空
  */
void imu_main(void){
    imu_get_data();
  	imu_temp = imu.rip.temp;
//    get_mpu_accel_gyro_offset_test();
    mahony_ahrs_update(&imu.rip);
    IMU_getYawPitchRoll(&imu.atti);
    imu_yaw_angle=imu.atti.yaw;
    imu_yaw_angular_speed=imu.rip.gz*57.3f;
    imu_pitch_angular_speed=imu.rip.gy*57.3f;
#if IMU_TEMPERATURE_CONTROL == 1
//    IMU_temp_Control(imu.rip.temp);
//	printf("temp:%f\r\n",imu.rip.temp);
#endif
#if Monitor_IMU_Angle == 1
    printf("yaw_angle:%8.3lf   pit_angle:%8.3lf  rol_angle:%8.3lf\r\n", imu.atti.yaw, imu.atti.pit, imu.atti.rol);
    delay_ms(5);
#endif
#if Monitor_IMU_Gyro == 1
    printf("wx:%8.3lf  wy:%8.3lf  wz:%8.3lf\r\n", imu.rip.gx * 57.3f, imu.rip.gy * 57.3f, imu.rip.gz * 57.3f);
    delay_ms(5);
#endif
#if Monitor_IMU_Mag == 1
    printf("%.3f,%.3f,%.3f\r\n",(float)imu.raw.mx/1000,(float)imu.raw.my/1000,(float)imu.raw.mz/1000);
    delay_ms(5);
#endif
}

float get_yaw_angle(void){
    return imu.atti.yaw;
}

float get_pit_angle(void){
    return imu.atti.pit;
}

float get_imu_wx(void){
    return imu.rip.gx * 57.3f;
}

float get_imu_wy(void){
    return imu.rip.gy * 57.3f;
}

float get_imu_wz(void){
    return imu.rip.gz * 57.3f;
}
