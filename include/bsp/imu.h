#ifndef _IMU_H_
#define _IMU_H_
#include "sys.h"

/* Adjust the Kp and Ki of IMU module
   This can change the convergence speed of the IMU output */
#define BOARD_DOWN                  1
#define sampleFreq                  200.0f                  // sample frequency in Hz
#define twoKpDef                    (40.0f * 0.5f)          // 2 * proportional gain
#define twoKiDef                    (2.0f * 0.005f)         // 2 * integral gain
#define AXIS_6                      1                       // 0--9axis   1--6axis
#define IMU_TEMPERATURE_CONTROL     1                       //imu温控，可能导致漂移，需测试后使用
#define IMU_CONTROL_TEMPERATURE     40                      //imu控制温度

#define ACC_X_OFFSET    -65   //
#define ACC_Y_OFFSET    41
#define ACC_Z_OFFSET    93

#define GYRO_X_OFFSET    -42     //
#define GYRO_Y_OFFSET    41
#define GYRO_Z_OFFSET    -9


#define MAG_SEN         0.1f    //转换成 uT
#define MAG_X_OFFSET    -10     //磁场偏移，采样空间磁场后matlab计算得到
#define MAG_Y_OFFSET    1
#define MAG_Z_OFFSET    12

#define MPU6500_NSS_Low() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_RESET)
#define MPU6500_NSS_High() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET)

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1
#define GYRO_CONST_MAX_TEMP 45 //陀螺仪控制恒温 最大控制温度

#define MPU6500_TEMPERATURE_FACTOR 0.002f
#define MPU6500_TEMPERATURE_OFFSET 23.0f

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //温度控制PID的max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //温度控制PID的max_iout

#define IMU_Temperature_PID_DEFAULT \
{\
    0,\
    0,\
    {0,0},\
    MPU6500_TEMPERATURE_PID_KP,\
    MPU6500_TEMPERATURE_PID_KI,\
    MPU6500_TEMPERATURE_PID_KD,\
    0,\
    0,\
    0,\
    0,\
    0,\
    0,\
    0,\
    MPU6500_TEMPERATURE_PID_MAX_OUT,\
    0,\
    0,\
    0,\
    &PID_Calc,\
    &PID_Reset,\
}

typedef struct{
    float ax; //unit: m/s2
    float ay;
    float az;
    
    float gx; //rad/s
    float gy;
    float gz;
    
    float mx; //unit: uT
    float my;
    float mz;

    float temp;
}imu_ripdata_t;  //处理过的数据

typedef struct{
    float rol;
    float pit;
    float yaw;
}attitude_angle_t;  //姿态角

typedef struct{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
    
    int16_t mx;
    int16_t my;
    int16_t mz;
    
}imu_offset_t;      //偏移

typedef struct{
    int16_t ax;
    int16_t ay;
    int16_t az;
 
    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;
    
    int16_t mx;     //unit: mG
    int16_t my;
    int16_t mz;
}imu_rawdata_t;     //原始数据

typedef struct{
    attitude_angle_t atti;
    imu_ripdata_t rip;
    imu_rawdata_t raw;
    imu_offset_t  offset;
}imu_t;

extern float imu_temp;

extern void imu_init(void);              //imu初始化
extern void imu_main(void);              //imu获取姿态角主函数
void IMU_temp_Control(double temp);

extern float imu_yaw_angle;              //yaw角度 °
extern float imu_yaw_angular_speed;      //yaw角速度 °/s
extern float imu_pitch_angular_speed;    //pitch角速度 °/s
#endif
