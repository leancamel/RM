#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_RIGHT_MOTOR_ID = 0x205,
    CAN_LEFT_MOTOR_ID = 0x206,
    CAN_WHEEL_ALL_ID = 0x1FF,
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;           //转子机械角度
    int16_t speed_rpm;      //转子转速
    int16_t given_current;  //实际转矩电流
    uint8_t temperate;      //电机温度
    int16_t last_ecd;       //上次转子机械角度
} motor_measure_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_WHEEL(int16_t right, int16_t left);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//返回右驱动轮电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void);
//返回左驱动轮电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void);
//返回关节电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_solve(void);
#endif

#endif
