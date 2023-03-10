/**
  ****************************RM Warrior 2023****************************
  * @file       start_task.c/h
  * @brief      一个普通的心跳程序，如果程序没有问题，蓝灯以1Hz的频率闪烁
  *             同时也用来发送相关数值到上位机调参。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/2/         pxx              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#include "User_Task.h"
#include "main.h"
#include "stdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "adc.h"

//#include "Detect_Task.h"
#include "INS_Task.h"
#include "gimbal_task.h"
#include "chassis_remote_control.h"
#include "ROS_Receive.h"
//#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

//姿态角 单位 度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};
const Gimbal_Control_t* local_gimbal_control;
const chassis_move_t* local_chassis_move;

extern int8_t temp_set;

void UserTask(void *pvParameters)
{
    static uint8_t Tcount = 0;
    //获取姿态角指针
    const volatile fp32 *angle;
    angle = get_INS_angle_point();
    //获取云台控制结构体指针
    local_gimbal_control = get_gimbal_control_point();
    //获取底盘控制结构体指针
    local_chassis_move = get_chassis_control_point();
    while (1)
    {
        Tcount++;
        if(Tcount >= 50)
        {
            led_blue_toggle();
            Tcount = 0;
        }
        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
        // angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        // angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        // angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;

        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET));
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET));
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET));


        //姿态角
        // printf("%.2f, %.2f, %.2f\n", angle_degree[0], angle_degree[1], angle_degree[2]);

        //云台yaw电机角度环串速度环pid调参
        // printf("%.2f, %.2f, %.2f, %.2f\n", 
        // local_gimbal_control->gimbal_yaw_motor.absolute_angle * 57.3f, local_gimbal_control->gimbal_yaw_motor.absolute_angle_set * 57.3f,
        // local_gimbal_control->gimbal_yaw_motor.motor_gyro * 10, local_gimbal_control->gimbal_yaw_motor.motor_gyro_set * 10);

        //云台pitch电机pid调参
        // printf("%.2f, %.2f, %.2f, %.2f\n", 
        // local_gimbal_control->gimbal_pitch_motor.relative_angle * 57.3f, local_gimbal_control->gimbal_pitch_motor.relative_angle_set * 57.3f,
        // local_gimbal_control->gimbal_pitch_motor.motor_gyro * 10, local_gimbal_control->gimbal_pitch_motor.motor_gyro_set * 10);

        //底盘跟随云台角度pid调参
        // printf("%.2f, %.2f\n", local_chassis_move->chassis_relative_angle * 57.3f, local_chassis_move->chassis_relative_angle_set * 57.3f);
        
        //imu 温度控制PID
        init_vrefint_reciprocal();
        // printf("%.2f, %d\n", get_temprate(), temp_set);

        // printf("%f\n",ROS_Response.wz);
        vTaskDelay(10);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
