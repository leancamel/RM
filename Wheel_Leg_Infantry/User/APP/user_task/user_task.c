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
#include "buzzer.h"
#include "uart1.h"

#include "INS_Task.h"
#include "chassis_task.h"
#include "remote_control.h"
#include "detect_task.h"

#include "voltage_task.h"
#include "Kalman_Filter.h"
#include "bluetooth.h"
// #define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

//归一化函数
fp32 normalize_fp32(fp32 data);
void buzzer_warn(uint8_t num, uint8_t interval);

//姿态角 单位 度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};
const chassis_move_t* local_chassis_move;
const fp32 *local_imu_gyro;	  // 获取角加速度指针

void UserTask(void *pvParameters)
{
    vTaskDelay(20);
    static uint8_t Tcount = 0;
    //获取姿态角指针
    const volatile fp32 *angle;
    angle = get_INS_angle_point();
    local_chassis_move = get_chassis_control_point();
    local_imu_gyro = get_gyro_data_point();

    while (1)
    {
        Tcount++;
        if(Tcount >= 100)
        {
            led_blue_toggle();
            Tcount = 0;
        }

#if USE_BUZZER_WARNING == 1
        if(toe_is_error(WHEEL_MOTOR5_TOE) || toe_is_error(WHEEL_MOTOR6_TOE || toe_is_error(CHASSIS_MOTOR1_TOE))
            || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE))
            buzzer_warn(MOTOR_LOST + 1, 10);
        else if(local_chassis_move->left_leg.leg_length > LEG_LENGTH_MAX || local_chassis_move->right_leg.leg_length > LEG_LENGTH_MAX)
            buzzer_warn(LEG_EXCEED + 1, 10);
        else
            buzzer_off();
#endif

        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;

        // angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET));
        // angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET));
        // angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET));

        //姿态角
        // printf("%f, %f, %f\n", angle_degree[0], angle_degree[1], angle_degree[2]);
        // printf("%f, %f, %f\n", *(local_chassis_move->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET),
        //                         *(local_chassis_move->chassis_imu_accel + INS_ACCEL_Y_ADDRESS_OFFSET),
        //                         *(local_chassis_move->chassis_imu_accel + INS_ACCEL_Z_ADDRESS_OFFSET));

        // printf("%d, %d, %d, %d\n", local_chassis_move->left_leg.front_joint.joint_motor_measure->ecd, 
        //                             local_chassis_move->left_leg.back_joint.joint_motor_measure->ecd,
        //                             local_chassis_move->right_leg.front_joint.joint_motor_measure->ecd,
        //                             local_chassis_move->right_leg.back_joint.joint_motor_measure->ecd);

        // printf("%f, %f\n", local_chassis_move->state_ref.theta * 57.3f, local_chassis_move->state_ref.phi * 57.3f);

        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->left_leg.front_joint.angle * 57.3f, 
        //                             local_chassis_move->left_leg.back_joint.angle * 57.3f,
        //                             local_chassis_move->right_leg.front_joint.angle * 57.3f,
        //                             local_chassis_move->right_leg.back_joint.angle * 57.3f);
        
        // 地面支持力
        // printf("%f, %f, %f\n", local_chassis_move->ground_force, local_chassis_move->state_ref.x, local_chassis_move->leg_length*100);
        // printf("%f, %f, %f\n", local_chassis_move->left_leg.leg_length_set, local_chassis_move->right_leg.leg_length_set, local_chassis_move->chassis_roll);

        // printf("%f, %f, %f, %f, %f\n", local_chassis_move->wheel_tor, local_chassis_move->leg_tor,
        //         local_chassis_move->state_ref.theta, local_chassis_move->state_ref.x, local_chassis_move->state_ref.phi);

        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->right_leg.leg_length * 100, local_chassis_move->right_leg.leg_angle * 57.3f,
        //         local_chassis_move->right_leg.front_joint.angle * 57.3f, local_chassis_move->right_leg.back_joint.angle * 57.3f);
        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->left_leg.leg_length * 100, local_chassis_move->left_leg.leg_angle * 57.3f,
        //         local_chassis_move->left_leg.front_joint.angle * 57.3f, local_chassis_move->left_leg.back_joint.angle * 57.3f);

        // 腿部速度
        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->left_leg.length_dot * 100.0f, local_chassis_move->left_leg.angle_dot * 57.3f,
        //                         local_chassis_move->right_leg.length_dot * 100.0f, local_chassis_move->right_leg.angle_dot * 57.3f);

        // 腿部长度PID
        // printf("%f, %f\n", local_chassis_move->leg_length, local_chassis_move->leg_angle_dot);
        // printf("%.2f, %.2f, %d, %d\n", local_chassis_move->right_leg.leg_length * 100, local_chassis_move->leg_length_set * 100, 
        //         local_chassis_move->right_leg.back_joint.give_current,
        //         local_chassis_move->right_leg.front_joint.give_current);

        // yaw跟随PID
        // printf("%f, %f, %f, %f\n", local_chassis_move->chassis_yaw_set*57.3f, local_chassis_move->chassis_yaw*57.3f, 
        //         local_chassis_move->wz_set*10.0f, *(local_chassis_move->chassis_imu_gyro+INS_GYRO_Z_ADDRESS_OFFSET)*10.0f);
        // printf("%f, %f\n", local_chassis_move->wz_set*10.0f, *(local_chassis_move->chassis_imu_gyro+INS_GYRO_Z_ADDRESS_OFFSET)*10.0f);

        // printf("%f, %f, %f\n", local_chassis_move->state_ref.x, local_chassis_move->state_ref.x_dot, local_chassis_move->leg_length);
        // printf("%f, %f\n", local_chassis_move->state_set.x_dot, local_chassis_move->state_ref.x_dot);
        // printf("%f, %f\n", local_chassis_move->state_set.x, local_chassis_move->state_ref.x);

        vTaskDelay(10);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif  
    }
}


/**
  * @brief          蜂鸣器报警
  * @param[in]      num:响声次数，最大为5次
  * @param[in]      interval:任务执行间隔ms，最大为200
  * @retval         none
  */
void buzzer_warn(uint8_t num, uint8_t interval)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 0;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 1000/interval;
    }
    else if(show_num == 0)//一次报警周期结束，暂停1s
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 600/interval/num)
        {
            buzzer_off();
        }
        else if(tick < 1000/interval/num)
        {
            buzzer_on(64, 20);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}

/**
  * @brief          归一化函数[0, 1]
  * @param[in]      data 要归一化的数据
  * @retval         归一化后的值
  */
fp32 normalize_fp32(fp32 data) 
{
    // 这里可以根据实际情况进行归一化计算
    // 此处假设数据范围为 [min_value, max_value]
    fp32 min_value = 0.0f;
    fp32 max_value = 1.0f;

    // 归一化计算
    fp32 normalized_data = (data - min_value) / (max_value - min_value);

    return normalized_data;
}