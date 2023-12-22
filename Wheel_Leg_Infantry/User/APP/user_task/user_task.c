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

#include "voltage_task.h"
#include "Kalman_Filter.h"
#include "bluetooth.h"
//#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

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
        if(Tcount >= 50)
        {
            led_blue_toggle();
            Tcount = 0;
        }
        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;

        // angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET));
        // angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET));
        // angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET));

        //姿态角
        // printf("%.2f, %.2f, %.2f\n", angle_degree[0], angle_degree[1], angle_degree[2]);

        // printf("%d, %d, %d, %d\n", local_chassis_move->left_leg.front_joint.joint_motor_measure->ecd, 
        //                             local_chassis_move->left_leg.back_joint.joint_motor_measure->ecd,
        //                             local_chassis_move->right_leg.front_joint.joint_motor_measure->ecd,
        //                             local_chassis_move->right_leg.back_joint.joint_motor_measure->ecd);

        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->left_leg.front_joint.angle * 57.3f, 
        //                             local_chassis_move->left_leg.back_joint.angle * 57.3f,
        //                             local_chassis_move->right_leg.front_joint.angle * 57.3f,
        //                             local_chassis_move->right_leg.back_joint.angle * 57.3f);
        
        // printf("%d, %d\n", local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->ecd, local_chassis_move->right_leg.wheel_motor.wheel_motor_measure->ecd);
        // printf("%.2f, %.2f\n", local_chassis_move->state_ref.x, local_chassis_move->state_ref.x_dot);
        // printf("%f. %f\n", local_chassis_move->left_leg_length_pid.out, local_chassis_move->right_leg_length_pid.out);
        printf("%f, %f, %f\n", local_chassis_move->state_ref.theta * 57.3f, (local_chassis_move->leg_angle - PI/2) * 57.3f, local_chassis_move->chassis_pitch * 57.3f);

        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->right_leg.leg_length * 100, local_chassis_move->right_leg.leg_angle * 57.3f,
        //         local_chassis_move->right_leg.front_joint.angle * 57.3f, local_chassis_move->right_leg.back_joint.angle * 57.3f);
        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->left_leg.leg_length * 100, local_chassis_move->left_leg.leg_angle * 57.3f,
        //         local_chassis_move->left_leg.front_joint.angle * 57.3f, local_chassis_move->left_leg.back_joint.angle * 57.3f);

        // 腿部速度
        // printf("%.2f, %.2f, %.2f, %.2f\n", local_chassis_move->left_leg.length_dot * 100.0f, local_chassis_move->left_leg.angle_dot * 57.3f,
        //                         local_chassis_move->right_leg.length_dot * 100.0f, local_chassis_move->right_leg.angle_dot * 57.3f);

        // 腿部长度PID
        // printf("%.2f, %.2f, %d, %d\n", local_chassis_move->right_leg.leg_length * 100, local_chassis_move->leg_length_set * 100, 
        //         local_chassis_move->right_leg.back_joint.give_current,
        //         local_chassis_move->right_leg.front_joint.give_current);
        
        // 陀螺仪角加速度一阶低通滤波数据
        // printf("%f, %f\n", *(local_imu_gyro+2)*1000, local_chassis_move->state_ref.phi_dot*1000);

        // printf("%.f\n", local_chassis_move->state_ref.theta_dot*100);
        // printf("%f, %f, %f\n", (local_chassis_move->right_leg.leg_angle - local_chassis_move->left_leg.leg_angle) * 57.3f, local_chassis_move->angle_err_pid.out * 1000, local_chassis_move->angle_err_pid.Dout * 1000);

        // 腿部角度PID
        // printf("%.2f, %.2f\n", local_chassis_move->right_leg.leg_angle * 57.3f, local_chassis_move->leg_angle_set * 57.3f,
        //         local_chassis_move->right_leg.back_joint.give_current,
        //         local_chassis_move->right_leg.front_joint.give_current);

        vTaskDelay(10);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif  
    }
}


/**
  * @brief          蜂鸣器报警
  * @param[in]      num:响声次数
  * @retval         none
  */
void buzzer_warn(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
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
