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
 
#ifndef USER_TASK_H
#define USER_TASK_H

// 是否开启蜂鸣器报警，1 on ： 2 off
#define USE_BUZZER_WARNING 1

enum errorType
{
  MOTOR_LOST = 0,
  MOTOR_TEMP_HIGH,
};

extern void UserTask(void *pvParameters);

#endif
