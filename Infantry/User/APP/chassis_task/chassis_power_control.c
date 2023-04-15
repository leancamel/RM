/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
//#include "detect_task.h"

// #define POWER_LIMIT         80.0f
#define WARNING_POWER       100.0f   
#define WARNING_POWER_BUFF  20.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    // uint8_t robot_id = get_robot_id();
    // if(toe_is_error(REFEREE_TOE))
    // {
    //     total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    // }
    // if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    // {
    //     total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    // }
    // else
	fp32 POWER_LIMIT = 40.0f;
	get_chassis_power_limit(&POWER_LIMIT);
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        //功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //缩小WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //缩小
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //功率大于WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {
                fp32 power_scale;
                //功率小于80w
                if(chassis_power < POWER_LIMIT)
                {
                    //缩小
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                //功率大于80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //功率小于WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    
    total_current = 0.0f;
    //计算原本电机电流设定
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_chassis[i].give_current);
    }
    

    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_chassis[0].give_current *= current_scale;
        chassis_power_control->motor_chassis[1].give_current *= current_scale;
        chassis_power_control->motor_chassis[2].give_current *= current_scale;
        chassis_power_control->motor_chassis[3].give_current *= current_scale;
    }
}
