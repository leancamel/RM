/**
  ****************************RM Warrior 2023****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/3/         wzl              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#include "shoot.h"
#include "main.h"

#include "arm_math.h"
#include "user_lib.h"
#include "trigger.h"
// #include "bsp_laser.h"
// #include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "pid.h"
// #include "detect_task.h"

#define shoot_fric1_on(pwm) fric1_on((pwm))
#define shoot_fric2_on(pwm) fric2_on((pwm))
#define shoot_fric_off()    fric_off()     



/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

/**
  * @brief          开启发射机构，子弹上膛初始化
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_on_reset(void);

/**
  * @brief          堵转判断，如果判断为堵转，改变状态
  * @param[in]      void
  * @retval         void
  */
static void is_stuck_bullet(int16_t block_time_set);

/**
  * @brief          已经判断为堵转，进行倒转操作
  * @param[in]      void
  * @retval         void
  */
static void stuck_bullet(void);



shoot_control_t shoot_control;          //射击数据

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    static const fp32 Trigger_anger_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
    //初始化PID
    PID_Init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_Init(&shoot_control.trigger_anger_pid, PID_POSITION, Trigger_anger_pid, TRIGGER_ANGER_PID_MAX_OUT, TRIGGER_ANGER_PID_MAX_IOUT);
    //更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = power_on_init;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据


    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //拨弹轮速度为0，停止旋转
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        //摩擦轮准备子弹上膛
        shoot_bullet_on_reset();
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        //开始射击子弹
        shoot_bullet_control();
    }


    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        // shoot_laser_off();
        shoot_control.given_current = 0;
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
    }
    else
    {
        //计算拨弹轮电机PID
        PID_Calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }

        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
    shoot_fric1_on(shoot_control.fric_pwm1);
    shoot_fric2_on(shoot_control.fric_pwm2);
    
    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //摩擦轮达到最大转速，等待子弹自动上膛
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，开始射击任务
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    


    // if(shoot_control.shoot_mode > SHOOT_READY_FRIC)
    // {
    //     //鼠标长按或者开关长期处于下档一直进入射击状态 保持连发
    //     if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
    //     {
    //         shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
    //     }
    //     else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    //     {
    //         shoot_control.shoot_mode =SHOOT_READY_BULLET;
    //     }
    // }

    // get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    // if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    // {
    //     if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    //     {
    //         shoot_control.shoot_mode =SHOOT_READY_BULLET;
    //     }
    // }

    // //如果云台状态是 无力状态，就关闭射击
    // if (gimbal_cmd_to_shoot_stop())
    // {
    //     shoot_control.shoot_mode = SHOOT_STOP;
    // }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
    //得到电机反馈速度
    shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

    //电机圈数重置， 因为输出轴旋转187圈， 电机轴旋转 3591圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -FULL_COUNT;
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT;
    }

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
     
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;

    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
    static uint16_t up_time = 0;
    if (shoot_control.press_r)
    {
        up_time = UP_ADD_TIME;
    }

    if (up_time > 0)
    {
        shoot_control.fric1_ramp.max_value = FRIC_UP;
        shoot_control.fric2_ramp.max_value = FRIC_UP;
        up_time--;
    }
    else
    {
        shoot_control.fric1_ramp.max_value = FRIC_DOWN;
        shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    }
}

/**
  * @brief          堵转判断，如果判断为堵转，改变状态
  * @param[in]      int16_t
  * @retval         void
  */
static void is_stuck_bullet(int16_t block_time_set)
{
    //设置速度，开始电机的转动
    shoot_control.speed_set = shoot_control.trigger_speed_set;

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < block_time_set)
    {
        shoot_control.block_time++;
    }
    else if (shoot_control.block_time == block_time_set)
    {
        //子弹上膛，或者卡弹，改变拨弹轮运动模式
        shoot_control.move_flag = Is_stuck_Bullet;
        shoot_control.block_time = 0;
        //设置倒转时间
        shoot_control.reverse_time = REVERSE_TIME;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          已经判断为堵转，进行倒转操作
  * @param[in]      void
  * @retval         void
  */
static void stuck_bullet(void)
{
    if( shoot_control.reverse_time > 0)
    {
        //使波弹轮倒转
        shoot_control.speed_set = -REVERSE_SPEED_SET;
        shoot_control.reverse_time--;
    }
    else if(shoot_control.reverse_time <= 0)
    {
        shoot_control.speed_set = 0.0f;
        //子弹就位，准备发射
        shoot_control.move_flag = Bullet_Alraedy;
    }
}

/**
  * @brief          开始射击，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    //拨动 1/6PI的角度
    if (shoot_control.move_flag == Bullet_Alraedy)
    {
        //拨动pi/6的角度，发射一颗子弹
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_SIX);
        //切换模式，开始子弹射击
        shoot_control.move_flag = Start_Shoot_bullet;
    }
    else if(shoot_control.move_flag == Start_Shoot_bullet)
    {
        // //PID闭环角度
        // PID_Calc(&shoot_control.trigger_anger_pid,shoot_control.angle,shoot_control.set_angle);
        // //给定速度
        // shoot_control.speed_set = shoot_control.trigger_speed_set;

        //设置速度，拨动拨弹轮电机
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        // //达到控制角度后，改变状态，已经发射完成
        // if (fabs(rad_format(shoot_control.set_angle - shoot_control.angle))< 0.05f)
        // {
        //     //到达设定角度，停止电机
        //     shoot_control.speed_set = 0.0f;
        //     //达到设定角度，切换模式
        //     shoot_control.shoot_mode = SHOOT_DONE;
        //     shoot_control.move_flag = power_on_init;
        // }
        //堵转判断
        is_stuck_bullet(1000);
    }

    if (fabs(rad_format(shoot_control.set_angle - shoot_control.angle))< 0.05f)
    {
        //到达设定角度，停止电机
        shoot_control.speed_set = 0.0f;
        //达到设定角度，切换模式
        shoot_control.shoot_mode = SHOOT_DONE;
        shoot_control.move_flag = power_on_init;
    }

}

/**
  * @brief          控制子弹的上膛操作
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_on_reset(void)
{
    //刚上电，控制转动一定的角度，通过堵转来判断子弹是否上膛
    if (shoot_control.move_flag == power_on_init)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_SIX);
        //进入准备上膛阶段，开始子弹上膛
        shoot_control.move_flag = Enter_the_init_state;
    }
    else if(shoot_control.move_flag == Enter_the_init_state)
    {
        // //PID闭环角度
        // PID_Calc(&shoot_control.trigger_anger_pid,shoot_control.angle,shoot_control.set_angle);
        // //给定速度
        // shoot_control.speed_set = shoot_control.trigger_speed_set;

        //设置速度，拨动拨弹轮电机
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        //达到控制角度，上膛初始化
        if (rad_format(shoot_control.set_angle - shoot_control.angle)< 0.05f)
        {
            //到达设定角度，停止电机
            shoot_control.trigger_speed_set = 0.0f;
            //子弹上膛，改变拨弹轮运动模式
            shoot_control.move_flag = Bullet_Alraedy;
        }

        //堵转判断
        is_stuck_bullet(100);
    }
    else if(shoot_control.move_flag == Is_stuck_Bullet)
    {
        //堵转后的倒转处理
        stuck_bullet();
    }
    else if (shoot_control.move_flag == Bullet_Alraedy)
    {
        //子弹上膛完毕，射击准备
        shoot_control.shoot_mode = SHOOT_READY;
    }
}

//测试摩擦轮电机用
void fric_loop(void)
{
    static double compare = 1000;
    if(compare < 1400)
    {
        compare += 0.1;
    }
    fric1_on((uint16_t)compare);
    fric2_on((uint16_t)compare);
}
