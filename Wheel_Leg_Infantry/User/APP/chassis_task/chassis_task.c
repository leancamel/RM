/**
  ****************************RM Warrior 2023****************************
  * @file    chassis_remote_control.c
  * @author  pxx
  * @version 
  * @date    2023-11-05
  * @brief   完成轮腿平衡底盘控制任务。
  *          
  *                               
  ****************************RM Warrior 2023****************************
  */
 
#include "chassis_task.h"
// #include "chassis_behaviour.h"

#include "arm_math.h"
#include "leg_pos.h"
#include "leg_conv.h"
#include "leg_spd.h"
#include "lqr_k.h"

#include "pid.h"
#include "INS_Task.h"
#include "state_estimate.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "stdio.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//底盘运动数据
static chassis_move_t chassis_move;

//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘状态改变后处理控制量的改变
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
// 腿部运动范围限制
void chassis_leg_limit(chassis_move_t *chassis_move_control, fp32 l_set);
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, int16_t offset_ecd);
//限制最大转矩电流
static int16_t limitted_motor_current(fp32 current, fp32 max);

// 机器人离地检测
static bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect);

void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);
    TickType_t Chassis_LastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&Chassis_LastWakeTime, 2);

        //遥控器设置状态
        chassis_set_mode(&chassis_move);
        //遥控器状态切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);
		//底盘功率限制
		// chassis_power_control(&chassis_move);
		//can发送关节数据
        CAN_CMD_WHEEL(chassis_move.right_leg.wheel_motor.give_current, chassis_move.left_leg.wheel_motor.give_current);
		CAN_CMD_CHASSIS(chassis_move.left_leg.front_joint.give_current, chassis_move.left_leg.back_joint.give_current, 
                        chassis_move.right_leg.back_joint.give_current, chassis_move.right_leg.front_joint.give_current);
		// CAN_CMD_CHASSIS(0, 0, 0, 0);
        // CAN_CMD_WHEEL(chassis_move.chassis_RC->rc.ch[1] * 5, -chassis_move.chassis_RC->rc.ch[1] * 5);
    }
}   



/**
  * @brief          底盘初始化
  * @author         pxx
  * @param          chassis_move_init   底盘结构体指针
  * @retval         void
  */
void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //轮腿底盘pid值
    const static fp32 leg_length_pid[3] = {LEG_LENGTH_PID_KP, LEG_LENGTH_PID_KI, LEG_LENGTH_PID_KD };
    const static fp32 leg_angle_err_pid[3] = {ANGLE_ERR_PID_KP, ANGLE_ERR_PID_KI, ANGLE_ERR_PID_KD };
    const static fp32 leg_angle_dot_pid[3] = {ANGLE_DOT_PID_KP, ANGLE_DOT_PID_KI, ANGLE_DOT_PID_KD };
    const static fp32 robot_roll_pid[3] = {ROLL_CTRL_PID_KP, ROLL_CTRL_PID_KI, ROLL_CTRL_PID_KD};
    //底盘旋转环pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    const static fp32 chassis_yaw_gyro_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //一阶低通滤波初始化
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 state_xdot_constant[1] = {CHASSIS_SPEED_NUM};

    //底盘开机状态为无力、不接触地面
    chassis_move_init->chassis_mode = CHASSIS_FORCE_RAW;
    chassis_move_init->touchingGroung = false;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_move_init->chassis_imu_gyro = get_gyro_data_point();
    chassis_move_init->chassis_imu_accel = get_accel_data_point();
    //获取关节电机指针
    chassis_move_init->right_leg.front_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(3);
    chassis_move_init->right_leg.back_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(2);
    chassis_move_init->left_leg.back_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(1);
    chassis_move_init->left_leg.front_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(0);
    //获取驱动轮电机指针
    chassis_move_init->right_leg.wheel_motor.wheel_motor_measure = get_Right_Wheel_Motor_Measure_Point();
    chassis_move_init->left_leg.wheel_motor.wheel_motor_measure = get_Left_Wheel_Motor_Measure_Point();

    //初始化PID 运动
    PID_Init(&chassis_move_init->left_leg_length_pid, PID_POSITION, leg_length_pid, LEG_LENGTH_PID_MAX_OUT, LEG_LENGTH_PID_MAX_IOUT);
    PID_Init(&chassis_move_init->right_leg_length_pid, PID_POSITION, leg_length_pid, LEG_LENGTH_PID_MAX_OUT, LEG_LENGTH_PID_MAX_IOUT);

    //初始双腿误差控制pid
    PID_Init(&chassis_move_init->angle_err_pid, PID_POSITION, leg_angle_err_pid, ANGLE_ERR_PID_MAX_OUT, ANGLE_ERR_PID_MAX_IOUT);
    PID_Init(&chassis_move_init->angle_dot_pid, PID_POSITION, leg_angle_dot_pid, ANGLE_DOT_PID_MAX_OUT, ANGLE_DOT_PID_MAX_IOUT);
    //初始化横滚角pid
    PID_Init(&chassis_move_init->roll_ctrl_pid, PID_POSITION, robot_roll_pid, ROLL_CTRL_PID_MAX_OUT, ROLL_CTRL_PID_MAX_IOUT);
    //初始化旋转PID
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    PID_Init(&chassis_move_init->chassis_yaw_gyro_pid, PID_POSITION, chassis_yaw_gyro_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->state_xdot_filter, CHASSIS_CONTROL_TIME, state_xdot_constant);

    //关节电机机械零点设置
    chassis_move_init->left_leg.front_joint.offset_ecd = 4162;
    chassis_move_init->left_leg.back_joint.offset_ecd = 4879 + 4096;
    chassis_move_init->right_leg.front_joint.offset_ecd = 3803;
    chassis_move_init->right_leg.back_joint.offset_ecd = 2771 - 4096;
   
    //关节电机限制角度，实际上是限制腿长
    chassis_move_init->leg_length_set = LEG_LENGTH_INIT;
    chassis_move_init->leg_length_max = LEG_LENGTH_MAX;
    chassis_move_init->leg_length_min = LEG_LENGTH_MIN;
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          ͨ设置遥控器设置状态
  * @author         pxx
  * @param          chassis_move_mode   底盘结构体指针
  * @retval         void
  */
void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    // chassis_behaviour_mode_set(chassis_move_mode);
    //遥控器设置行为模式
    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_FORCE_RAW;
    }
}

/**
  * @brief          遥控器状态切换数据保存
  * @author         pxx
  * @param          chassis_move_transit    底盘结构体指针
  * @retval         void
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //切入跟随云台模式(暂为开启lqr控制)
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        chassis_move_transit->state_ref.x = 0;
        chassis_move_transit->state_set.x = 0;
    }
    //切入不跟随云台模式(暂为开启腿长控制)
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        chassis_move_transit->state_ref.x = 0;
        chassis_move_transit->state_set.x = 0;
    }
    //切入无力模式，清空里程计
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_FORCE_RAW) && chassis_move_transit->chassis_mode == CHASSIS_FORCE_RAW)
    {
        // chassis_move_transit->state_ref.x = 0;
        // chassis_move_transit->state_set.x = 0;

        chassis_move_transit->left_support_force = 0.0f;
        chassis_move_transit->right_support_force = 0.0f;
        chassis_move_transit->leg_length_set = LEG_LENGTH_INIT;
    }
    

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          底盘数据更新
  * @author         pxx
  * @param          chassis_move_update 底盘结构体指针
  * @retval         void
  */
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    //计算底盘姿态角度，陀螺仪需要在底盘上
    //陀螺仪数据映射
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - 0.01994f);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET) - 0.0154f;

    //更新关节电机角度
    chassis_move_update->right_leg.front_joint.angle = motor_ecd_to_angle_change(chassis_move_update->right_leg.front_joint.joint_motor_measure->ecd,
                                                                                                chassis_move_update->right_leg.front_joint.offset_ecd);
    chassis_move_update->right_leg.back_joint.angle = motor_ecd_to_angle_change(chassis_move_update->right_leg.back_joint.joint_motor_measure->ecd,
                                                                                                chassis_move_update->right_leg.back_joint.offset_ecd);                                                                                
    chassis_move_update->left_leg.front_joint.angle = 2*PI - motor_ecd_to_angle_change(chassis_move_update->left_leg.front_joint.joint_motor_measure->ecd,
                                                                                                chassis_move_update->left_leg.front_joint.offset_ecd);
    chassis_move_update->left_leg.back_joint.angle = 2*PI - motor_ecd_to_angle_change(chassis_move_update->left_leg.back_joint.joint_motor_measure->ecd,
                                                                                                chassis_move_update->left_leg.back_joint.offset_ecd); 
    
    //更新关节转动速度
    chassis_move_update->right_leg.back_joint.angle_dot = chassis_move_update->right_leg.back_joint.joint_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE;
    chassis_move_update->right_leg.front_joint.angle_dot = chassis_move_update->right_leg.front_joint.joint_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE;
    chassis_move_update->left_leg.back_joint.angle_dot = -chassis_move_update->left_leg.back_joint.joint_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE;
    chassis_move_update->left_leg.front_joint.angle_dot = -chassis_move_update->left_leg.front_joint.joint_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE;

    //VMC 计算腿部姿态
    fp32 L0_PHI[2];
    leg_pos(chassis_move_update->right_leg.back_joint.angle, chassis_move_update->right_leg.front_joint.angle, L0_PHI);
    chassis_move_update->right_leg.leg_angle = L0_PHI[1];
    chassis_move_update->right_leg.leg_length = L0_PHI[0];

    leg_pos(chassis_move_update->left_leg.back_joint.angle, chassis_move_update->left_leg.front_joint.angle, L0_PHI);
    chassis_move_update->left_leg.leg_angle = L0_PHI[1];
    chassis_move_update->left_leg.leg_length = L0_PHI[0];

    leg_spd(chassis_move_update->left_leg.back_joint.angle_dot, chassis_move_update->left_leg.front_joint.angle_dot,
            chassis_move_update->left_leg.back_joint.angle, chassis_move_update->left_leg.front_joint.angle, L0_PHI);
    chassis_move_update->left_leg.length_dot = L0_PHI[0];
    chassis_move_update->left_leg.angle_dot = L0_PHI[1];

    leg_spd(chassis_move_update->right_leg.back_joint.angle_dot, chassis_move_update->right_leg.front_joint.angle_dot,
            chassis_move_update->right_leg.back_joint.angle, chassis_move_update->right_leg.front_joint.angle, L0_PHI);
    chassis_move_update->right_leg.length_dot = L0_PHI[0];
    chassis_move_update->right_leg.angle_dot = L0_PHI[1];

    //双腿状态量取平均即机器人腿部姿态(暂时没用到)
    chassis_move_update->leg_angle = 0.5f * (chassis_move_update->right_leg.leg_angle + chassis_move_update->left_leg.leg_angle);
    chassis_move_update->leg_length = 0.5f * (chassis_move_update->right_leg.leg_length + chassis_move_update->left_leg.leg_length);
    chassis_move_update->leg_length_dot = 0.5f * (chassis_move_update->right_leg.length_dot + chassis_move_update->left_leg.length_dot);
    chassis_move_update->leg_angle_dot = 0.5f * (chassis_move_update->right_leg.angle_dot + chassis_move_update->left_leg.angle_dot);

    //更新驱动轮电机速度，加速度是速度的PID微分
    chassis_move_update->right_leg.wheel_motor.speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->right_leg.wheel_motor.wheel_motor_measure->speed_rpm;
    chassis_move_update->left_leg.wheel_motor.speed = -CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->left_leg.wheel_motor.wheel_motor_measure->speed_rpm;

    //更新底盘旋转速度wz，坐标系为右手系
    chassis_move_update->wz = 0.5f * (chassis_move_update->right_leg.wheel_motor.speed - chassis_move_update->left_leg.wheel_motor.speed) / MOTOR_DISTANCE_TO_CENTER;
    // TODO: 加入Kalman Filter得到机器人位移和速度
    first_order_filter_cali(&chassis_move_update->state_xdot_filter, (chassis_move_update->right_leg.wheel_motor.speed + chassis_move_update->left_leg.wheel_motor.speed) * 0.5f);
    SpeedEstimation(chassis_move_update);
    //底盘状态量组装
    chassis_move_update->state_ref.theta = chassis_move_update->leg_angle - PI/2 - chassis_move_update->chassis_pitch; // 注意theta并不是腿与机体的夹角
    chassis_move_update->state_ref.theta_dot = chassis_move_update->leg_angle_dot - *(chassis_move_update->chassis_imu_gyro + INS_GYRO_Y_ADDRESS_OFFSET);
    chassis_move_update->state_ref.x_dot = chassis_move_update->state_xdot_filter.out;
    // chassis_move_update->state_ref.x_dot = chassis_move_update->estimated_speed;
    chassis_move_update->state_ref.x += chassis_move_update->state_ref.x_dot * CHASSIS_CONTROL_TIME;
    chassis_move_update->state_ref.phi = chassis_move_update->chassis_pitch;
    chassis_move_update->state_ref.phi_dot = *(chassis_move_update->chassis_imu_gyro + INS_GYRO_Y_ADDRESS_OFFSET);

    // 机器人离地判断
    Robot_Offground_detect(chassis_move_update);

    // 以下用于手动给定机器人触地状态，方便调试，不使用注释
    // if(switch_is_up(chassis_move_update->chassis_RC->rc.s[MODE_CHANNEL])){
    //     chassis_move_update->touchingGroung = true;
    // }
    // else
    //     chassis_move_update->touchingGroung = false;
}

/**
  * @brief          设置遥控器输入控制量
  * @author         pxx
  * @param          chassis_move_control    底盘结构体指针
  * @retval         void
  */
void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    //设置速度
    fp32 vx_set = 0.0f, l_set = 0.0f, angle_set = 0.0f, roll_set = 0.0f;
    // chassis_behaviour_control_set(&vx_set, &l_set, &angle_set, chassis_move_control);
    l_set = chassis_move_control->leg_length_set;
    if(switch_is_down(chassis_move_control->chassis_RC->rc.s[1]))// 左边拨到最下档才允许摇杆操控，防止不小心误触
    {
        chassis_rc_to_control_vector(&vx_set, &angle_set, chassis_move_control);
    }
    else if(switch_is_up(chassis_move_control->chassis_RC->rc.s[1]))// 控制腿长以及横滚角
    {
        chassis_rc_to_control_euler(&l_set, &roll_set, chassis_move_control);
    }
    else
    {
        vx_set = 0.0f;
        angle_set = 0.0f;
        roll_set = 0.0f;
    }
    
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_control->chassis_yaw_set = rad_format(chassis_move_control->chassis_yaw_set + angle_set);
        chassis_move_control->chassis_roll_set = roll_set;
        chassis_leg_limit(chassis_move_control, l_set);

        chassis_move_control->state_set.phi = 0.0f;
        chassis_move_control->state_set.phi_dot = 0.0f;
        chassis_move_control->state_set.theta = 0.0f;
        chassis_move_control->state_set.theta_dot = 0.0f;
        
        static uint16_t T_count = 1000;
        if(vx_set != 0)
        {
            T_count = 0;
            chassis_move_control->state_set.x_dot = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
            chassis_move_control->state_set.x += chassis_move_control->state_set.x_dot * CHASSIS_CONTROL_TIME;
        }
        else // 停止立即刹车
        {
            chassis_move_control->state_set.x_dot = 0.0f;
            if(T_count < 750){
                T_count++;
                chassis_move_control->state_set.x = chassis_move_control->state_ref.x;
            }
            else{
                chassis_move_control->state_set.x = chassis_move_control->state_set.x;
            }
        }
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        // 保持底盘与云台的相对角度不变
        ;
    }
}

// 腿部运动范围限制
void chassis_leg_limit(chassis_move_t *chassis_move_control, fp32 l_set)
{
    chassis_move_control->leg_length_set = l_set;

    if(chassis_move_control->leg_length_set > LEG_LENGTH_MAX)
        chassis_move_control->leg_length_set = LEG_LENGTH_MAX;
    else if(chassis_move_control->leg_length_set < LEG_LENGTH_MIN)
        chassis_move_control->leg_length_set = LEG_LENGTH_MIN;
}

/**
  * @brief          遥控器的数据处理成底盘的前进vx速度，vy速度
  * @author         pxx
  * @param          vx_set  x轴前进速度设置，m/s
  * @param          vy_set  y轴前进速度设置，m/s
  * @param          chassis_move_rc_to_vector   底盘结构体指针
  * @retval         void
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *add_yaw_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || add_yaw_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, wz_channel;
    fp32 vx_set_channel, add_yaw_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    add_yaw_channel = wz_channel * -CHASSIS_WZ_RC_SEN;

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    if(add_yaw_channel < CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN && add_yaw_channel > -CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN)
    {
        add_yaw_channel = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *add_yaw_set = add_yaw_channel;
}

/**
  * @brief          遥控器的数据处理成轮腿底盘腿长、roll角
  * @author         pxx
  * @param          l_set  x轴前进速度设置，m/s
  * @param          roll_set  y轴前进速度设置，m/s
  * @param          chassis_move_rc_to_vector   底盘结构体指针
  * @retval         void
  */
void chassis_rc_to_control_euler(fp32 *l_set, fp32 *roll_set, chassis_move_t *chassis_move_rc_to_euler)
{
    fp32 add_l, add_l_channel;
    rc_deadline_limit(chassis_move_rc_to_euler->chassis_RC->rc.ch[CHASSIS_L_CHANNEL], add_l_channel, CHASSIS_RC_DEADLINE);
    add_l = add_l_channel * LEH_LENGTH_RC_SEN;
    *l_set += add_l;

    fp32 roll_channel;
    rc_deadline_limit(chassis_move_rc_to_euler->chassis_RC->rc.ch[CHASSIS_ROLL_CHANNEL], roll_channel, CHASSIS_RC_DEADLINE);
    *roll_set = roll_channel * CHASSIS_ROLL_RC_SEN;
}


//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, int16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if(relative_ecd < 0)
        relative_ecd += joint_ecd_range;

    return relative_ecd * Motor_Ecd_to_Rad;
}

/**
  * @brief          底盘控制PID计算
  * @author         pxx
  * @param          chassis_move_control_loop   底盘结构体指针
  * @retval         void
  */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)// TODO: 将该函数重写，剥离其中的控制模块以及各种约束限制
{
    if(chassis_move_control_loop->chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_control_loop->right_leg.front_joint.give_current = 0;
        chassis_move_control_loop->right_leg.back_joint.give_current = 0;
        chassis_move_control_loop->left_leg.front_joint.give_current = 0;
        chassis_move_control_loop->left_leg.back_joint.give_current = 0;
        chassis_move_control_loop->right_leg.wheel_motor.give_current = 0;
        chassis_move_control_loop->left_leg.wheel_motor.give_current = 0;
        return;
    }

    fp32 l_force = 0.0f, r_force = 0.0f;
    fp32 err_tor = 0.0f, yaw_err_force = 0.0f, roll_err = 0.0f;
    // 反馈矩阵乘以一定系数用于调整
    fp32 coefficient[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
                            {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
    fp32 kRes[12] = {0}, k[2][6] = {0};
    lqr_k(chassis_move_control_loop->leg_length, kRes);
    if(chassis_move_control_loop->touchingGroung) //正常触地状态
    {
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * coefficient[j][i];
    }
    else //腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
    {
        memset(k, 0, sizeof(k));
        k[1][0] = kRes[1] * 1.0f;
        k[1][1] = kRes[3] * 1.0f;
    }
    fp32 x[6] = {chassis_move_control_loop->state_set.theta - chassis_move_control_loop->state_ref.theta,
                chassis_move_control_loop->state_set.theta_dot - chassis_move_control_loop->state_ref.theta_dot,
                chassis_move_control_loop->state_set.x - chassis_move_control_loop->state_ref.x,
                chassis_move_control_loop->state_set.x_dot - chassis_move_control_loop->state_ref.x_dot,
                chassis_move_control_loop->state_set.phi - chassis_move_control_loop->state_ref.phi,
                 chassis_move_control_loop->state_set.phi_dot - chassis_move_control_loop->state_ref.phi_dot};
    chassis_move_control_loop->wheel_tor = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
    chassis_move_control_loop->leg_tor = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];
    chassis_move_control_loop->wheel_tor *= 0.5f; // 两条腿，每条腿只取一半
    chassis_move_control_loop->leg_tor *= 0.5f;
    // TODO: lqr输出端一阶低通滤波?
    
    // PID计算转向 左右轮力矩差 注意过零保护
    chassis_move_control_loop->wz_set = PID_Calc(&chassis_move_control_loop->chassis_angle_pid, rad_format(chassis_move_control_loop->chassis_yaw - chassis_move_control_loop->chassis_yaw_set), 0);
    yaw_err_force = PID_Calc(&chassis_move_control_loop->chassis_yaw_gyro_pid, *(chassis_move_control_loop->chassis_imu_gyro+INS_GYRO_Z_ADDRESS_OFFSET), chassis_move_control_loop->wz_set);
    
    // PID补偿横滚角roll，这里作为腿长控制的外环，而不是直接补偿支持力
    if(chassis_move_control_loop->touchingGroung == true)
        roll_err = PID_Calc(&chassis_move_control_loop->roll_ctrl_pid, chassis_move_control_loop->chassis_roll, chassis_move_control_loop->chassis_roll_set);

    chassis_move_control_loop->left_leg.leg_length_set = chassis_move_control_loop->leg_length_set + roll_err;
    chassis_move_control_loop->right_leg.leg_length_set = chassis_move_control_loop->leg_length_set - roll_err;
    if(chassis_move_control_loop->left_leg.leg_length_set < LEG_LENGTH_MIN)
    {
        chassis_move_control_loop->right_leg.leg_length_set += LEG_LENGTH_MIN - chassis_move_control_loop->left_leg.leg_length_set;
    }
    else if(chassis_move_control_loop->right_leg.leg_length_set < LEG_LENGTH_MIN)
    {
        chassis_move_control_loop->left_leg.leg_length_set += LEG_LENGTH_MIN - chassis_move_control_loop->right_leg.leg_length_set;
    }
     
    // 使用双腿长度的平均值，离地修改目标腿长
    if(chassis_move_control_loop->touchingGroung) //正常触地状态
    {
        r_force = PID_Calc(&chassis_move_control_loop->right_leg_length_pid, chassis_move_control_loop->right_leg.leg_length, chassis_move_control_loop->right_leg.leg_length_set);
        l_force = PID_Calc(&chassis_move_control_loop->left_leg_length_pid, chassis_move_control_loop->left_leg.leg_length, chassis_move_control_loop->left_leg.leg_length_set);

        static const fp32 gravity_comp = 3.0f; // 补偿机体重力
        chassis_move_control_loop->left_support_force = l_force + gravity_comp;
        chassis_move_control_loop->right_support_force = r_force + gravity_comp;
    }
    else
    {
        r_force = PID_Calc(&chassis_move_control_loop->right_leg_length_pid, chassis_move_control_loop->right_leg.leg_length, LEG_LENGTH_INIT);
        l_force = PID_Calc(&chassis_move_control_loop->left_leg_length_pid, chassis_move_control_loop->left_leg.leg_length, LEG_LENGTH_INIT);

        chassis_move_control_loop->left_support_force = l_force;
        chassis_move_control_loop->right_support_force = r_force;
    }


    // 双腿角度误差控制
    fp32 angle_dot = PID_Calc(&chassis_move_control_loop->angle_err_pid, (chassis_move_control_loop->right_leg.leg_angle - chassis_move_control_loop->left_leg.leg_angle), 0);
    err_tor = PID_Calc(&chassis_move_control_loop->angle_dot_pid, (chassis_move_control_loop->right_leg.angle_dot - chassis_move_control_loop->left_leg.angle_dot), angle_dot);

    // VMC 虚拟力解算
    fp32 tor_vector[2] = {0.0f};
    leg_conv(-chassis_move_control_loop->right_support_force, (chassis_move_control_loop->leg_tor+err_tor), 
            chassis_move_control_loop->right_leg.back_joint.angle, chassis_move_control_loop->right_leg.front_joint.angle, tor_vector);
    chassis_move_control_loop->right_leg.back_joint.give_current = limitted_motor_current(tor_vector[1] * M3508_TOR_TO_CAN_DATA, MAX_MOTOR_CAN_CURRENT);
    chassis_move_control_loop->right_leg.front_joint.give_current = limitted_motor_current(tor_vector[0] * M3508_TOR_TO_CAN_DATA, MAX_MOTOR_CAN_CURRENT);

    leg_conv(-chassis_move_control_loop->left_support_force, (chassis_move_control_loop->leg_tor-err_tor), 
            chassis_move_control_loop->left_leg.back_joint.angle, chassis_move_control_loop->left_leg.front_joint.angle, tor_vector);
    chassis_move_control_loop->left_leg.back_joint.give_current = limitted_motor_current(-tor_vector[1] * M3508_TOR_TO_CAN_DATA, MAX_MOTOR_CAN_CURRENT);
    chassis_move_control_loop->left_leg.front_joint.give_current = limitted_motor_current(-tor_vector[0] * M3508_TOR_TO_CAN_DATA, MAX_MOTOR_CAN_CURRENT);
 
    if(chassis_move_control_loop->touchingGroung)
    {
        chassis_move_control_loop->right_leg.wheel_motor.give_current = limitted_motor_current((chassis_move_control_loop->wheel_tor+yaw_err_force) * M3508_TOR_TO_CAN_DATA, MAX_MOTOR_CAN_CURRENT);
        chassis_move_control_loop->left_leg.wheel_motor.give_current = limitted_motor_current(-(chassis_move_control_loop->wheel_tor-yaw_err_force) * M3508_TOR_TO_CAN_DATA, MAX_MOTOR_CAN_CURRENT);
    }
    else
    {
        chassis_move_control_loop->right_leg.wheel_motor.give_current = 0;
        chassis_move_control_loop->left_leg.wheel_motor.give_current = 0;
    }
}

static int16_t limitted_motor_current(fp32 current, fp32 max)
{
    if(abs(current) > max)
    {
        current = current > 0 ? max : -max;
    }
    return (int16_t)current;
}

//获取底盘结构体指针
const chassis_move_t *get_chassis_control_point(void)
{
    return &chassis_move;
}

// 机器人支持力解算，判断离地
static bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect)
{
    static fp32 last_length_dot = 0.0f; // 差分计算腿长变化的加速度
    static fp32 last_theta_dot = 0.0f;  // 差分计算腿部倾角变化加速度

    static const fp32 m_w = 0.1939f; // 轮毂的质量
    static const fp32 g = 9.8f;      // 重力加速度
    // 机体竖直方向运动加速度
    fp32 b_acc_z = *(chassis_move_detect->chassis_imu_accel + INS_ACCEL_Z_ADDRESS_OFFSET) - g;
    // 直接使用上一控制周期计算的力矩作为力矩反馈值
    fp32 leg_support_force = (chassis_move_detect->left_support_force + chassis_move_detect->right_support_force);
    // 由于使用了双腿误差控制，双腿角度一致，只计算一次三角函数
    fp32 cos_theta = arm_cos_f32(chassis_move_detect->state_ref.theta);
    fp32 sin_theta = arm_sin_f32(chassis_move_detect->state_ref.theta);

    // 使用差分代替微分运算
    fp32 ddlength = (chassis_move_detect->leg_length_dot - last_length_dot) / CHASSIS_CONTROL_TIME;
    fp32 ddtheta = (chassis_move_detect->state_ref.theta_dot - last_theta_dot) / CHASSIS_CONTROL_TIME;
    ddlength = 0.0f;// 噪声太大了，需要滤波，于是暂时没有使用
    ddtheta = 0.0f;
    last_length_dot = chassis_move_detect->leg_length_dot;
    last_theta_dot = chassis_move_detect->state_ref.theta_dot;

    // 机器人腿部机构作用于驱动轮竖直向下的力
    fp32 P = leg_support_force * cos_theta + chassis_move_detect->leg_tor * sin_theta / chassis_move_detect->leg_length;
    // 驱动轮竖直方向运动加速度
    fp32 w_acc_z = b_acc_z - ddlength * cos_theta
                    + 2.0f * chassis_move_detect->leg_length_dot * chassis_move_detect->state_ref.theta_dot * sin_theta
                    + chassis_move_detect->leg_length * ddtheta * sin_theta
                    + chassis_move_detect->leg_length * chassis_move_detect->state_ref.theta_dot * chassis_move_detect->state_ref.theta_dot * cos_theta;

    chassis_move_detect->ground_force = P + 2 * m_w * g + 2 * m_w * w_acc_z;

    // 使用双阈值的方式进行判断，防止机器人在离地与触地之间反复切换
    if(chassis_move_detect->touchingGroung && chassis_move_detect->ground_force < 6.0f)
        chassis_move_detect->touchingGroung = false;
    else if(!chassis_move_detect->touchingGroung && chassis_move_detect->ground_force > 7.0f){
        chassis_move_detect->touchingGroung = true;
        chassis_move_detect->chassis_yaw_set = chassis_move_detect->chassis_yaw;// 落地后保证朝向
    }
    else
        chassis_move_detect->touchingGroung = chassis_move_detect->touchingGroung;
     
    // if(chassis_move_detect->ground_force < 6.0f || chassis_move_detect->leg_length > chassis_move_detect->leg_length_set * 1.2f)
    //     chassis_move_detect->touchingGroung = false;
    // else
    //     chassis_move_detect->touchingGroung = true;

    // chassis_move_detect->ground_force = w_acc_z;
    // chassis_move_detect->touchingGroung = true;
    return chassis_move_detect->touchingGroung;
}

