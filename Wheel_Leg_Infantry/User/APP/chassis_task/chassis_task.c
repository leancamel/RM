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
#include "chassis_behaviour.h"

#include "arm_math.h"
#include "leg_pos.h"
#include "leg_conv.h"
#include "leg_spd.h"

#include "pid.h"
#include "INS_Task.h"

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
void chassis_leg_limit(chassis_move_t *chassis_move_control, fp32 l_set, fp32 angle_set);
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, int16_t offset_ecd);
//限制最大转矩电流
static int16_t limitted_motor_current(fp32 current, fp32 max);

void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);

    while(1)
    {
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
		//can发送底盘数据
		CAN_CMD_CHASSIS(chassis_move.left_leg.front_joint.give_current, chassis_move.left_leg.back_joint.give_current, 
                        chassis_move.right_leg.back_joint.give_current, chassis_move.right_leg.front_joint.give_current);
		// CAN_CMD_CHASSIS(0, 0, chassis_move.right_leg.back_joint.give_current, chassis_move.right_leg.front_joint.give_current);

        // CAN_CMD_CHASSIS(0, 0, 3000, 0);
        vTaskDelay(2);
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
    const static fp32 leg_angle_pid[3] = {LEG_ANGLE_PID_KP, LEG_ANGLE_PID_KI, LEG_ANGLE_PID_KD };
    const static fp32 leg_angle_err_pid[3] = {ANGLE_ERR_PID_KP, ANGLE_ERR_PID_KI, ANGLE_ERR_PID_KD };
    //底盘旋转环pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};

    //底盘开机状态为无力
    chassis_move_init->chassis_mode = CHASSIS_FORCE_RAW;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_move_init->chassis_imu_gyro = get_gyro_data_point();
    //获取关节电机指针
    chassis_move_init->right_leg.front_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(3);
    chassis_move_init->right_leg.back_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(2);
    chassis_move_init->left_leg.back_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(1);
    chassis_move_init->left_leg.front_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(0);

    //初始化PID 运动
    PID_Init(&chassis_move_init->left_leg_length_pid, PID_POSITION, leg_length_pid, LEG_LENGTH_PID_MAX_OUT, LEG_LENGTH_PID_MAX_IOUT);
    PID_Init(&chassis_move_init->right_leg_length_pid, PID_POSITION, leg_length_pid, LEG_LENGTH_PID_MAX_OUT, LEG_LENGTH_PID_MAX_IOUT);

    PID_Init(&chassis_move_init->left_leg_angle_pid, PID_POSITION, leg_angle_pid, LEG_ANGLE_PID_MAX_OUT, LEG_ANGLE_PID_MAX_IOUT);
    PID_Init(&chassis_move_init->right_leg_angle_pid, PID_POSITION, leg_angle_pid, LEG_ANGLE_PID_MAX_OUT, LEG_ANGLE_PID_MAX_IOUT);
    //初始双腿误差控制pid
    PID_Init(&chassis_move_init->angle_err_pid, PID_POSITION, leg_angle_err_pid, ANGLE_ERR_PID_MAX_OUT, ANGLE_ERR_PID_MAX_IOUT);

    //初始化旋转PID
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);

    //关节电机机械零点设置
    chassis_move_init->left_leg.front_joint.offset_ecd = 5540;
    chassis_move_init->left_leg.back_joint.offset_ecd = 1451 + 4096;
    chassis_move_init->right_leg.front_joint.offset_ecd = 2438;
    chassis_move_init->right_leg.back_joint.offset_ecd = 6167 - 4096;
    //关节电机限制角度，实际上是限制腿长
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

    chassis_behaviour_mode_set(chassis_move_mode);
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

    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入无力模式，清空里程计
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_FORCE_RAW) && chassis_move_transit->chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_transit->state_ref.x = 0;
        chassis_move_transit->state_set.x = 0;
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
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

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
    chassis_move_update->left_leg.back_joint.angle_dot = chassis_move_update->left_leg.back_joint.joint_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE;
    chassis_move_update->left_leg.front_joint.angle_dot = chassis_move_update->left_leg.front_joint.joint_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE;

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

    //更新摆杆与竖直方向夹角
    chassis_move_update->state_ref.theta = chassis_move_update->leg_angle - chassis_move_update->chassis_pitch;

    //更新机体与水平方向夹角，如果陀螺仪在底盘上，夹角就是陀螺仪的pitch轴角度
    chassis_move_update->state_ref.phi = chassis_move_update->chassis_pitch;
    chassis_move_update->state_ref.phi_dot = *(chassis_move_update->chassis_imu_gyro + INS_PITCH_ADDRESS_OFFSET); // 未验证地址是否正确，与C板安装有关
 
    //更新驱动轮电机速度，加速度是速度的PID微分
    chassis_move_update->right_leg.wheel_motor.speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->right_leg.wheel_motor.wheel_motor_measure->speed_rpm;
    chassis_move_update->left_leg.wheel_motor.speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->left_leg.wheel_motor.wheel_motor_measure->speed_rpm;

    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->state_ref.x_dot = (chassis_move_update->right_leg.wheel_motor.speed + chassis_move_update->left_leg.wheel_motor.speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->state_ref.x += chassis_move_update->state_ref.x_dot * CHASSIS_CONTROL_TIME;
    chassis_move_update->wz = (chassis_move_update->right_leg.wheel_motor.speed - chassis_move_update->left_leg.wheel_motor.speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
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
    fp32 vx_set = 0.0f, l_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &l_set, &angle_set, chassis_move_control);
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //放弃跟随云台
        //这个模式下，角度设置的为 角速度
        fp32 chassis_wz = angle_set;
        chassis_move_control->state_set.x_dot = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->wz_set = chassis_wz;
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_POSITION_LEG)
    {
        // 当腿部控制模式时，angle为腿部摆杆角度
        chassis_leg_limit(chassis_move_control, l_set, angle_set);
    }
}

// 腿部运动范围限制
void chassis_leg_limit(chassis_move_t *chassis_move_control, fp32 l_set, fp32 angle_set)
{
#if defined(LEG_POS_TEST_START)
    chassis_move_control->leg_length_set = l_set;
    chassis_move_control->leg_angle_set = angle_set;
#else
    // 空
#endif
    if(chassis_move_control->leg_length_set > LEG_LENGTH_MAX)
        chassis_move_control->leg_length_set = LEG_LENGTH_MAX;
    else if(chassis_move_control->leg_length_set < LEG_LENGTH_MIN)
        chassis_move_control->leg_length_set = LEG_LENGTH_MIN;

    if(chassis_move_control->leg_angle_set > PI * 2 / 3)
        chassis_move_control->leg_angle_set = PI * 2 / 3;
    else if(chassis_move_control->leg_angle_set < PI / 3)
        chassis_move_control->leg_angle_set = PI / 3;
}

/**
  * @brief          遥控器的数据处理成底盘的前进vx速度，vy速度
  * @author         pxx
  * @param          vx_set  x轴前进速度设置，m/s
  * @param          vy_set  y轴前进速度设置，m/s
  * @param          chassis_move_rc_to_vector   底盘结构体指针
  * @retval         void
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel;
    fp32 vx_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
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
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    if(chassis_move_control_loop->chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_control_loop->right_leg.front_joint.give_current = 0;
        chassis_move_control_loop->right_leg.back_joint.give_current = 0;
        chassis_move_control_loop->left_leg.front_joint.give_current = 0;
        chassis_move_control_loop->left_leg.back_joint.give_current = 0;
        return;
    }

    fp32 l_force = 0.0f, l_torque = 0.0f;
    fp32 r_force = 0.0f, r_torque = 0.0f;
    fp32 err_tor = 0.0f;
#if defined(LEG_POS_TEST_START)
    r_torque = PID_Calc(&chassis_move_control_loop->right_leg_angle_pid, chassis_move_control_loop->leg_angle_set, chassis_move_control_loop->right_leg.leg_angle);
    l_torque = PID_Calc(&chassis_move_control_loop->left_leg_angle_pid, chassis_move_control_loop->leg_angle_set, chassis_move_control_loop->left_leg.leg_angle);
    // l_torque = PID_Calc(&chassis_move_control_loop->left_leg_angle_pid, PI / 2, chassis_move_control_loop->left_leg.leg_angle);
#endif
    r_force = PID_Calc(&chassis_move_control_loop->right_leg_length_pid, chassis_move_control_loop->leg_length_set, chassis_move_control_loop->right_leg.leg_length);
    l_force = PID_Calc(&chassis_move_control_loop->left_leg_length_pid, chassis_move_control_loop->leg_length_set, chassis_move_control_loop->left_leg.leg_length);

    // 双腿角度误差控制
    err_tor = PID_Calc(&chassis_move_control_loop->angle_err_pid, 0, (chassis_move_control_loop->right_leg.leg_angle - chassis_move_control_loop->left_leg.leg_angle));

    // VMC 虚拟力解算
    fp32 tor_vector[2] = {0.0f};
    leg_conv(r_force, (-r_torque-err_tor), chassis_move_control_loop->right_leg.back_joint.angle, 
            chassis_move_control_loop->right_leg.front_joint.angle, tor_vector);
    chassis_move_control_loop->right_leg.back_joint.give_current = limitted_motor_current(tor_vector[1] * 1000, MAX_MOTOR_CAN_CURRENT);
    chassis_move_control_loop->right_leg.front_joint.give_current = limitted_motor_current(tor_vector[0] * 1000, MAX_MOTOR_CAN_CURRENT);
    leg_conv(l_force, (-l_torque+err_tor), chassis_move_control_loop->left_leg.back_joint.angle, 
            chassis_move_control_loop->left_leg.front_joint.angle, tor_vector);
    chassis_move_control_loop->left_leg.back_joint.give_current = limitted_motor_current(-tor_vector[1] * 1000, MAX_MOTOR_CAN_CURRENT);
    chassis_move_control_loop->left_leg.front_joint.give_current = limitted_motor_current(-tor_vector[0] * 1000, MAX_MOTOR_CAN_CURRENT);
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

