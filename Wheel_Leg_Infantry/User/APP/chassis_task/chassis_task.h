/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  v2.0.0     Nov-05-2023     pxx             刚起步
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "main.h"
#include "remote_control.h"
#include "pid.h"
#include "CAN_Receive.h"
#include "user_lib.h"

#include "pid.h"

/*底盘CAN_ID
  4     3

  1     2
*/

// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

// 前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
// 旋转的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 2

// 选择普通底盘状态 开关通道号
#define MODE_CHANNEL 0
// 遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
// 跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
// 不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.007f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

// 遥控遥感死区限制
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.5f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.5f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

// 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
// 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
// 底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
// 底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//电机码盘值最大以及中值
#define Half_joint_ecd_range 4096
#define joint_ecd_range 8191
// m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
//  #define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f  // 2PI/60/(3591/187) * (0.1524/2)
#define M3508_MOTOR_RPM_TO_VECTOR 0.000831619497806989034418f // 2PI/60/(3591/187)*2 * (0.1524/2)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define MOTOR_RPM_TO_ROTATE 0.10471975512f // 2PI /60

// 底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
// 底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 4.0f
// 底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.9f
// 底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.0f

// 腿部初始长度
#define LEG_LENGTH_INIT 0.10f

#define LEG_LENGTH_MAX 0.14f
#define LEG_LENGTH_MIN 0.06f

//电机编码值转化成角度值
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192

// 腿部长度控制PID
#define LEG_LENGTH_PID_KP 6000.0f
#define LEG_LENGTH_PID_KI 20.0f
#define LEG_LENGTH_PID_KD 80.0f
#define LEG_LENGTH_PID_MAX_OUT 500.0f
#define LEG_LENGTH_PID_MAX_IOUT 200.0f

// 腿部角度控制PID
#define LEG_ANGLE_PID_KP 30.0f
#define LEG_ANGLE_PID_KI 0.0f
#define LEG_ANGLE_PID_KD 12.0f
#define LEG_ANGLE_PID_MAX_OUT 100.0f
#define LEG_ANGLE_PID_MAX_IOUT 20.0f

// 腿部误差控制PID
#define ANGLE_ERR_PID_KP 60.0f
#define ANGLE_ERR_PID_KI 0.0f
#define ANGLE_ERR_PID_KD 0.0f
#define ANGLE_ERR_PID_MAX_OUT 30.0f
#define ANGLE_ERR_PID_MAX_IOUT 0.0f

// 底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 8.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.15f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
	CHASSIS_FORCE_RAW,				  // 底盘开环控制
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW, // 底盘跟随云台
	CHASSIS_VECTOR_NO_FOLLOW_YAW,	  // 底盘不跟随云台

	CHASSIS_POSITION_LEG			  // 底盘腿部控制
} chassis_mode_e;

typedef struct
{
	const motor_measure_t *wheel_motor_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
} Chassis_Motor_t; // 驱动轮电机结构体

typedef struct
{
	const motor_measure_t *joint_motor_measure;
	int16_t offset_ecd;	    // 关节电机中值
	fp32 angle;				// 关节角度
	fp32 angle_dot;			// 关节转动速度
	fp32 current_set;		// 关节电机力矩，由PID计算给出
	int16_t give_current;	// 实际由CAN通信给电调发送的电流值
} Joint_Motor_t; // 关节电机结构体

typedef struct
{
	Chassis_Motor_t wheel_motor;
	Joint_Motor_t front_joint;
	Joint_Motor_t back_joint;

	fp32 leg_length;    		   // 摆杆长度
	fp32 leg_angle;    		   // 摆杆与竖直方向的夹角
	fp32 angle_dot;   		   // 腿部摆杆的旋转速度
	fp32 length_dot;   		   // 腿部摆杆的旋转速度

	fp32 virtual_pole_force;   // 腿部五连杆机构的推力
	fp32 virtual_pole_torque;  // 沿中心轴的力矩

	fp32 front_joint_tor;     // 前关节电机VMC期望扭矩
	fp32 back_joint_tor;      // 后关节电机VMC期望扭矩
} Leg_Control_t;

typedef struct 
{
	fp32 theta;			// 摆杆与竖直方向的夹角
	fp32 theta_dot;		
	fp32 x;				// 驱动轮位移
	fp32 x_dot;
	fp32 phi;			// 机体与水平方向夹角
	fp32 phi_dot;
} Robot_Statement_t;


typedef struct
{
	const RC_ctrl_t *chassis_RC;	  // 底盘使用的遥控器指针
	const fp32 *chassis_INS_angle;	  // 获取陀螺仪解算出的欧拉角指针
	const fp32 *chassis_imu_gyro;	  // 获取角加速度指针
	chassis_mode_e chassis_mode;	  // 底盘控制状态机
	chassis_mode_e last_chassis_mode; // 底盘上次控制状态机
	PidTypeDef chassis_angle_pid;	  // 底盘跟随角度pid

	Leg_Control_t left_leg;
	Leg_Control_t right_leg;
	PidTypeDef left_leg_length_pid;   //腿长控制器
	PidTypeDef right_leg_length_pid;  //腿长控制器
	PidTypeDef angle_err_pid;		  //双腿角度误差控制器

	PidTypeDef left_leg_angle_pid;    // 仅测试用
	PidTypeDef right_leg_angle_pid;   // 仅测试用

	Robot_Statement_t state_ref;	// 机器人状态量
	Robot_Statement_t state_set;	// 机器人预期的状态
	first_order_filter_type_t chassis_cmd_slow_set_vx; // vx一阶低通滤波

	fp32 leg_angle;
	fp32 leg_angle_set;
	fp32 leg_length;
	fp32 leg_length_set;
	fp32 leg_length_max;             // 腿部活动范围限制
	fp32 leg_length_min;             // 间接限制了关节电机的活动范围，关节电机还要有机械限位
	fp32 wz;						 // 底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 wz_set;					 // 底盘设定旋转角速度，逆时针为正 单位 rad/s
	fp32 chassis_relative_angle;	 // 底盘与云台的相对角度，单位 rad/s
	fp32 chassis_relative_angle_set; // 设置相对云台控制角度
	fp32 chassis_yaw_set;

	fp32 vx_max_speed;	// 前进方向最大速度 单位m/s
	fp32 vx_min_speed;	// 前进方向最小速度 单位m/s
	fp32 chassis_yaw;	// 陀螺仪和云台电机叠加的yaw角度
	fp32 chassis_pitch; // 陀螺仪和云台电机叠加的pitch角度
	fp32 chassis_roll;	// 陀螺仪和云台电机叠加的roll角度
} chassis_move_t;

extern void chassis_task(void *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

// 底盘初始化，主要是pid初始化
extern void chassis_init(chassis_move_t *chassis_move_init);
// 底盘状态机选择，通过遥控器的开关
extern void chassis_set_mode(chassis_move_t *chassis_move_mode);
// 底盘数据更新
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
// 底盘状态改变后处理控制量的改变static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
// 底盘设置根据遥控器控制量
extern void chassis_set_contorl(chassis_move_t *chassis_move_control);
// 底盘PID计算以及运动分解
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

// 获取底盘结构体指针
const chassis_move_t *get_chassis_control_point(void);
#endif
