#include "state_estimate.h"
#include "chassis_task.h"
#include "user_lib.h"

/**
 * @brief 底盘为右手系
 *
 *           ^ x     左轮     右轮
 *     y     |        |      |  前
 *    < _____|        |------|
 *    z 轴从屏幕向外   |      |  后
 */



/**
  * @brief          使用卡尔曼滤波估计底盘速度
  * @author         pxx
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
void SpeedEstimation(chassis_move_t *chassis_estimate)
{
    // 根据腿部运动学修正轮速
    float motor_left = chassis_estimate->left_leg.wheel_motor.wheel_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE 
                        + chassis_estimate->left_leg.angle_dot 
                        + *(chassis_estimate->chassis_imu_gyro + INS_GYRO_Y_ADDRESS_OFFSET);
    float motor_right = chassis_estimate->right_leg.wheel_motor.wheel_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE 
                        + chassis_estimate->right_leg.angle_dot 
                        + *(chassis_estimate->chassis_imu_gyro + INS_GYRO_Y_ADDRESS_OFFSET);

    // 计算机体x方向速度，由于腿部角度变化导致的相对速度使用线速度代替，轮半径0.03
    float body_dx_left = motor_left * 0.03f 
                        + chassis_estimate->left_leg.angle_dot * chassis_estimate->left_leg.leg_length
                        + chassis_estimate->left_leg.length_dot * arm_sin_f32(chassis_estimate->left_leg.leg_angle - PI/2);
    float body_dx_right = motor_right * 0.03f 
                        + chassis_estimate->right_leg.angle_dot * chassis_estimate->right_leg.leg_length
                        + chassis_estimate->right_leg.length_dot * arm_sin_f32(chassis_estimate->right_leg.leg_angle - PI/2);
    float body_dx = (body_dx_left + body_dx_right) / 2;

    // 计算机体旋转导致的向心加速度和角加速度
}