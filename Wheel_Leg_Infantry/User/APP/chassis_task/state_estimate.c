#include "state_estimate.h"
#include "arm_math.h"
#include "user_lib.h"
#include "INS_task.h"

/**
 * @brief 底盘为右手系
 *
 *           ^ x     左轮     右轮
 *     y     |        |      |  前
 *    < _____|        |------|
 *    z 轴从屏幕向外   |      |  后
 */

#define VEL_PROCESS_NOISE 50
#define VEL_MEASURE_NOISE 2500

/**
  * @brief          使用卡尔曼滤波估计底盘速度
  * @author         pxx
  * @param[in]      底盘控制结构体
  * @retval         返回空
  */
void SpeedEstimation(chassis_move_t *chassis_estimate)
{
    // 根据腿部运动学修正轮速
    float motor_left = -chassis_estimate->left_leg.wheel_motor.wheel_motor_measure->speed_rpm * MOTOR_RPM_TO_ROTATE 
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
    // float body_dx = (chassis_estimate->right_leg.wheel_motor.speed + chassis_estimate->left_leg.wheel_motor.speed) * 0.5f;

    // TODO: 此处还应该计算陀螺仪相对旋转中心的偏移所带来的误差

    // 计算机体水平方向加速度
    float acc_x = *(chassis_estimate->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET) * arm_cos_f32(chassis_estimate->chassis_pitch)
                  - *(chassis_estimate->chassis_imu_accel + INS_ACCEL_Z_ADDRESS_OFFSET) * arm_sin_f32(chassis_estimate->chassis_pitch)
                  - 0.22f;// 由于y轴安装误差导致的固定偏置


    // 卡尔曼滤波变量
    static float estimate_vel = 0;
    static float estimate_cov = 1;
    float kalman_gain;
    float measure_val;
    float dt = CHASSIS_CONTROL_TIME;

    // 先验估计（预测步骤）
    float prior_vel = estimate_vel + acc_x * dt; // 加速度积分预测速度
    float prior_cov = estimate_cov + VEL_PROCESS_NOISE * dt; // 预测协方差增加过程噪声

    // 测量步骤
    measure_val = body_dx; // 测量值为计算得到的机体x方向速度

    // 卡尔曼增益计算
    kalman_gain = prior_cov / (prior_cov + VEL_MEASURE_NOISE); // 计算卡尔曼增益

    // 后验估计（更新步骤）
    estimate_vel = prior_vel + kalman_gain * (measure_val - prior_vel); // 更新估计速度
    estimate_cov = (1 - kalman_gain) * prior_cov; // 更新估计协方差
    loop_fp32_constrain(estimate_cov, 0.01f, 100.0f);       // 协方差限幅

    // 更新估计结果
    chassis_estimate->estimated_speed = estimate_vel;
}
