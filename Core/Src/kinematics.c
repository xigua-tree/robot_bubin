#include "kinematics.h"
#include <math.h>

/**
 * @brief  全向轮运动学解算
 * @note   轮子编号与底盘布局对应关系（俯视，x前 y左）：
 *         轮1: 右前, 轮2: 左前, 轮3: 左后, 轮4: 右后
 * @param  Vx   X 方向速度（正值=前进）
 * @param  Vy   Y 方向速度（正值=左移）
 * @param  omega 旋转角速度（正值=逆时针）
 * @param  wheel_rpm[4]  输出的 4 轮目标转速 (RPM)
 */
void OmniKinematics(int Vx, int Vy, int omega, float wheel_rpm[4])
{
    /* 全向轮速度分解公式 */
    wheel_rpm[2] = (float)( -Vx + Vy - omega * (LX + LY));  /* 轮1: 右前 */
    wheel_rpm[0] = (float)(  Vx + Vy - omega * (LX + LY));  /* 轮2: 左前 */
    wheel_rpm[1] = (float)(  Vx - Vy - omega * (LX + LY));  /* 轮3: 左后 */
    wheel_rpm[3] = (float)( -Vx - Vy - omega * (LX + LY));  /* 轮4: 右后 */

    /* 限幅 */
    for (int i = 0; i < 4; i++) {
        if (wheel_rpm[i] > MAX_SPEED) {
            wheel_rpm[i] = MAX_SPEED;
        } else if (wheel_rpm[i] < -MAX_SPEED) {
            wheel_rpm[i] = -MAX_SPEED;
        }
    }
}

/**
 * @brief  角度归一化到 [-PI, PI] 范围
 */
float normalizeAngleRad(float angle)
{
    float two_pi = PI_2;

    angle = fmodf(angle, two_pi);
    if (angle > PI)
        angle -= two_pi;
    else if (angle < -PI)
        angle += two_pi;

    return angle;
}
