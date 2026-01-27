#include "servo.hpp"
#include "arm_math.h"

void Servo::Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    BspTIMPWM_InstRegist(&pwm_inst, htim, channel);
    initialized = true;
    
}

void Servo::Enable()
{
    if (!initialized) return;

    // 确认一下频率等于50Hz，舵机一般都是这个频率
    if (fabs(pwm_inst.GetFreq(pwm_inst) - 50.0f) > 0.1f) return;
    
    enabled = true;
    BspTIMPWM_Enable(&pwm_inst);
}

void Servo::Disable()
{
    if (!initialized) return;
    
    enabled = false;
    BspTIMPWM_Disable(&pwm_inst);
}


void Servo::SetAngle(float ang)
{
    if (!initialized) return;
    
    // 假设舵机的工作范围是-90到90度，对应的PWM占空比是2.5%到12.5%
    if (ang < -90.0f) ang = -90.0f;
    if (ang > 90.0f) ang = 90.0f;
    
    // 给类成员赋值
    angle = ang;

    // 通过线性映射计算占空比
    float duty = (angle + 90.0f) / 180.0f * (0.125f - 0.025f) + 0.025f;
    BspTIMPWM_SetDuty(&pwm_inst, duty);
}








