#include "bsp_tim_pwm.h"


static float GetFreq(struct BspTIMPWM_t pwm_inst)
{
    // 计算PWM频率 = 定时器时钟频率 / (ARR + 1)
    // 假设使用APB1TimerClock作为定时器时钟（考虑分频）
    uint32_t timer_clock_freq = HAL_RCC_GetPCLK1Freq() * 2 / (pwm_inst.htim->Instance->PSC + 1);   

    float pwm_freq = (float)timer_clock_freq / (pwm_inst.auto_reload_value + 1);
    return pwm_freq;
}


void BspTIMPWM_InstRegist(BspTIMPWM_TypeDef *tim_inst, TIM_HandleTypeDef *htim, uint32_t channel)
{
    tim_inst->htim = htim;
    tim_inst->channel = channel;

    // 获取ARR的值
    tim_inst->auto_reload_value = __HAL_TIM_GET_AUTORELOAD(tim_inst->htim);
    // 获取CCR的值
    tim_inst->compare_value = __HAL_TIM_GET_COMPARE(tim_inst->htim, tim_inst->channel);
    // 给函数指针赋值
    tim_inst->GetFreq = GetFreq;

    tim_inst->freq = tim_inst->GetFreq(*tim_inst);
    // 设置初始占空比为 0
    BspTIMPWM_SetDuty(tim_inst, 0.0f);
}


void BspTIMPWM_SetDuty(BspTIMPWM_TypeDef *tim_inst, float duty)
{
    if(duty < 0.0f) duty = 0.0f;
    if(duty > 1.0f) duty = 1.0f;

    tim_inst->duty = duty;
    
    // 计算CCR的对应值
    tim_inst->compare_value = (uint32_t)(tim_inst->auto_reload_value * duty);
    __HAL_TIM_SET_COMPARE(tim_inst->htim, tim_inst->channel, tim_inst->compare_value);
}


void BspTIMPWM_Enable(BspTIMPWM_TypeDef *tim_inst)
{
    HAL_TIM_PWM_Start(tim_inst->htim, tim_inst->channel);
    tim_inst->enabled = 1;
}

void BspTIMPWM_Disable(BspTIMPWM_TypeDef *tim_inst)
{
    HAL_TIM_PWM_Stop(tim_inst->htim, tim_inst->channel);
    tim_inst->enabled = 0;
}
