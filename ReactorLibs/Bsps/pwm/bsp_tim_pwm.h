#ifndef BSP_TIM_PWM_H
#define BSP_TIM_PWM_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "tim.h"

typedef struct BspTIMPWM_t
{
    TIM_HandleTypeDef *htim;        // PWM定时器句柄
    uint32_t channel;               // PWM通道
    float freq;                     // PWM频率
    uint8_t enabled;                // PWM使能标志

    uint32_t auto_reload_value;     // 自动重装载寄存器的值（ARR）
    uint32_t compare_value;         // 比较寄存器的值（CCR）

    float duty;                     // PWM占空比
    float (*GetFreq)(struct BspTIMPWM_t pwm_inst);             // 获取PWM频率的函数指针    
}BspTIMPWM_TypeDef;


void BspTIMPWM_InstRegist(BspTIMPWM_TypeDef *tim_inst, TIM_HandleTypeDef *htim, uint32_t channel);

void BspTIMPWM_SetDuty(BspTIMPWM_TypeDef *tim_inst, float duty);
void BspTIMPWM_Enable(BspTIMPWM_TypeDef *tim_inst);
void BspTIMPWM_Disable(BspTIMPWM_TypeDef *tim_inst);

#ifdef __cplusplus
}
#endif

#endif