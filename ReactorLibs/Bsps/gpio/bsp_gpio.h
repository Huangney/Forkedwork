#ifndef BSP_GPIO_H
#define BSP_GPIO_H
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "gpio.h"

#define BSPGPIO_MAX_INSTS 140 // 最多支持140个GPIO实例


/// @brief BSPGPIO实例定义
typedef struct BspGpio_Instance_t
{
    GPIO_TypeDef *GPIO_Port;     //GPIO初始化的端口
    uint32_t Pin;                //引脚号

} BspGpio_Instance;


/****** 函数 ******/
void BspGpio_InstRegist(BspGpio_Instance *inst, GPIO_TypeDef *GPIO_Init,uint32_t Pin); 


uint32_t BspGpio_GetState(BspGpio_Instance *inst);
void BspGpio_SetState(BspGpio_Instance *inst, uint32_t PinState);
void BspGpio_TogglePin(BspGpio_Instance *inst);
void BspGpio_LockPin(BspGpio_Instance *inst);

typedef void (*BspGpio_ExtiHandler)(uint16_t pin);

// BSP层定义一个注册函数接口，上层应用通过这个函数将自己的逻辑函数注册进来
void BspGpio_ExtiHandlerRegist(BspGpio_Instance *inst, BspGpio_ExtiHandler handler_func);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static uint8_t BspGpio_GetPinIndex(uint16_t GPIO_Pin);



#ifdef __cplusplus
}
#endif
#endif
