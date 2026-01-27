#include "bsp_gpio.h"
#include "main.h"
#include "gpio.h"

static BspGpio_Instance *bspgpio_insts[BSPGPIO_MAX_INSTS] = {NULL}; 
static int bspgpio_inst_count = 0; // 当前实例数量

///@brief 在cube中初始化后，注册一个GPIO实例
void BspGpio_InstRegist(BspGpio_Instance *inst,GPIO_TypeDef *GPIOx,uint32_t Pin)
{
    // 检验参数有效性
    if (inst == NULL || GPIOx == NULL)
    {
        return; //参数无效
    }
    // 将实例映射到已经初始化的GPIO
    inst->GPIO_Port = GPIOx;
    inst->Pin = Pin;

    // 记录实例
    if (bspgpio_inst_count < BSPGPIO_MAX_INSTS)
    {
        bspgpio_insts[bspgpio_inst_count++] = inst;    
    }

}

static BspGpio_ExtiHandler exti_handlers[16] = {NULL};  //将所有上层注册的中断处理函数存储在这里

///@brief 当收到中断时，通过快速索引数组可以获取对应的处理函数
static uint8_t BspGpio_GetPinIndex(uint16_t GPIO_Pin)
{
    // 检查是否有且仅传入一个引脚,原则上通过NVIC后在PR寄存器中只能有一个位被置位
    if (GPIO_Pin == 0 || (GPIO_Pin & (GPIO_Pin - 1)) != 0)  //因为寄存器中GPIO_Pin的值都是以位掩码存在的，如0x0001,0x0002,0x0004,0x0008...
    {
        return 16; // 无效索引
    }
    
    // 计算位的位置 (即EXTI0-15的索引)
    uint8_t index = 0;
    uint16_t temp_pin = GPIO_Pin;
    while (temp_pin > 1) // 循环直到找到设置的位
    {
        temp_pin >>= 1;
        index++;
    }
    return index;
}

void BspGpio_ExtiHandlerRegist(BspGpio_Instance *inst, BspGpio_ExtiHandler handler_func)
{
    uint8_t index = BspGpio_GetPinIndex(inst->GPIO_Port);
    
    // 检查索引是否在0-15范围内
    if (index < 16)
    {
        // 将函数指针存储到静态数组中
        exti_handlers[index] = handler_func;
    }
    
}
///@brief 回调函数的实函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t index = BspGpio_GetPinIndex(GPIO_Pin);
    if (index < 16)
    {
        BspGpio_ExtiHandler upper_handler = exti_handlers[index];
        
        if (upper_handler != NULL)
        {
            upper_handler(GPIO_Pin);
        }
    }
}


/******* 以下是对HAL库函数的复写，供上层使用的接口函数 *******/

uint32_t BspGpio_GetState(BspGpio_Instance *inst)
{
    return HAL_GPIO_ReadPin(inst->GPIO_Port, inst->Pin);
}

void BspGpio_SetState(BspGpio_Instance *inst, uint32_t PinState)
{
    HAL_GPIO_WritePin(inst->GPIO_Port, inst->Pin, PinState);
}

void BspGpio_TogglePin(BspGpio_Instance *inst)
{
    HAL_GPIO_TogglePin(inst->GPIO_Port, inst->Pin);
}

void BspGpio_LockPin(BspGpio_Instance *inst)
{
    HAL_GPIO_LockPin(inst->GPIO_Port, inst->Pin);
}
