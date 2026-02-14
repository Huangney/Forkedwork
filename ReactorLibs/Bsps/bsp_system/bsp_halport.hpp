#pragma once

#define USE_REAL_HAL



/* ==================================================================
 * 场景 1：在 Main 分支（真实战场），有 CubeMX 环境
 * ================================================================== */
#ifdef USE_REAL_HAL  // 这个宏可以在 main.h 或 编译选项里定义

    // 包含真的 HAL 库
    #include "main.h" 
    #include "can.h"
    #include "usart.h"
    #include "dma.h"

/* ==================================================================
 * 场景 2：在 Pure-Fram 分支（实验室），无硬件环境
 * ================================================================== */
#else 

    #include <stdint.h>
    #include <string.h> // for memset

    typedef void CAN_HandleTypeDef;
    typedef void CAN_RxHeaderTypeDef;
    typedef void CAN_FilterTypeDef;

    typedef void UART_HandleTypeDef;

    typedef void TIM_HandleTypeDef;

    typedef void GPIO_TypeDef;

    // 这种写法解决报错
    #define REACTOR_UART_SEND(...)      ((void)0) 
    #define REACTOR_CAN_FILTER_CFG(...) ((void)0)
    #define REACTOR_DMA_DISABLE_IT(...) ((void)0)


    // 补充缺失的枚举值
    #define DMA_IT_HT 0x01
    #define CAN_RX_FIFO0 0x00

    #ifndef __weak
    #if defined ( __GNUC__ )
        #define __weak   __attribute__((weak))
    #elif defined ( __CC_ARM )
        #define __weak   __weak
    #endif
    #endif

#endif 


namespace Hardware
{
    void Config_Hardwares();
    void Config_Parameters();

    extern float MainFreq_MHz;

    /// @note 如果本标志位被激活，日志会被同步发送到 UART_HOST
    extern bool RTTLogAtUart;
    
    /***---------------     框架CAN    ---------------***/
    /// @brief 框架所用CAN句柄
    extern CAN_HandleTypeDef* hcan_main;
    /// @brief 框架所用CAN句柄
    extern CAN_HandleTypeDef* hcan_sub;

    /***---------------     框架串口    ---------------***/
    /// @brief 工控机串口
    extern UART_HandleTypeDef* huart_host;
    /// @brief 遥控器串口
    extern UART_HandleTypeDef* huart_farcon;
    /// @brief 里程计串口
    extern UART_HandleTypeDef* huart_odom;
    /// @brief 其他串口
    extern UART_HandleTypeDef* huart_other;

    /***---------------     框架定时器    ---------------***/
    /// @brief WS2812灯带定时器
    extern TIM_HandleTypeDef* htim_led;
};
