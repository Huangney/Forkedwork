#include "bsp_halport.hpp"

namespace Hardware
{
    float MainFreq_MHz = 72.0f;

    bool RTTLogAtUart = false;

    /**-----    配置CAN总线     -----**/
    CAN_HandleTypeDef* hcan_main = nullptr;
    CAN_HandleTypeDef* hcan_sub = nullptr;

    /**-----    配置串口    -----**/
    UART_HandleTypeDef* huart_host = nullptr;
    UART_HandleTypeDef* huart_farcon = nullptr;
    UART_HandleTypeDef* huart_odom = nullptr;

    /**-----    配置定时器    -----**/
    TIM_HandleTypeDef* htim_led = nullptr;

    /**-----    配置 GPIO   -----**/
    
}