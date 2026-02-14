#include "bsp_halport.hpp"

/**
 * @brief 硬件与框架的 映射链接处 
 * @note 请在此处映射所有的 硬件 到框架内部
 */
void Hardware::Config_Hardwares()
{
    /**-----    配置CAN总线     -----**/
    Hardware::hcan_main = &hcan1;
    Hardware::hcan_sub = nullptr;

    /**-----    配置串口    -----**/
    Hardware::huart_host = &huart1;
    Hardware::huart_farcon = nullptr;
    Hardware::huart_odom = nullptr;

    /**-----    配置定时器    -----**/
    Hardware::htim_led = nullptr;

    /**-----    配置 GPIO   -----**/

    Hardware::Config_Parameters();
}

/**
 * @brief 框架参数配置函数
 * @note 请在此处配置 所用硬件基底的参数
 */
void Hardware::Config_Parameters()
{
    /// @brief 配置主频
    Hardware::MainFreq_MHz = 168;

    /// @brief 日志输出模式
    Hardware::RTTLogAtUart = true;
}