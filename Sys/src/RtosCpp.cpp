#include "RtosCpp.hpp"
#include "std_cpp.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "motor_dji.hpp"
#include "System.hpp"
#include "Action.hpp"
#include "RtosCpp.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"

/**
 * @brief 预初始化函数
 * @warning 为什么要搞一个这个，而不是在RTOS启动的线程初始化呢
 * 主要是因为怕线程爆栈，主函数的栈深基本上摸不到底的
 */
void InitializeCpp()
{
    System.Init();
    MainFrameCpp();
}


/**
 * @brief   机器人应用层任务
 * @note    200Hz运行
 */
void ApplicationCpp()   
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {   
        // 更新所有应用
        System._Update_Applications();
        
        /***    最大循环频率：200Hz     ***/
        osDelayUntil(&AppTick, 5);    // 200Hz
    }
    
}



/**
 * @brief       机器人系统主进程
 * @note        负载 `较低`，以200Hz运行，自行分频
 * @warning     这意味着，系统无法分辨200Hz以上的事件
 */
void RobotSystemCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {
        // 维护DWT计时器
        DWT_CntUpdate();
        System.Run();
        
        /***    最大循环频率：200Hz     ***/
        osDelayUntil(&AppTick, 5);
    }
}



/**
 * @brief   机器人的低频循环任务
 * @warning 只承担控制类任务而非逻辑类任务
 * @note    负载 `较低`，以200Hz运行，自行分频
 */
void StateCoreCpp()
{
    uint32_t AppTick = xTaskGetTickCount();
    StateCore& core = StateCore::GetInstance();
    while (1)
    {
        core.Run();
        /***    最大循环频率：250Hz     ***/
        osDelayUntil(&AppTick, 4);
    }
}



/**
 * @brief   机器人的高频循环任务
 * @warning 只承担控制类任务而非逻辑类任务
 * @note    负载 `最高`，以最高1000Hz运行，自行分频
 */
void ControlCpp()
{
    while (1)
    {
        MotorDJI::ControlAllMotors();
        /***    最大循环频率：1000Hz     ***/
        osDelay(1);     // FreeRTOS的极限，1ms喂狗
    }
}