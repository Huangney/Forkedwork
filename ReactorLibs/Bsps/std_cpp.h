#ifndef C_CPP_H
#define C_CPP_H

/**
 * @file C_Cpp.h
 * @author Huangney
 * @date 2025.9.6
 * @note 本文件用于作为C和C++代码的接口文件，所有要在C中调用的函数都放在此文件中
 */
#ifdef __cplusplus
extern "C" {
#endif

/******     RTOS函数        ******/
/// @brief 高频率的控制任务（100Hz ~ 1000Hz）
void ControlCpp();

/// @brief 低频率的控制任务（100Hz以下）
void StateCoreCpp();

/// @brief 应用管理进程
void ApplicationCpp();

/// @brief 机器人主任务
void RobotSystemCpp();

/// @brief 测试任务
void TestCpp();

/******     MAIN函数        ******/
/**
 * @brief 预初始化函数
 * @warning 为什么要搞一个这个，而不是在RTOS启动的线程初始化呢
 * 主要是因为怕线程爆栈，主函数的栈深基本上摸不到底的
 */
void Reactor46H_Initialize();


/******     IT函数        ******/

#ifdef __cplusplus
}
#endif

#endif