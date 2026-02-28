#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"
#include "bsp_log.hpp"

/**     测试用      **/
#include "signator.hpp"


StateCore& core = StateCore::GetInstance();
Monitor& monit = Monitor::GetInstance();


StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore* core);


// 测试框架用
MotorDJI test_motor_0;

float K_t = 0.01562f; // 转矩常数
float J = 0.000352925847;  // 转动惯量
float dt = 0.001f; // 采样时间间隔
float B = 1.56e-5f; // 阻尼系数


/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    // 配置电机控制器 (新 Fluent API 示例)
    test_motor_0.ConfigPID()
                .AsPosC()
                .SPD_PID(0.015f, 0.0001f, 0.0f)
                .SPD_Limit(2.4f, 20.0f)
                .POS_PID(0.5f, 0.0f, 0.0f)
                .POS_Limit(100.0f, 300.0f)
                .Apply();

    // 配置状态图为简并模式
    example_graph.Degenerate(Action_of_Dege);
    
    // 向状态机核心注册
    core.RegistGraph(example_graph);
    core.Enable(0);         // 启动状态机核心，指定初始状态图为0号图
}

void Action_of_Dege(StateCore* core)
{
    
    Seq::Wait(1.5f); // 等待1.5秒

    test_motor_0.SetPos(30000.0f); // 设置目标速度3000RPM

    Seq::Wait(4.0f); // 等待4秒

    test_motor_0.SetPos(0.0f); // 设置目标速度0RPM

    Seq::Wait(8.0f); // 等待8秒
}