#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"
#include "bsp_log.h"

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
    // 配置状态图为简并模式
    example_graph.Degenerate(Action_of_Dege);
    
    // 向状态机核心注册
    core.RegistGraph(example_graph);
    core.Enable(0);         // 启动状态机核心，指定初始状态图为0号图

    test_motor_0.Init(&hcan1, 1, PID_PosControl);
    test_motor_0.speed_pid.Init(10.0f, 0.0f, 0.0f);
    test_motor_0.position_pid.Init(0.035f, 0.0f, 0.0f);

    // test_motor_0.motor_adrc.Init(ADRC::Sec_Ord, 15.0f, 2.7f, J, B, K_t, dt, 18.0f);
    // test_motor_0.g_Identifier.b0_ = K_t / J;

    // test_motor_0.Dynamicle(Fluid);                    // 启动电机的在线辨识
    
    // 配置跟踪器
    monit.Track(test_motor_0.motor_adrc.debug_current);      // 速度跟踪曲线
    monit.Track(test_motor_0.motor_adrc.debug_TL);

    monit.Track(test_motor_0.motor_adrc.debug_omega);        // 位置跟踪曲线
    monit.Track(test_motor_0.measure.total_angle);

    monit.Track(test_motor_0.g_Identifier.rho_ru);                 // 电机的转动惯量
    monit.Track(test_motor_0.g_Identifier.J_hat_);

    monit.Perflize();  // 切换高性能模式
}

void Action_of_Dege(StateCore* core)
{
    
    Seq::WaitUntil([](){
        return test_motor_0.enabled;
    });
    Seq::Wait(1.5f); // 等待1.5秒

    test_motor_0.SetPos(30000.0f); // 设置目标速度3000RPM

    Seq::Wait(4.0f); // 等待4秒

    test_motor_0.SetPos(0.0f); // 设置目标速度0RPM

    Seq::Wait(8.0f); // 等待8秒
}