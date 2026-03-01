#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"

ChassisType& chas = ChassisType::GetInstance();
StateGraph grp("Test");
StateCore& core = StateCore::GetInstance();

void DegeAct(StateCore* core);

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    System.monit.Watch({&chas.motors[0].online, "Motor0_Offline!", true});
    System.monit.Watch({&chas.motors[1].online, "Motor1_Offline!", true});
    System.monit.Watch({&chas.motors[2].online, "Motor2_Offline!", true});
    System.monit.Watch({&chas.motors[3].online, "Motor3_Offline!", true});

    System.monit.Track(chas.motors[3].measure.speed_rpm);
    System.monit.Track(chas.motors[3].targ_current);

    chas.Config(true);

    System.SetPositionSource(System.odometer.transform);

    System.RegistApp(chas);
    
//     grp.Degenerate(DegeAct);
//     core.RegistGraph(grp);
//     core.Enable(0);
}



void DegeAct(StateCore* core)
{
    Seq::WaitUntil(chas.enabled);

    // 先向前走2s
    chas.MoveAt(Vec2(1.0f, 0));
    chas.RotateAt(0);

    Seq::Wait(4.0f);
    // 再向左走2s
    chas.MoveAt(Vec2(0, 1.0f));
    chas.RotateAt(0);
    
    Seq::Wait(4.0f);
}
