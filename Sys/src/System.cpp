#include "System.hpp"
#include "bsp_dwt.h"
#include "msg_coder.hpp"
#include "Chassis.hpp"
#include "led_ws2812.hpp"
#include "Monitor.hpp"
#include "farcon.hpp"
SystemType& System = SystemType::GetInstance();
LedWs2812 sys_ledband;
Farcon farcon;

void SystemType::Init(bool Sc)
{
    // 初始化DWT计时器
    DWT_Init(CPU_HERT_A_BOARD_MHZ);

    // 初始化Monitor监视器
    Monitor::GetInstance().Init(&huart2, nullptr, false);

    // 初始化系统灯带
    sys_ledband.Init(&htim5, TIM_CHANNEL_4, 13);
    // 颜色偏置因子（用于校正颜色）
    sys_ledband.BiasFactor = Vec3(0.843f, 1.0f, 0.843f); 

    // 里程计初始化
    odometer.Init(&huart6, true, false, false, true);
    
    // 遥控器初始化
    farcon.init(&huart3);
    
    // 自动开始自检
    if (Sc) status = Systems::SELF_CHECK;
}

/**
 * @brief 运行机器人系统主进程
 * @note 该方法应被周期性调用，以处理系统任务
 */
void SystemType::Run()
{
    // 自检
    _Update_SelfCheck();

    /*--<       正式运行        >--*/

    // 管理 主灯带 状态（50Hz分频）
    _Update_LedBand();

    // 提供位置
    if (pos_source != nullptr)
    {
        position = *pos_source;
    }

    // 更新全局时间
    runtime_tick = DWT_GetTimeline_Sec();

    static int temp_cnt = 0;
    if (temp_cnt++ >= 1)
    {
        Monitor::GetInstance().LogTrack();
        temp_cnt = 0;
    }
}

void SystemType::_Update_LedBand()
{
    static uint32_t prescaler_cnt = 0;
    prescaler_cnt++;

    if (prescaler_cnt >= ledband_prescaler)
    {
        prescaler_cnt = 0;

        if (Display.display_overlay)
        {
            // 覆盖显示接口
            _LedBandDisplayControl();
        }
        else
        {
            // 根据不同的要求来控制系统灯
            _LedBandControl();
        }

        sys_ledband.Upload();
    }
}



/**
 * @brief 自检（检查所有被Monitor给Watch的变量是否正常）
 */
void SystemType::_Update_SelfCheck()
{
    static uint16_t check_cnt = 0;
    static bool error_list[24] = {false};
    static bool warning_list[24] = {false};

    if (start_selfcheck_flag && status == Systems::ORIGIN)
    {
        start_selfcheck_flag = false;
        status = Systems::SELF_CHECK;
    }

    // 处于自检状态时，进行自检
    if (status != Systems::SELF_CHECK)   return;

    /**     确保所有关键Watch在一秒内都持续为使能状态   **/
    for (uint8_t i = 0; i < monit.watch_count; i++) 
    {
        if(*monit.watch_buf[i].targ != true && check_cnt > 100)    // 遇到有不在线的
        {
            // 判断其严重程度
            if (monit.watch_buf[i].is_neccessary)   error_list[i] = true;
            else    warning_list[i] = true;
        }
    }

    check_cnt++;
    if (check_cnt >= 300)   // 先沉默半秒，再持续一秒，总共1.5秒
    {
        // 检查是否有错误
        bool have_error = false;
        for (uint8_t i = 0; i < monit.watch_count; i++)
        {
            if (error_list[i])
            {
                // 严重错误，直接报错
                monit.LogError("Offline: %s\n", monit.watch_buf[i].warning_info);
                have_error = true;
            }
            else if (warning_list[i])
            {
                // 一般错误，记录警告
                monit.LogWarning("Offline: %s\n", monit.watch_buf[i].warning_info);
            }
        }

        if (have_error)
        {
            // 返回初始状态，看有没有机会修好
            status = Systems::ORIGIN;
            check_cnt = 0;

            // 重置错误列表
            memset(error_list, 0, sizeof(error_list));
            memset(warning_list, 0, sizeof(warning_list));
            return;
        }
        else
        {
            // 自检完成，进入READY状态
            memset(error_list, 0, sizeof(error_list));
            memset(warning_list, 0, sizeof(warning_list));
            status = Systems::READY;
            monit.Log("System Self-Check Passed!\n");
            check_cnt = 0;
        }
    }
}



/**
 * @brief 默认情况下，系统的灯带控制逻辑
 * @note 可被Display接口覆盖
 */
void SystemType::_LedBandControl()
{
    switch (status)
    {
        case Systems::ORIGIN:
        {
            // 正常：呼吸白灯
            if (!is_retrying)
                sys_ledband.Breath(Color(255.0f, 255.0f, 255.0f), 1.0f);
            // 重试：呼吸橙灯
            else
                sys_ledband.Breath(Color(255.0f, 127.0f, 0.0f), 1.0f);
            break;
        }
        case Systems::SELF_CHECK:
        {
            // 正常自检中：白色滚动
            if (!is_retrying)
                sys_ledband.Running(Color(255.0f, 255.0f, 255.0f), 0.2, 0.3);
            // 重试自检中：橙色滚动
            else
                sys_ledband.GradientFlow(Color(255.0f, 127.0f, 0.0f), Color(0, 0, 0), 0.3);
            break;
        }
        case Systems::READY:
        {
            // 准备好了：呼吸阵营灯
            if (camp == Systems::Camp_Blue)
                sys_ledband.Breath(Color(0.0f, 0.0f, 255.0f), 1.0f);
            else
                sys_ledband.Breath(Color(255.0f, 0.0f, 0.0f), 1.0f);
            
            break;
        }
        case Systems::WORKING:
        {
            // 分红蓝区
            // 蓝区：常亮蓝灯
            if (camp == Systems::Camp_Blue)     sys_ledband.Lit(Color(0.0f, 0.0f, 255.0f));
            // 红区：常亮红灯
            else    sys_ledband.Lit(Color(255.0f, 0.0f, 0.0f));
            break;
        }
    }
}












/*****--<       灯带显示接口        >--*****/
/**
 * @brief 系统灯带显示控制接口（可覆盖常态显示）
 */
void SystemType::_LedBandDisplayControl()
{
    switch (Display.display_type)
    {
        case SystemType::_LedDisplayAPI::SysLEDDisp_WarningBlink:
        {
            // 闪烁逻辑（在周期的前一半亮，后一半灭）
            if ((Display.blink_cnt * 5) % (Display.blink_interval) < (Display.blink_interval / 2))
            {
                sys_ledband.Lit(Color(255.0f, 255.0f, 0.0f));   // 黄色
            }
            else
            {
                sys_ledband.Lit(Color(0.0f, 0.0f, 0.0f));       // 熄灭
            }
            Display.blink_cnt++;
            break;
        }
        case SystemType::_LedDisplayAPI::SysLEDDisp_ErrorBlink:
        {
            // 闪烁逻辑（在周期的前一半亮，后一半灭）
            if ((Display.blink_cnt * 5) % (Display.blink_interval) < (Display.blink_interval / 2))
            {
                sys_ledband.Lit(Color(255.0f, 255.0f, 0.0f));   // 紫色
            }
            else
            {
                sys_ledband.Lit(Color(0.0f, 0.0f, 0.0f));     // 熄灭
            }
            Display.blink_cnt++;
            break;
        }
    }

    // 检查是否完成，如果闪烁时间大于要求次数乘以间隔时间，则结束
    if (Display.blink_cnt * 5 >= Display.blink_times * Display.blink_interval)
    {
        // 结束覆盖显示
        Display.display_overlay = false;
        Display.display_type = SystemType::_LedDisplayAPI::SysLEDDisp_None;
        Display.blink_cnt = 0;
    }
}

/**
 * @brief 警告闪烁LED
 */
void SystemType::_LedDisplayAPI::WarningBlink(uint8_t times, uint16_t interval)
{
    // 正在执行的话，就直接退出
    if (display_overlay) return;
    // 启用覆盖显示
    display_overlay = true;

    // 执行闪烁
    display_type = SysLEDDisp_WarningBlink;
    
    // 设置参数
    blink_times = times;
    blink_interval = interval;
    blink_cnt = 0;
}


/**
 * @brief 
 */
void SystemType::SetPositionSource(Vec3& pos)
{
    pos_source = &pos;
}

template <typename T>
T *SystemType::FindApp(const char *name)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] != nullptr)
        {
            // 首先对比类型
            if (typeid(app_list[i]->GetType()) == typeid(T))
            {
                // 再对比名字
                if (strncmp(app_list[i]->name, name, 24) == 0)
                {
                    return dynamic_cast<T *>(app_list[i]);
                }
            }
        }
    }
    return nullptr;
}

bool SystemType::RegistApp(Application &app_inst)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] == nullptr)
        {
            // 添加到应用实例列表中
            app_list[i] = &app_inst;

            // 运行其Start函数
            app_inst.Start();
            return true;
        }
    }
    return false;
}

/**
 * @brief 运行所有应用实例
 */
void SystemType::_Update_Applications()
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] != nullptr)
        {
            if (app_list[i]->CntFull())
            {
                app_list[i]->Update();
            }
        }
    }
}


/**
 * @brief 监测预分频计数器是否满了
 */
bool Application::CntFull()
{
    _prescaler_cnt++;
    if (_prescaler_cnt >= prescaler)
    {
        _prescaler_cnt = 0;
        return true;
    }
    return false;
}


