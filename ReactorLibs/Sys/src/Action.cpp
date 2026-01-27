#include "Action.hpp"
#include "bsp_dwt.h"
// 全局唯一管理器变量
ActionManager Action;

void BaseAction::Start(uint32_t timeout)
{
    state = Action.RUNNING;
    timeout_ms = timeout;
    dwt_tick = DWT_GetTimeline_MSec();
    process = 0.0f;
}

void BaseAction::Update()
{
    // 若状态非运行中，直接返回
    if (state != Action.RUNNING) return;

    // 超时检查（通用逻辑）
    if (timeout_ms > 0 && (DWT_GetTimeline_MSec() - dwt_tick) > timeout_ms)
    {
        state = Action.FAILED;          // 判定任务超时失败
        OnTimeout();                    // 调用子类的超时
        return;                         // 结束返回
    }

    // 调用子类的具体更新逻辑
    if (OnUpdate())
    {
        state = Action.COMPLETED;
        OnComplete();           // 调用子类的完成
    }
}

void BaseAction::Cancel()
{
    if (state == Action.RUNNING) {
        state = Action.CANCELED;
        OnCancel();             // 调用子类的取消
    }
}

/**
 * @brief 普通的非阻塞式等待
 * @param ms 等待时间，单位毫秒
 */
void ActionManager::Wait(uint32_t ms, bool* blocked, uint32_t* seq_tick)
{
    // 记录起始时间
    if (*seq_tick == 0)     *seq_tick = DWT_GetTimeline_MSec();
    
    // 检查是否达到等待时间
    if ((DWT_GetTimeline_MSec() - *seq_tick) >= ms)
    {
        *seq_tick = 0;      // 重置时间记录
        if (blocked) *blocked = false;   // 解除阻塞
    }
    else
    {
        if (blocked) *blocked = true;    // 设置阻塞
    }
}


void ActionManager::WaitUntil(bool condition, bool* blocked, uint32_t* seq_tick, uint32_t timeout_ms)
{
    // 记录起始时间
    if (*seq_tick == 0)     *seq_tick = DWT_GetTimeline_MSec();
    
    // 检查条件是否满足
    if (condition)
    {
        *seq_tick = 0;
        if (blocked) *blocked = false;
    }
    // 检查超时
    else if (timeout_ms > 0 && (DWT_GetTimeline_MSec() - *seq_tick) >= timeout_ms)
    {
        *seq_tick = 0;
        if (blocked) *blocked = false;
    }
    // 保持阻塞
    else
    {
        if (blocked) *blocked = true;
    }
}




/// @brief 抛出一个持续（异步）动作
/// @param hact 
/// @param timeout_ms 
BaseAction* ActionManager::LaunchAsync(BaseAction* hact, uint32_t timeout_ms)
{
    if (hact == nullptr)    return nullptr;        // 预防空指针

    // 首先将动作加入运行列表
    bool slot_found = false;
    for (int i = 0; i < 12; i++)
    {
        if (!RunningActionFlags[i])        // 找到一个空闲槽位
        {
            RunningActions[i] = hact;       // 分配给动作
            RunningActionFlags[i] = true;
            slot_found = true;
            break;
        }
    }
    
    // 没有空闲槽位，无法启动新动作
    if (!slot_found)    return nullptr;

    // 启动动作
    hact->Start(timeout_ms);

    return hact;
}


/**
 * @name ExecutorRun
 * @brief 持续 追踪/执行 抛出的动作
 */
void ActionManager::ExecutorRun()
{
    for (int i = 0; i < 12; i++)
    {
        // 如果该槽位有动作
        if (RunningActionFlags[i])
        {
            BaseAction* act = RunningActions[i];

            // 每周期更新动作状态
            act->Update();

            // 检查动作是否已结束
            if (act->GetState() == Action.COMPLETED ||
                act->GetState() == Action.FAILED ||
                act->GetState() == Action.CANCELED)
            {
                // 清理动作槽位
                RunningActions[i] = nullptr;
                RunningActionFlags[i] = false;
            }
        }
    }
}