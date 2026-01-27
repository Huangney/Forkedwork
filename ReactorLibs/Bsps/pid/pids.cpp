// filePath: pids.c

#include "pids.hpp"
#include "bsp_dwt.h"
#include "arm_math.h"

float limit_ab(float a, float limit)
{
    if (a > limit)
    {
        return limit;
    }
    else if (a < -limit)
    {
        return -limit;
    }
    else
    {
        return a;
    }
}

/**
 * @brief 完整初始化函数
 */
void PidGeneral::Init(float kp, float ki, float kd, int reverse)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    this->reverse = reverse;

    inte_errors = 0;
    last_error = 0;
    prev_error = 0;
    kd_error = 0;
    control_value = 0;
}

/**
 * @brief 启用增量PID模式
 * @param inner_acc 是否启用内部维护控制值
 */
void PidGeneral::IncreLize(bool inner_acc)
{
    InnerAcc = inner_acc;
    Incremental = true;
}

/**
 * @brief 启用前馈控制
 * @param fwd_type 前馈控制器类型
 * @param kf 前馈系数
 */
void PidGeneral::ForwardLize(Forward_Typedef fwd_type, float kf, float K, float Tc)
{
    this->fwd_type = fwd_type;
    this->Kf = kf;
    this->K = K;
    this->Tc = Tc;

    Feedforward = true;   
}


/**
 * @brief 手动设置时间间隔（同时禁用自动时间微分计算）
 */
void PidGeneral::ManualDt(float dt)
{
    delta_t = dt;
    AutoDt = false;    // 禁用自动时间间隔计算
}

/**
 * @brief 重设PID参数
 */
void PidGeneral::SetParam(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

/**
 * @brief 设置PID限幅
 */
void PidGeneral::SetLimit(float intelim, float outlim, float dfilter)
{
    inte_lim = intelim;
    out_lim = outlim;
    kd_filter_rate = dfilter;
}

/**
 * @brief 设置反向控制
 */
void PidGeneral::SetRev(bool reverse)
{
    this->reverse = reverse;
}

/**
 * @brief 设置死区控制
 */
void PidGeneral::SetDeadband(float start, float end)
{
    deadband_start = start;
    deadband_end = end;
    DeadbandEnabled = (end > start); // 若end > start（输入合法）则启用死区控制
}



/**
 * @brief 重置PID控制器的状态
 * @param targ_pid PID结构体指针
 */
void PidGeneral::Reset()
{
    inte_errors = 0;
    last_error = 0;
    prev_error = 0;
    kd_error = 0;
    control_value = 0;
}

/**
 * @brief 计算PID输出（供外部调用）
 * @note 根据配置的模式，调用不同的计算函数，输出结果也不同
 */
float PidGeneral::Calc(float targ, float real, float output_lim)
{
    float result;
     
    if (Incremental)
    {
        if (InnerAcc)           result = CalcIncAuto(targ, real, output_lim);      // 增量式PID，带内部累加
        else                    result = CalcInc(targ, real, output_lim);          // 增量式PID
    }
    else                    
    {
        result = CalcPos(targ, real, output_lim);      // 位置式PID
    }

    // 死区控制
    if (DeadbandEnabled)
    {
        if (fabs(error) < deadband_start) result = 0;
        else if (fabs(error) < deadband_end)
        {
            // 线性映射
            result = (fabs(error) - deadband_start) / (deadband_end - deadband_start) * result;
        } 
    }
    
    return result;
}


/**
 * @brief 计算位置式PID的输出
 * @param error 当前误差
 * @param output_lim 输出限幅（若为0则不限制）
 * @return PID控制输出 
 */
float PidGeneral::CalcPos(float targ, float real, float output_lim)
{
    float pid_output;
    error = targ - real;

    // 自适应时间间隔
    if (AutoDt) delta_t = DWT_GetDeltaTime(&dwt_dt);                    // 单位为秒
    if (delta_t > 10 * _delta_t_protect) delta_t = _delta_t_protect;         // 保护性Dt，防止频率过低导致计算错误
    // 根据Dt配置保护性Dt
    _delta_t_protect = 0.9f * _delta_t_protect + 0.1f * delta_t;

    // 为Kd计算经过一阶低通滤波的误差
    kd_error = kd_error + kd_filter_rate * (error - kd_error);
    
    // 累加积分项
    inte_errors += Ki * error * delta_t;

    // 如果用户配置了积分限幅，作限幅
    if (inte_lim > 0) inte_errors = limit_ab(inte_errors, inte_lim);

    // 计算普通PID
    pid_output = Kp * error + inte_errors + Kd * ((kd_error - last_kderr) / delta_t);

    // 记录误差项，用于下次计算
    prev_error = last_error;
    last_error = error;
    last_kderr = kd_error;

    // 记录前馈项
    u_prev_2 = u_prev;
    u_prev = u;
    u = targ;

    // 前馈控制
    if (Feedforward) switch (fwd_type)
    {
        case SpeedForward: pid_output = pid_output + FwdFuncs::SpdForward(u, u_prev, delta_t, K, Tc) * Kf; break;
        case PosForward:   pid_output = pid_output + FwdFuncs::PosForward(u, u_prev, u_prev_2, delta_t, K, Tc) * Kf; break;
        default: break;
    }

    // 如果用户配置了反向，作反向输出
    if (reverse) pid_output = -pid_output;
    
    // 应用输出限幅（如果配置了限幅）
    if (output_lim > 0) pid_output = limit_ab(pid_output, output_lim);

    // 应用内部输出限幅（如果配置了限幅）
    if (out_lim > 0)     pid_output = limit_ab(pid_output, out_lim);

    
    return pid_output;
}

/**
 * @brief 计算增量式PID的输出增量
 * @param targ_pid PID结构体指针
 * @param error 当前误差
 * @return 控制量的 `增量` 
 */
float PidGeneral::CalcInc(float targ, float real, float output_lim)
{
    error = targ - real;

    // 自适应时间间隔
    if (AutoDt) delta_t = DWT_GetDeltaTime(&dwt_dt);                    // 单位为秒
    if (delta_t > 10 * _delta_t_protect) delta_t = _delta_t_protect;         // 保护性Dt，防止频率过低导致计算错误
    // 根据Dt配置保护性Dt
    _delta_t_protect = 0.9f * _delta_t_protect + 0.1f * delta_t;

    // 增量式PID的微分项低通滤波
    kd_error = kd_error + kd_filter_rate * (error - kd_error);
    
    // 计算增量式PID
    inc_output =    Kp * (error - last_error) +
                    Ki * error * delta_t +
                    Kd * (kd_error - 2 * last_error + prev_error) / delta_t;
    
    // 记录误差项，用于下次计算
    prev_error = last_error;
    last_error = error;
    
    // 如果用户配置了反向，作反向输出
    if (reverse)            inc_output = -inc_output;
    
    // 应用输出限幅（如果配置了限幅） 
    if (output_lim > 0)     inc_output = limit_ab(inc_output, output_lim);

    // 返回增量
    return inc_output;
}

/**
 * @brief 计算增量式PID的输出（内部维护累加值和限幅），带微分滤波器
 * @param targ_pid PID结构体指针
 * @param error 当前误差
 * @warning PID的频率要是低于20Hz，显然不可能保证各项正确了；将采用保护性Dt(Dt_protect工作正常时间累计)
 * @return 经过累加和限幅后的控制量
 */
float PidGeneral::CalcIncAuto(float targ, float real, float output_lim)
{
    error = targ - real;

    // 自适应时间间隔
    if (AutoDt) delta_t = DWT_GetDeltaTime(&dwt_dt);                    // 单位为秒
    if (delta_t > 10 * _delta_t_protect) delta_t = _delta_t_protect;         // 保护性Dt，防止频率过低导致计算错误
    // 根据Dt配置保护性Dt
    _delta_t_protect = 0.9f * _delta_t_protect + 0.1f * delta_t;


    // 增量式PID的微分项低通滤波
    kd_error = kd_error + kd_filter_rate * (error - kd_error);
    
    // 计算控制量增量，使用误差进行微分项计算（P和I千万不能用微分滤波误差）
    inc_output =    Kp * (error - last_error) +
                    Ki * error * delta_t +
                    Kd * (kd_error - 2 * last_kderr + prev_kderr) / delta_t;
    
    // 记录误差项，用于下次计算
    prev_error = last_error;
    last_error = error;

    prev_kderr = last_kderr;
    last_kderr = kd_error;

    // 记录前馈项
    u_prev_2 = u_prev;
    u_prev = u;
    u = targ;
    
    // 更新控制量
    control_value += inc_output;
    
    float reterval = control_value;

    // 前馈控制
    if (Feedforward) switch (fwd_type)
    {
        case SpeedForward: reterval = control_value + FwdFuncs::SpdForward(u, u_prev, delta_t, K, Tc) * Kf; break;
        case PosForward:   reterval = control_value + FwdFuncs::PosForward(u, u_prev, u_prev_2, delta_t, K, Tc) * Kf; break;
        default: break;
    }
    

    // 应用外界输出限幅（如果配置了限幅）
    if (output_lim > 0)
    {
        control_value = limit_ab(control_value, output_lim);
        reterval = limit_ab(reterval, output_lim);
    }     

    // 应用内部输出限幅（如果配置了限幅）
    if (out_lim > 0)
    {
        control_value = limit_ab(control_value, out_lim);
        reterval = limit_ab(reterval, out_lim);
    }     
    
    // 如果用户配置了反向，作反向输出
    if (reverse)            reterval = -reterval;

    return reterval;

}

float PidGeneral::GetDt()
{
    return delta_t;
}


/***        前馈控制函数        ***/

/**
 * @brief 速度前馈控制器
 * @details 假设被控对象的传递函数为 标准一阶惯性环节 G(s) = K / (T*s + 1)
 */
float PidGeneral::FwdFuncs::SpdForward(float u, float u_prev, float dt, float K, float T_c)
{
    float part_a = (T_c / K) * (u - u_prev) / dt;
    float part_b = (1 / K) * u;

    return part_a + part_b;
}

/**
 * @brief 位置前馈控制器
 * @details 假设被控对象的传递函数为 一个积分环节与 一个一阶惯性环节的串联 G(s) = K / s(T*s + 1)
 */
float PidGeneral::FwdFuncs::PosForward(float u, float u_prev, float u_prev_2, float dt, float K, float T_c)
{
    float part_a = (T_c / K) * (u - 2 * u_prev + u_prev_2) / (dt * dt);
    float part_b = (1 / K) * (u - u_prev) / dt;

    return part_a + part_b;
}