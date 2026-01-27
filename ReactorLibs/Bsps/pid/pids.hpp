#ifndef PIDS_HPP
#define PIDS_HPP
#ifdef __cplusplus
#include "stm32f4xx_hal.h"


extern "C"
{
#endif
    // 通用PID类
    class PidGeneral
    {
    private:
        class FwdFuncs
        {
            public:
            FwdFuncs(){};
            static float SpdForward(float u, float u_prev, float dt, float K, float T_c);
            static float PosForward(float u, float u_prev, float u_prev_2, float dt, float K, float T_c);
        };

        // PID参数
        float Kp;      // 比例系数
        float Ki;      // 积分系数
        float Kd;      // 微分系数
        float Kf = 0.0f;      // 前馈系数
        float delta_t = 0.0f; // 时间间隔
        float _delta_t_protect = 0.05f; // 保护性时间间隔，防止频率过低

        int reverse;   // 反向控制标志
        uint32_t dwt_dt;    // 自动DT用的DWT句柄

        // 前馈用参数
        float Tc = 1;       // 时间常数
        float K = 5;        // 环节增益
        float u;
        float u_prev;
        float u_prev_2;

        // 限制参数
        float inte_lim;    // 积分限幅
        float out_lim;      // 输出限幅
        float kd_filter_rate = 0.9f; // Kd低通滤波系数
        
        /// @brief 举个例子吧：假设死区起始为80，结束为160，那么误差 < 80输出0，误差 > 160正常输出，80 ~ 160之间线性映射
        float deadband_start = 0; // 死区起始值
        float deadband_end = 0;   // 死区结束值

        // PID高级选项
        bool Incremental = false; // 是否为增量式PID
        bool Feedforward = false; // 是否启用前馈控制
        bool InnerAcc = false;    // 是否启用内部累加（仅增量式有效）
        bool AutoDt = true;      // 是否自动计算PID时间间隔（默认开启）
        bool DeadbandEnabled = false;    // 是否启用死区控制

        float CalcPos(float targ, float real, float output_lim = 0);               // 位置式计算
        float CalcInc(float targ, float real, float output_lim = 0);               // 增量式计算
        float CalcIncAuto(float targ, float real, float output_lim = 0);           // 带内部累加的 增量式计算
        
    public:
        /// @brief 前馈控制器类型
        /// @note 前馈控制器需要专门设计，所以这里简单地给出两种提前设计好的前馈控制器
        typedef enum
        {
            SpeedForward,
            PosForward,
        }Forward_Typedef;

        Forward_Typedef fwd_type = SpeedForward;

        // 内部状态变量
        float inte_errors;      // 积分误差累计
        float error;            // 当前误差
        float last_error;      // 上一次误差
        float prev_error;      // 上上次误差（用于增量式计算）
        
        float kd_error;        // 微分误差
        float last_kderr;       
        float prev_kderr;      
        
        float inc_output;       // 增量式PID的增量输出
        float control_value;   // 内部维护控制量累加值

        // 构造函数，带默认参数
        PidGeneral(){};
        
        // 初始化函数，带完整参数
        void Init(float kp, float ki, float kd, int reverse = false);
        /// @brief 启用增量PID模式
        void IncreLize(bool inner_acc = true);
        /// @brief 启用前馈控制
        void ForwardLize(Forward_Typedef fwd_type, float kf, float K = 1.0f, float Tc = 1.0f);
        /// @brief 手动设置时间间隔（同时禁用自动时间微分计算） 
        void ManualDt(float dt);
        
        /// @brief 设置参数
        void SetParam(float kp, float ki, float kd);
        /// @brief 设置限幅
        void SetLimit(float intelim, float outlim, float dfilter);
        /// @brief 设置反向控制
        void SetRev(bool reverse);
        /// @brief 设置死区控制
        void SetDeadband(float start, float end);

        /// @brief 获取
        float GetDt();

        // 计算PID输出（供外部调用） 
        float Calc(float targ, float real, float output_lim = 0);
        // 重置PID状态
        void Reset();
    };

typedef PidGeneral Pids; // 兼容C代码中的PIDs类型定义

#ifdef __cplusplus
}
#endif
#endif