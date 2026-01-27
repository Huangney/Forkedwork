#ifndef MONITOR_HPP
#define MONITOR_HPP

#include "msg_coder.hpp"
#include "typeinfo"
#include "stdarg.h"
#include "SysDefs.hpp"

/**
 * @brief 用于监控机器人的各项状态，还有调试、日志等功能]
 * @warning 本类也属于：单例
 */
class Monitor
{
    friend class SystemType;      // 允许系统类访问私有成员
    SINGLETON(Monitor){};

public:
    /// @brief 用于监视的结构体
    typedef struct WatchInfo
    {
        const bool* targ;
        char warning_info[24];
        bool is_neccessary;
    };
private:
    typedef enum
    {
        track_uint8,
        track_int8,
        track_uint16,
        track_int16,
        track_uint32,
        track_int32,
        track_float,
    } Track_t;

    typedef struct
    {
        void *track_addr; // 变量地址
        void *buf_addr;   // 将其变量值存入的缓存地址
        uint8_t bytes;    // 变量的字节数
    } MonitorLinkage;

    /// @brief 最多监测24个Bool
    WatchInfo watch_buf[24];
    /// @brief 最多追踪32Byte的数据
    byte track_buf[32];
    /// @brief 最多向Vofa发送64Byte数据
    byte vofa_buf[64];

    // 变量的地址和类型存储在这里
    void *track_list[8];
    Track_t track_type[8];

    uint8_t watch_count = 0;
    uint8_t track_count = 0;

    bool high_performance_mode = false;         // 高性能模式标志位（1000Hz跟踪）

public:
    UartMsgCoder host_coder;                    // 发送到上位机的编码器
    UartMsgCoder farcon_coder;                  // 发送到遥控器的编码器

    /**
     * @brief 监视器的初始化函数
     * @note 上位机有两种情况，一种是视觉组的工控机，一种是调试时候的电脑
     */
    void Init(UART_HandleTypeDef *huart_host, UART_HandleTypeDef *huart_farc, bool vofa_mode = false);

    /**
     * @brief 运行方法
     * @note 该方法应被周期性调用，以处理监控任务
     * @details 内含发送日志信息、发送机器人状态码、监控维护模块等功能
     */
    void Run();

    /// @brief 切换高性能模式
    void Perflize();

    /// @brief 发送日志
    void Log(const char *format, ...);
    // void Log(const char *format);       // 空参数重载

    /// @brief 发送警告
    void LogWarning(const char *format, ...);
    // void LogWarning(const char *format); // 空参数重载

    /// @brief 发送错误
    void LogError(const char *format, ...);
    // void LogError(const char *format);   // 空参数重载

    /// @brief 监控某个模块的状态变化
    void Watch(WatchInfo info);

    /**
     * @brief 跟踪某个变量
     * @note 调用本函数后，其将被编码并加入机器人状态码中
     * @param track_hz 跟踪频率，单位Hz，最大为200Hz
     */
    template <typename T>
    void Track(T &targ)
    {
        using namespace std;

        // 检查追踪越界
        if (track_count >= 8)
        {
            LogError("Monitor: Track list full!\n");
            return;
        }

        // 存储其类型信息
        const type_info &targ_type = typeid(targ);
        if (targ_type == typeid(uint8_t))
        {
            track_type[track_count] = track_uint8;
        }
        else if (targ_type == typeid(int8_t))
        {
            track_type[track_count] = track_int8;
        }
        else if (targ_type == typeid(uint16_t))
        {
            track_type[track_count] = track_uint16;
        }
        else if (targ_type == typeid(int16_t))
        {
            track_type[track_count] = track_int16;
        }
        else if (targ_type == typeid(uint32_t))
        {
            track_type[track_count] = track_uint32;
        }
        else if (targ_type == typeid(int32_t))
        {
            track_type[track_count] = track_int32;
        }
        else if (targ_type == typeid(float))
        {
            track_type[track_count] = track_float;
        }
        else
        {
            LogError("Monitor: Unknown Track type!\n");
            return;
        }

        // 存储其地址
        track_list[track_count] = (void *)&targ;

        track_count++;
    }

    /// @brief 发送监控信息
    void LogWatch();

    /// @brief 发送跟踪信息
    void LogTrack();

    /**
     * @brief 发送跟踪信息 (JustFloat格式，极快)
     * @note 无论由于Track的是什么类型，这里统统转为float发送
     * @note 协议帧尾: 00 00 80 7f
     */
    void LogTrackJustFloat();
};

#endif