#ifndef __ODO_OPS_HPP__
#define __ODO_OPS_HPP__

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "std_math.hpp"
#include "bsp_uart.h"

#define ACTION_USART_HANDLE huart6


#pragma pack(2)
/**
 * @brief 里程计接收数据结构体
 * @note 该结构体必须2字节对齐，否则在memcpy时会出错
 * （这里涉及到结构体存储优化、内存对齐等微机原理知识点，可以自己搜一下）
 * @warning 老子服了，ACTION的例程注释和说明书都是错的，正确的帧头是0A0D，帧尾是0D0A
 */
typedef struct OdometerData
{
    /// @note 按照action发送的数据包，结构体成员的顺序必须如下
    uint16_t frame_head;                //帧头0A0D          
    float yaw;                          //逆时针旋转为角度正方向，范围0~180，-180~0
    float pith;
    float roll;
    float x;
    float y;
    float yaw_speed;
    uint16_t frame_tail;                //帧尾0D0A
}OdoData;                              //数据包严格为28字节
#pragma pack()


class Odometer_Ops9
{
private:
    /* data */
public:
    Odometer_Ops9(){};
    ~Odometer_Ops9(){};

    BspUart_Instance uart_inst;     // 串口实例
    OdoData odom_data;              // 里程计数据
    uint32_t recv_times;            // 接收次数
    uint32_t err_recv;              // 接收错误次数

    /// @brief 这个数组用于计算速度，x放每次移动的距离，y放每次移动所间隔的时间
    Vec2 distan_and_time[10];       // 距离和时间

    uint8_t DaT_index;         // distan_and_time数组的索引[0 ~ 9]
    float total_dist;               // 累计的距离，减小计算量
    float total_time;               // 累计的时间，减小计算量

    Vec3 last_transform;    // 上次位置
    Vec3 transform;         // 里程计的当前位置
    // Vec2 velo;              // 带方向的速度
    float speed;            // 速率

    bool rev_x;             // 激活时，x方向数据取反
    bool rev_y;             // 激活时，y方向数据取反
    bool rev_yaw;           // 激活时，yaw方向数据取反
    bool swap_xy;           // 激活时，x、y数据交换

    void Init(UART_HandleTypeDef *huart, bool rev_x = false, bool rev_y = false,
            bool rev_yaw = false, bool swap_xy = false);
};

typedef Odometer_Ops9 Ops9;







//extern uint8_t dma_rx_buffer[30];//存储action发的数据包，30个字节
//extern Odometer odometer_data;//里程计数据

static void update_odom(char a, float value);
void odom_dma_init();
// Vec3 odom_get_tf_vec(Odometer odom_sturct);

void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
// Odometer get_odometer_data();

extern uint32_t odom_recvtimes;
extern uint8_t odom_position_inited;
// 车体里程计
// extern Odometer chassis_odom;


#endif


