/**
 * @author: Agilawood, Huangney
 * @name: Action—— OPS-9里程计 全场定位库 
 * @last_edit_time: 2025-9-7
 * 
 * @note: 1.本文件采用UTF-8编码
 *        2.action使用RS232物理层协议，需使用TTL转RS232模块( 型号: Freestrong TTL to RS485/RS232 Rev 1.0 )
 *        3.硬件接法说明(Crucial)：模块的 T 接“TTL转RS232模块”的 A/TXD, R 接 B/RXD ; MCU的串口TX 接 “TTL转RS232模块” 的 TX， RX 接 RX；模块VIN接输出稳定的5V，并做好“共地”处理
 *        4.串口波特率为115200 bit/s
 *
 * @brief: 该库支持接收和更新x、y、yaw三方向坐标，使用dma机制
 * 
 * @api: 1.更新坐标函数：update_odom(char a, float value);
 *       2.dma中断、接收初始化函数：void odom_dma_init();
 *       3.dma中断处理函数：void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
 *       4.里程计数据接口函数：Odometer get_odometer_data();
 *
 * @example：一、必要准备
 * 
 *           1.在主函数中调用以下函数初始化dma接收： 
 *             odom_dma_init();
 *           
 *           2.在interrupt.c中的对应串口中断函数中【如 void USART1_IRQHandler(void) 】，加入以下函数：
 *             USER_UART_IRQHandler(&ACTION_USART_HANDLE);
 * 
 *          二、使用示例
 * 
 *           1.可以根据需要使用以下函数更新坐标： 
 *             update_odom('x', value); // (x坐标)
 *             update_odom('y', value); // (y坐标)
 *             update_odom('j', value); // (yaw角) 
 * 
 *           2.可供用户读取 x，y，yaw，yaw_speed 四个值
 *             方法：定义一个Odometer型的变量，调用 get_odometer_data() 接收数据
 */

#include "odo_ops.hpp"
#include "bsp_dwt.h"

static uint8_t dma_rx_buffer[30];               //存储action发的数据包，30个字节
// static Odometer odometer_data;                  //里程计数据
#define OPS9_RECV_FREQ 100                          // 说明书中的里程计接收频率，单位Hz

static Odometer_Ops9* targ_odom = nullptr; // 全局指针，用于DMA回调访问类成员    

// 车体里程计
// Odometer chassis_odom;
// uint32_t odom_recvtimes;
uint8_t odom_position_inited;
static void OdometerOps9_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);


void Odometer_Ops9::Init(UART_HandleTypeDef *huart, bool rev_x, bool rev_y,
                        bool rev_yaw, bool swap_xy)
{
    // 初始化赋值里程计数据
    transform = Vec3(0, 0, 0);
    speed = 0;
    this->rev_x = rev_x;
    this->rev_y = rev_y;
    this->rev_yaw = rev_yaw;
    this->swap_xy = swap_xy;

    // 注册串口实例
    BspUart_InstRegist(&uart_inst, huart, 30, BspUartType_DMA, BspUartType_Normal, OdometerOps9_RxCallback);

    // 完成注册
    targ_odom = this;
}





static void OdometerOps9_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
    static uint32_t dwt_tick;

    if (size == 28) // 确认数据长度正确
    {
        memcpy(&(targ_odom->odom_data), rxData, 28); 
        // 这里可以添加对数据的验证，比如检查帧头和帧尾
        if (targ_odom->odom_data.frame_head == 0x0A0D && targ_odom->odom_data.frame_tail == 0x0D0A)
        {
            // 数据有效，更新里程计位置Vec（米和弧度制）
            targ_odom->last_transform = targ_odom->transform;
            targ_odom->transform.x = (targ_odom->rev_x ? -targ_odom->odom_data.x : targ_odom->odom_data.x) / 1000.0;
            targ_odom->transform.y = (targ_odom->rev_y ? -targ_odom->odom_data.y : targ_odom->odom_data.y) / 1000.0;
            targ_odom->transform.z = (targ_odom->rev_yaw ? -targ_odom->odom_data.yaw : targ_odom->odom_data.yaw) * 3.1415926 / 180.0;

            // 如果需要反转x、y轴
            if (targ_odom->swap_xy)
            {
                float temp = targ_odom->transform.x;
                targ_odom->transform.x = targ_odom->transform.y;
                targ_odom->transform.y = temp;
            }

            // 计算速率，首先计算本次移动距离
            float dist = (targ_odom->transform - targ_odom->last_transform).Length();
            // 获取与上次的时间差，单位秒
            float dt = DWT_GetDeltaTime(&dwt_tick);                 
            
            // 更新距离和时间
            targ_odom->total_dist -= targ_odom->distan_and_time[targ_odom->DaT_index].x;
            targ_odom->distan_and_time[targ_odom->DaT_index].x = dist;
            targ_odom->total_dist += dist;

            targ_odom->total_time -= targ_odom->distan_and_time[targ_odom->DaT_index].y;
            targ_odom->distan_and_time[targ_odom->DaT_index].y = dt;
            targ_odom->total_time += dt;

            targ_odom->DaT_index++;
            if (targ_odom->DaT_index > 9) targ_odom->DaT_index = 0;

            // 计算速率
            if (targ_odom->total_time > 0)
                targ_odom->speed = targ_odom->total_dist / targ_odom->total_time;
            else
                targ_odom->speed = 0;

            // 累计接收次数
            targ_odom->recv_times++;
        }
        else targ_odom->err_recv++;
    }
    else targ_odom->err_recv++;
}












































/* 字符串拼接函数 */
void stractString(char str1[], char str2[], int num)
{
	int i = 0, j = 0;
    while(str1[i] != '\0')
    {
        i++;
    }
    for(j = 0; j < num; j++)
    {
        str1[i++] = str2[j];
    }
}

/* 以下为给Action发数据的函数 */
/**
  *@brief  更新X坐标函数，与update_Odometer()配合使用
  *@param  需要更新的float型 x坐标，该模块规定必须为float
  *@retval void 
  *@plus   使用共用体主要是为了方便数据类型的转换(float和char)，此外还可以节省内存
*/

static void update_x(float new_x)
{
    char x[8] = "ACTX";
    static union 
    {
        float X;
        char data[4];
    }NewSet;
    
    NewSet.X = new_x;

    stractString(x, NewSet.data, 4);    //整合数据
    
    for(int i = 0; i < 8; i++)    //串口把数据发送给action
    {
        HAL_UART_Transmit(&ACTION_USART_HANDLE, (uint8_t*)&x[i], 1, 1000);//内部自置等待上一笔数据发送完的flag
    }

    HAL_Delay(10);//应使用vTaskDelay
}

/* 更新标定Y坐标 */
static void update_y(float new_y)
{
    char y[8] = "ACTY";
    static union 
    {
        float Y;
        char data[4];
    }NewSet;
    
    NewSet.Y = new_y;

    stractString(y, NewSet.data, 4);    //整合数据

    for(int i = 0; i < 8; i++)    //串口把数据发送给action
    {
        HAL_UART_Transmit(&ACTION_USART_HANDLE, (uint8_t*)&y[i], 1, 1000);//内部自置等待上一笔数据发送完的flag
    }

    HAL_Delay(10);//应使用vTaskDelay
}

/* 更新标定yaw角 */
static void update_yaw(float new_yaw)
{
    char yaw[8] = "ACTJ";//char型存疑
    static union 
    {
        float J;
        char data[4];
    }NewSet;
    NewSet.J = new_yaw;
    stractString(yaw, NewSet.data, 4);//整合数据
    
    for(int i = 0; i < 8; i++)//串口把数据发送给action
    {
        HAL_UART_Transmit(&ACTION_USART_HANDLE, (uint8_t*)&yaw[i], 1, 1000);//内部自置等待上一笔数据发送完的flag，是否应该转换为uint8_t存疑
    }
    HAL_Delay(10);//应使用vTaskDelay
}


/**
  * @brief 更新三方向坐标 
  * @note  更新yaw角时，传的参数是j; value必须为float型 
  * @param char x,y,j
  * @param float value
  * @retval void
  */
static void update_odom(char a, float value)
{
    if(a == 'x')
    {
        update_x(value);
    }
    if(a == 'y')
    {
        update_y(value);
    }
    if(a == 'j')
    {
        update_yaw(value);
    }
}