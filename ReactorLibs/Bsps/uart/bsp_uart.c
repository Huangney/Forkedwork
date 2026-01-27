#include "bsp_uart.h"

#define BSPUART_MAX_CANINSTS 6          // 最多支持6个串口实例


/// @brief 收集记录所有实例，便于管理（static化避免外部调用）
static BspUart_Instance *bspuart_insts[BSPUART_MAX_CANINSTS] = {NULL};
static int bspuart_inst_count = 0; // 当前实例数量


/// @brief 注册一个串口实例
/// @param inst 实例指针
/// @param huart 所用串口
/// @param type 工作模式：普通、IT、DMA
/// @param rx_callback 响应触发的回调函数（一个Uart通道只允许一个实例存在，自然也只允许一个回调函数存在）
void BspUart_InstRegist(BspUart_Instance *inst, UART_HandleTypeDef *huart, uint8_t rx_setlen,
                        BspUart_TypeDef rxtype, BspUart_TypeDef txtype, BspUart_InstRxCallback rx_callback)
{
    // 检验参数有效性
    if (inst == NULL || huart == NULL) return; // 参数无效

    // 初始化实例
    inst->huart = huart;
    inst->rxtype = rxtype;
    inst->txtype = txtype;
    inst->rx_callback = rx_callback;

    // 清空接收缓冲区
    memset(inst->rx_buffer, 0, sizeof(inst->rx_buffer));
    // 期望接收数据长度，也是DMA的最大长度，DMA模式下达到本长度将触发全满中断
    inst->rx_setlen = rx_setlen;        

    // 根据工作模式，配置接收中断 注：IT的接收中断我没测试过，DMA的肯定可以用，IT的想用可以自己测试（我甚至没写回调函数嘿嘿）
    if (rxtype == BspUartType_IT)
    {
        // 使能接收中断
        HAL_UART_Receive_IT(huart, &inst->rx_byte, 1); // IT模式下单次接收1字节
    }
    else if (rxtype == BspUartType_DMA)
    {
        // 使能DMA接收，同时关闭DMA半满中断
        HAL_UARTEx_ReceiveToIdle_DMA(huart, inst->rx_buffer, rx_setlen);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    // 记录实例：1. 实例列表未满；2. 同一通道实例只能注册一次
    if (bspuart_inst_count < BSPUART_MAX_CANINSTS)
    {
        // 检查是否已有相同通道的实例
        for (int i = 0; i < bspuart_inst_count; i++)
        {
            if (bspuart_insts[i]->huart == huart)
            {
                // 已有相同通道实例，进入软件错误中断
                Error_Handler();
            }
        }
        // 添加新实例
        bspuart_insts[bspuart_inst_count++] = inst;
    }
}


void BspUart_Transmit(BspUart_Instance inst, uint8_t *data, uint8_t len)
{
    if (inst.huart == NULL || data == NULL || len == 0) return; // 参数无效

    // 根据其txtype发送数据
    if (inst.txtype == BspUartType_Normal)
    {
        HAL_UART_Transmit(inst.huart, data, len, 100); // 普通模式，阻塞发送
    }
    else if (inst.txtype == BspUartType_IT)
    {
        HAL_UART_Transmit_IT(inst.huart, data, len);   // IT模式，非阻塞发送
    }
    else if (inst.txtype == BspUartType_DMA)
    {
        HAL_UART_Transmit_DMA(inst.huart, data, len);  // DMA模式，非阻塞发送
    }
    else
    {
        Error_Handler(); // 未知模式，进入软件错误中断
    }
}




/// @brief 覆写原DMA接收中断函数
/// @param huart 
/// @param Size 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 遍历所有实例，根据huart通道，找到对应的实例
    for (int i = 0; i < bspuart_inst_count; i++)
    {
        BspUart_Instance *inst = bspuart_insts[i]; // 取出实例
        if (inst->huart == huart)
        {
            // 找到匹配的实例，调用其接收回调函数
            if (inst->rx_callback != NULL)
            {
                inst->rx_len = Size;                                // 记录实际接收长度
                inst->rx_callback(huart, inst->rx_buffer, Size);    // 调用用户定义的回调函数
            }
            // 执行后清空缓冲区
            memset(inst->rx_buffer, 0, inst->rx_setlen);
            // 重新使能DMA接收 并 关闭半满中断
            HAL_UARTEx_ReceiveToIdle_DMA(huart, inst->rx_buffer, inst->rx_setlen);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }
}