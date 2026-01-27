#include "bsp_can.h"
#include "can.h"
#include "main.h"
#include "bsp_dwt.h"

// 统计类变量
uint32_t can_recv_times = 0; // CAN接收次数
uint32_t can_busy_times = 0; // CAN忙碌次数



// 过滤器组资源管理：CAN1用0-13，CAN2用14-27
static uint8_t can1_filter_idx = 0;  // CAN1过滤器组索引（0-13）
static uint8_t can2_filter_idx = 14; // CAN2过滤器组索引（14-27）
static uint8_t CanActivated = 0;     // CAN是否已启动标志

/// @brief 收集记录所有实例，便于管理（static化避免外部调用）
static BspCan_HandleTypeDef bspcan_insts[BSPCAN_MAX_CANINSTS] = {NULL};
static int bspcan_inst_count = 0; // 当前实例数量

/// @brief 添加一个新的CAN过滤器 到 过滤器列表中（static化避免外部调用）
/// @namespace BspCan
/// @param inst 目标实例
/// @return -1: 添加失败 0：添加成功
static int BspCan_FilterAdd(BspCan_HandleTypeDef inst)
{
    if (inst == NULL || inst->hcan == NULL)
    {
        return -1; // 参数无效
    }

    CAN_FilterTypeDef FilterConfig = {0};
    uint8_t filter_bank;

    // 分配过滤器组（逻辑不变）
    if (inst->hcan == &hcan1)
    {
        if (can1_filter_idx > 13)
            return -1;
        filter_bank = can1_filter_idx++;
    }
    else
    {
        if (can2_filter_idx > 27)
            return -1;
        filter_bank = can2_filter_idx++;
    }

    // 配置过滤器ID和掩码
    if (inst->rx_isExtend) // 扩展帧配置（29位ID）
    {

        FilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器才能容纳29位ID

        // 扩展帧ID格式：[28:0]为ID，需设置IDE位（bit31=1表示扩展帧）
        FilterConfig.FilterIdHigh = (inst->rx_id >> 13) & 0xFFFF;        // 高16位（28-13位）
        FilterConfig.FilterIdLow = ((inst->rx_id << 3) | 0x04) & 0xFFFF; // 低13位（12-0位）+ IDE=1（bit3=1）
    }
    else // 标准帧配置（11位ID）
    {

        FilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        FilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // 16位过滤器足够

        // 标准帧ID格式：左移5位（11位ID占高11位，低5位为0），IDE=0（默认）
        FilterConfig.FilterIdLow = inst->rx_id << 5;
        FilterConfig.FilterIdHigh = 0x0000;
    }

    // FIFO分配和其他配置（保持不变）
    FilterConfig.FilterFIFOAssignment = (inst->rx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    FilterConfig.SlaveStartFilterBank = 14;
    FilterConfig.FilterBank = filter_bank;
    FilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(inst->hcan, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
        return -1;
    }

    return 0;
}

/// @brief 启用目标CAN路线
/// @param hcan
static void BspCan_Activate()
{
#ifdef CAN_1_USED
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif

#ifdef CAN_2_USED
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif
}

/// @brief 实例注册
/// @param inst 为了避免内存泄漏，目标变量应该由外部提供
/// @param rx_callback 接收回调函数
/// @return
void BspCan_InstRegist(
    BspCan_HandleTypeDef inst, CAN_HandleTypeDef *hcan,
    uint32_t rx_id, uint32_t tx_id,
    uint8_t rx_isExtend, uint8_t tx_isExtend,
    BspCan_InstRxCallback rx_callback)
{
    // 检验参数有效性
    if (inst == NULL || hcan == NULL)
    {
        return; // 参数无效
    }

    // 初始化实例
    inst->hcan = hcan;
    inst->rx_id = rx_id;
    inst->tx_id = tx_id;
    inst->rx_isExtend = rx_isExtend;
    inst->tx_isExtend = tx_isExtend;
    inst->rx_callback = rx_callback;

    // 添加过滤器
    BspCan_FilterAdd(inst);

    // 启动CAN
    if (!CanActivated)
    {
        BspCan_Activate();
        CanActivated = 1;
    }

    // 记录实例
    if (bspcan_inst_count < BSPCAN_MAX_CANINSTS)
    {
        bspcan_insts[bspcan_inst_count++] = inst;
    }
}




/// @brief 获取发送配置
/// @param IDE 是否为扩展帧
/// @param RTR 
/// @param DLC 
/// @param timeout_ms 
/// @return 
BspCan_TxConfig BspCan_GetTxConfig(CAN_HandleTypeDef *hcan , int id, uint8_t IDE, uint8_t RTR, uint32_t DLC, uint16_t timeout_ms)
{
    BspCan_TxConfig conf = {0};
    conf.hcan = hcan;
    conf.IDE = IDE;
    conf.RTR = RTR;
    conf.Id = id;
    conf.DLC = DLC;
    conf.timeout_ms = timeout_ms;

    return conf;
}

/// @brief 快速获取默认发送配置
/// @param 默认参数： `标准帧` 、`数据帧`、`8字节`、`10ms超时`
/// @return 
BspCan_TxConfig BspCan_GetTxConfigFast(CAN_HandleTypeDef *hcan , int id)
{
    BspCan_TxConfig conf = {0};
    conf.hcan = hcan;
    conf.IDE = BSPCAN_STD;
    conf.RTR = BSPCAN_DATA;
    conf.Id = id;
    conf.DLC = 8;
    conf.timeout_ms = 10;

    return conf;
}




void BspCan_Transmit(BspCan_TxConfig conf, uint8_t *data)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    // 配置报文头
    TxHeader.StdId = conf.Id;   // 标准ID
    TxHeader.ExtId = conf.Id;   // 扩展ID
    TxHeader.IDE = conf.IDE ? CAN_ID_EXT : CAN_ID_STD; // 设置ID类型(1=扩展帧, 0=标准帧)
    TxHeader.RTR = conf.RTR ? CAN_RTR_REMOTE : CAN_RTR_DATA; // 设置帧类型(1=远程帧, 0=数据帧   )
    TxHeader.DLC = conf.DLC; // 数据长度

    // 获取时间，防止超时
    float time_start = DWT_GetTimeline_MSec();

    // 在规定时间内尝试发送CAN消息
    while (HAL_CAN_AddTxMessage(conf.hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        // 检查超时
        if (DWT_GetTimeline_MSec() - time_start > conf.timeout_ms)
        {
            can_busy_times++; // 记录忙碌次数
            return;
        }
    }
}



/// @brief 接收回调函数，处理接收到的CAN消息，会遍历所有实例，找到符合的CANID并调用对应的回调函数
static void BspCan_RxCallbackManager(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
    // 先获取报文头和数据
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0}; // 接收数据缓冲区
    if (HAL_CAN_GetRxMessage(hcan, fifo, &RxHeader, RxData) != HAL_OK)
    {
        Error_Handler(); // 获取失败，调用错误处理函数
    }

    // 遍历所有实例，查找匹配的ID，并调用其回调函数
    for (int i = 0; i < bspcan_inst_count; i++)
    {
        BspCan_HandleTypeDef inst = bspcan_insts[i]; // 取出实例
        if (inst->hcan == hcan &&
            ((inst->rx_isExtend && RxHeader.ExtId == inst->rx_id) ||
             (!inst->rx_isExtend && RxHeader.StdId == inst->rx_id)))
        {
            // 找到匹配的实例，调用其接收回调函数
            if (inst->rx_callback != NULL)
            {
                inst->rx_callback(&RxHeader, RxData, hcan); // 调用用户定义的回调函数
            }
            return; // 执行后退出
        }
    }
}

/// @brief 覆写原接收中断函数_FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_recv_times++; // 记录接收次数
    BspCan_RxCallbackManager(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}
/// @brief 覆写原接收中断函数_FIFO1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_recv_times++; // 记录接收次数
    BspCan_RxCallbackManager(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}
