#ifndef BSP_CAN_H
#define BSP_CAN_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "can.h"

#define BSPCAN_STD 0 // 标准帧发送
#define BSPCAN_EXT 1 // 扩展帧发送

#define BSPCAN_DATA 0 // 数据帧发送
#define BSPCAN_REMOTE 1 // 远程帧发送

#define BSPCAN_MAX_CANINSTS 24 // 最多支持24个实例
#define CAN_1_USED
#define CAN_2_USED

// 发送用的结构体
typedef struct
{
    CAN_HandleTypeDef *hcan;    // CAN句柄
    uint8_t IDE;                // 是否是扩展帧
    uint8_t RTR;                // 是否是远程帧
    uint32_t Id;                // 标准ID
    uint32_t DLC;               // 数据长度
    uint16_t timeout_ms;        // 超时时间（毫秒）
}BspCan_TxConfig;

// 定义接收回调函数类型
typedef void (*BspCan_InstRxCallback)(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData, CAN_HandleTypeDef *hcan);
/// @brief BSPCAN实例及其句柄定义
typedef struct
{
    CAN_HandleTypeDef *hcan; // CAN句柄，用哪路CAN看自己喜好了
    uint32_t rx_id;          // 接收ID
    uint8_t rx_isExtend;     // 是否扩展ID
    uint32_t tx_id;          // 发送ID
    uint8_t tx_isExtend;     // 是否扩展ID

    BspCan_InstRxCallback rx_callback; // 接收回调函数
} BspCan_Instance;
typedef BspCan_Instance *BspCan_HandleTypeDef;

/******     函数    ******/
void BspCan_InstRegist(BspCan_HandleTypeDef inst, CAN_HandleTypeDef *hcan, uint32_t rx_id, uint32_t tx_id,
                        uint8_t rx_isExtend, uint8_t tx_isExtend, BspCan_InstRxCallback rx_callback);
BspCan_TxConfig BspCan_GetTxConfig(CAN_HandleTypeDef *hcan , int id, uint8_t IDE, uint8_t RTR, uint32_t DLC, uint16_t timeout_ms);
BspCan_TxConfig BspCan_GetTxConfigFast(CAN_HandleTypeDef *hcan , int id);
void BspCan_Transmit(BspCan_TxConfig conf, uint8_t *data);


#ifdef __cplusplus
}
#endif
#endif
