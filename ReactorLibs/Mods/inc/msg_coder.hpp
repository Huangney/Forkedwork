#ifndef MSG_CODER_HPP
#define MSG_CODER_HPP

#include "stm32f4xx_hal.h"
#include "bsp_uart.h"
#include "std_math.hpp"

typedef struct
{
    uint8_t frame_head; // 帧头
    uint8_t frame_type; // 帧类型
    uint8_t data[64];   // 数据包（最大64Byte）
    int data_len;       // 数据包长度
    uint8_t frame_tail; // 帧尾
} FrameStruct;

static void UartMsgCoder_General_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);

/**
 * @brief 串口消息调制器
 * @note 只有遇到自定义通信的时候才会用到单独的调制器实例
 */
class UartMsgCoder
{
    friend void UartMsgCoder_General_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);
private:
    /// @brief 对应 MsgCoder 实例的回调函数指针
    void (*_coder_callback)(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size) = nullptr;

public:
    UartMsgCoder() {};
    
    /// @brief 已注册的调制器实例指针列表
    bool coder_ocupied = false;
    
    /// @brief 帧结构体
    FrameStruct frame; 

    /// @brief 串口实例
    BspUart_Instance uart_inst;

    /// @brief 初始化函数
    void Init(UART_HandleTypeDef *huart);

    /// @brief 创建其中一个回调函数（可能某一个串口实例会有多个MsgCoder）
    /// @param Callback 目标回调函数
    void SetCallback(void (*Callback)(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size));
    
    uint8_t CalculateFrameHead(uint8_t *data, int data_len);
    void SetFrameParam(uint8_t Frame_Head, uint8_t Frame_Type, uint8_t *Data, int data_len, uint8_t Frame_Tail);

    /**     消息编码与解码  	**/
    int Encode(uint8_t *buf);
    int EncodeMsg(uint8_t frame_type, uint8_t *data, int data_len, uint8_t *out_buf);
    int DecodeMsg(uint8_t *buf);

    /**     发送接口     **/
    bool SendRawMsg(uint8_t *encoded_data, int length);
    bool SendMsg(uint8_t frame_type, uint8_t *data, int data_len);

    /**     工具方法（静态方法）  	**/
    static void FloatToBytes(float value, uint8_t *bytes);
    static float BytesToFloat(uint8_t *bytes);
    static void Uint16ToBytes(uint16_t value, uint8_t *bytes);
    static uint16_t BytesToUint16(uint8_t *bytes);

    /**     预定义的回调函数    **/
    static void EchoRx(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);
};

/**
 * @brief CAN消息调制器
 * @note 只有遇到自定义通信的时候才会用到单独的调制器实例
 */
class CanMsgCoder
{
public:
    CanMsgCoder() {};

    void Init();
};

#endif