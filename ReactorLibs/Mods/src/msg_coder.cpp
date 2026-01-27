#include "msg_coder.hpp"
#include "string.h"

// 下列变量用于计算有效接收率的（可删除）
int total = 0;     // 总共发送的次数
int effective = 0; // 有效接收的次数
float ratio = 0;   // 有效接收率

/// @brief 存储已经注册的串口调制器实例的指针，便于回调函数查表处理
static UartMsgCoder *UartMsgPointList[16] = {nullptr}; // 最多支持16个实例
uint8_t UartMsgPointListCount = 0;                     // 当前串口调制器实例数量

static void UartMsgCoder_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);


/**
 * @brief 串口接收（总）回调函数
 * @param huart 串口句柄
 * @param rxData 接收到的数据s
 * @param size 接收到的数据长度
 * @details 会依次调用每一个注册了的回调函数
 */
static void UartMsgCoder_General_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
    // 遍历所有注册的调制器实例，根据huart通道，找到对应的实例
    for (int i = 0; i < UartMsgPointListCount; i++)
    {
        // 取出实例
        UartMsgCoder& coder = *(UartMsgPointList[i]); 

        // 对比是否是对应的串口通道
        if (coder.uart_inst.huart == huart)
        {
            // 如果有用户自定义的回调函数，则调用它
            if (coder._coder_callback != nullptr)
            {
                coder._coder_callback(huart, rxData, size);
            }
            break; // 找到对应实例后跳出循环
        }
    }
}


/**
 * @brief 初始化串口消息调制器
 * @param huart 串口句柄
 */
void UartMsgCoder::Init(UART_HandleTypeDef *huart)
{
    UartMsgPointList[UartMsgPointListCount++] = this; // 注册该调制器实例到全局列表中

    // 初始化（即注册）该调制器的串口实例
    BspUart_InstRegist(&uart_inst, huart, 64, BspUartType_DMA, BspUartType_DMA, UartMsgCoder_General_RxCallback);
}


/**
 * @brief 编码主机消息
 * @param buf 输出缓冲区
 * @return 编码后的总长度
 */
int UartMsgCoder::Encode(uint8_t *buf)
{
    if (this->frame.frame_head == 0 || this->frame.frame_tail == 0)
        return 0; // 帧头或帧尾未设置，返回0

    int index = 0;                                               // 索引
    buf[index++] = this->frame.frame_head;                       // 帧头
    buf[index++] = this->frame.frame_type;                       // 帧类型
    memcpy(&buf[index], this->frame.data, this->frame.data_len); // 数据包
    index += this->frame.data_len;                               // 更新索引
    buf[index++] = this->frame.frame_tail;                       // 帧尾
    return index;                                                // 返回总长度
}

/**
 * @brief 高级编码接口 - 自动处理帧头和帧尾
 * @param frame_type 帧类型
 * @param data 数据指针
 * @param data_len 数据长度
 * @param out_buf 输出缓冲区
 * @return 编码后的数据长度
 */
int UartMsgCoder::EncodeMsg(uint8_t frame_type, uint8_t *data, int data_len, uint8_t *out_buf)
{
    if (data_len > 64)
    {
        data_len = 64; // 限制最大长度为64
    }

    // 自动计算帧头帧尾
    uint8_t temp_buf[65] = {0};
    temp_buf[0] = frame_type;
    memcpy(&temp_buf[1], data, data_len);

    // 计算帧头和帧尾校验值
    uint8_t frame_head = CalculateFrameHead(temp_buf, data_len + 1);
    uint8_t frame_tail = frame_head;

    // 设置帧参数
    SetFrameParam(frame_head, frame_type, data, data_len, frame_tail);

    // 编码
    return Encode(out_buf);
}

/**
 * @brief 解码主机消息
 * @param buf 输入缓冲区
 * @return 校验成功返回 帧类型，校验失败返回 -1
 */
int UartMsgCoder::DecodeMsg(uint8_t *buf)
{
    uint8_t frame_head = buf[0]; // 帧头
    uint8_t frame_type = buf[1]; // 帧类型

    // 检查帧头校验
    uint8_t calc_frame_head = this->CalculateFrameHead(&buf[1], this->frame.data_len + 1); // 计算帧头校验值
    if (frame_head != calc_frame_head)
        return -1; // 校验失败，返回-1

    // 检查帧尾校验
    uint8_t frame_tail = buf[2 + this->frame.data_len]; // 帧尾
    if (frame_tail != calc_frame_head)
        return -1; // 校验失败，返回-1

    return frame_type; // 校验通过，返回帧类型
}

/**
 * @brief 发送编码后的消息
 * @param encoded_data 编码后的数据指针
 * @param length 数据长度
 * @return 发送成功返回 true，失败返回 false
 */
bool UartMsgCoder::SendRawMsg(uint8_t *encoded_data, int length)
{
    if (encoded_data == nullptr || length == 0 || coder_ocupied)
    {
        return false;
    }

    coder_ocupied = true;
    // 使用BspUart发送数据
    BspUart_Transmit(uart_inst, encoded_data, length);
    coder_ocupied = false;
    return true;
}

/**
 * @brief 发送消息（自动编码并发送）
 * @param frame_type 帧类型
 * @param data 数据指针
 * @param data_len 数据长度（最大为64）
 * @return 发送成功返回 true，失败返回 false
 */
bool UartMsgCoder::SendMsg(uint8_t frame_type, uint8_t *data, int data_len)
{
    if (data_len > 64)
    {
        data_len = 64; // 限制最大长度为64
    }

    uint8_t encoded_buf[67]; // 足够大的缓冲区
    int encoded_len = EncodeMsg(frame_type, data, data_len, encoded_buf);

    if (encoded_len > 0)
    {
        return SendRawMsg(encoded_buf, encoded_len);
    }
    return false;
}

/**
 * @brief 设置帧参数
 * @param Frame_Head 帧头
 * @param Frame_Type 帧类型
 * @param Data 数据指针
 * @param Data_Len 数据长度
 * @param Frame_Tail 帧尾
 */
void UartMsgCoder::SetFrameParam(uint8_t Frame_Head, uint8_t Frame_Type, uint8_t *Data, int Data_Len, uint8_t Frame_Tail)
{
    this->frame.frame_head = Frame_Head;                    // 设置帧头
    this->frame.frame_type = Frame_Type;                    // 设置帧类型
    this->frame.data_len = (Data_Len > 64) ? 64 : Data_Len; // 设置数据包长度
    memcpy(this->frame.data, Data, this->frame.data_len);   // 设置数据包
    this->frame.frame_tail = Frame_Tail;                    // 设置帧尾
}

/**
 * @brief 计算帧头校验值（即把所有数据相加）
 * @param data 数据指针
 * @param data_len 数据长度
 * @return 帧头校验值
 */
uint8_t UartMsgCoder::CalculateFrameHead(uint8_t *data, int data_len)
{
    uint8_t sum = 0;
    for (int i = 0; i < data_len; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**                以下为常用的类型转换方法（静态方法）                       **/

// 将float类型转换为4个字节
void UartMsgCoder::FloatToBytes(float value, uint8_t *bytes)
{
    memcpy(bytes, &value, sizeof(float));
}

// 将4个字节转换为float类型
float UartMsgCoder::BytesToFloat(uint8_t *bytes)
{
    float result;
    memcpy(&result, bytes, sizeof(float));
    return result;
}

// 将uint16_t类型转换为2个字节
void UartMsgCoder::Uint16ToBytes(uint16_t value, uint8_t *bytes)
{
    bytes[0] = (uint8_t)(value & 0xFF);
    bytes[1] = (uint8_t)((value >> 8) & 0xFF);
}

// 将2个字节转换为uint16_t类型
uint16_t UartMsgCoder::BytesToUint16(uint8_t *bytes)
{
    return (uint16_t)(bytes[0]) | ((uint16_t)(bytes[1]) << 8);
}


/**
 * @brief 串口接收回调函数 - 复读收到的数据
 */
void UartMsgCoder::EchoRx(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
    // 遍历所有注册的调制器实例，根据huart通道，找到对应的实例
    for (int i = 0; i < UartMsgPointListCount; i++)
    {
        UartMsgCoder *targ_coder = UartMsgPointList[i]; // 取出实例
        if (targ_coder->uart_inst.huart == huart)
        {
            uint8_t data[64] = {0};                                              // 临时数据缓冲区
            memcpy(data, rxData, size);                                  // 复制有效数据
            
            // 复读回去
            targ_coder->SendRawMsg(data, size);
            break;                                                                              // 找到对应实例后跳出循环
        }
    }
}