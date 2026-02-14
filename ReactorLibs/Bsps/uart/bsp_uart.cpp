#include "bsp_uart.hpp"
#include "bsp_log.hpp"
#include "string.h"
using namespace BSP::UART;

/// @brief 最大实例数量，取决于实际使用的 UART 数
static constexpr uint8_t max_bspuart_inst_nums = 6;

/// @brief 用于存储 UART 实例的静态数组和计数器
static Instance insts[max_bspuart_inst_nums];

/// @brief 已经创建的实例数量
static uint8_t instance_count = 0;

/**
 * @brief 根据 UART 句柄查找对应的实例
 */
static Instance *FindInstByHuart(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return nullptr;
    }

    for (uint8_t i = 0; i < instance_count; i++)
    {
        if (insts[i].IsUsing(huart))
        {
            return &insts[i];
        }
    }

    return nullptr;
}

/**
 * @brief 申请一个 UART 实例句柄
 */
Handler BSP::UART::Apply(UART_HandleTypeDef *huart)
{
    // 确保不是空的 UART 句柄
    if (huart == NULL)
    {
        BspLog_LogError("[Bsp] Empty UartHandle When Apply!\n");
        return Handler(nullptr);
    }

    // 看看是否已经存在该Uart的实例
    Instance *exist_inst = FindInstByHuart(huart);

    // 如果已经存在，直接返回对应的 Handler
    if (exist_inst != nullptr)
    {
        return Handler(exist_inst);
    }

    // 如果实例数量已达上限，无法创建新实例
    if (instance_count >= max_bspuart_inst_nums)
    {
        BspLog_LogError("[Bsp] Too Many Uart Instance!\n");
        return Handler(nullptr);
    }
    
    // 初始化新实例
    insts[instance_count].Init(huart);

    // 返回新实例的 Handler
    Instance *new_inst = &insts[instance_count];
    instance_count++;
    return Handler(new_inst);
}

Handler::Handler(Instance *inst) : instance_(inst)
{
}

void Handler::Transmit(const uint8_t *data, uint16_t len)
{
    // 确保不是无效调用
    if (instance_ == nullptr)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Empty Handler!\n");
        return;
    }

    instance_->Transmit(data, len);
}

bool Handler::RegisterRx(uint16_t rx_setlen, RxCallback rx_callback)
{
    // 确保不是无效调用
    if (instance_ == nullptr)
    {
        BspLog_LogWarning("[Bsp] Invalid RxRegist, Empty Handler!\n");
        return false;
    }

    return instance_->RegisterRx(rx_setlen, rx_callback);
}

bool Handler::IsValid() const
{
    return (instance_ != nullptr);
}

/***---------    实例部分接口    ---------***/

Instance::Instance()
{
    Init(nullptr);
}

Instance::Instance(UART_HandleTypeDef *huart)
{
    Init(huart);
}

/**
 * @brief 初始化实例，重置所有成员变量
 */
void Instance::Init(UART_HandleTypeDef *huart)
{
    this->huart = huart;
    this->rx_callback = nullptr;
    this->rx_setlen_ = 0;
    this->rx_registered_ = 0;

    this->tx_fifo_.head = 0;
    this->tx_fifo_.tail = 0;
    this->tx_fifo_.sending_len = 0;
    this->tx_fifo_.is_busy = 0;

    memset(this->tx_fifo_.buffer, 0, sizeof(this->tx_fifo_.buffer));
    memset(this->rx_buffer_, 0, sizeof(this->rx_buffer_));
}


bool Instance::IsUsing(UART_HandleTypeDef *target_huart) const
{
    return (this->huart != NULL && this->huart == target_huart);
}

void Instance::Transmit(const uint8_t *data, uint16_t len)
{
    // 确保不是空调用
    if (this->huart == NULL)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Empty Instance!\n");
        return;
    }
    if (data == NULL)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Empty Data Pointer!\n");
        return;
    }
    if (len == 0)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Zero Length!\n");
        return;
    }

    // 将希望发送的数据写入FIFO缓冲区
    for (uint16_t i = 0; i < len; i++)
    {
        // 计算下一个头部位置，检查是否会与尾部重叠（即缓冲区满）
        uint16_t next_head = (uint16_t)((this->tx_fifo_.head + 1U) % BSP_UART_TX_BUF_SIZE);

        // 如果下一个头部位置与尾部重叠，说明缓冲区已满，无法继续写入
        if (next_head == this->tx_fifo_.tail)
        {
            BspLog_LogWarning("[Bsp] UART Tx FIFO Full, Data Lost!\n");
            break;
        }

        // 写入数据并更新头部位置
        this->tx_fifo_.buffer[this->tx_fifo_.head] = data[i];
        this->tx_fifo_.head = next_head;
    }

    // 启用DMA发送（如果当前DMA空闲）
    this->TryStartTxDMA();
}

/**
 * @brief 注册接收回调函数，并启动DMA接收
 * @param rx_setlen 期望接收数据长度，也是DMA的最大长度
 * @param rx_callback 用户定义的接收回调函数
 */
bool Instance::RegisterRx(uint16_t rx_setlen, RxCallback rx_callback)
{
    // 确保参数有效
    if (this->huart == NULL)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, Empty Instance!");
        return false;
    }
    if (rx_callback == nullptr)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, Empty Callback!");
        return false;
    }
    if (rx_setlen == 0)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, Zero Length!");
        return false;
    }

    // 每个实例只允许注册一次接收回调
    if (this->rx_registered_ != 0)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, This Uart Already has RxCallback!");
        return false;
    }

    // 限制设置的接收长度不能超过缓冲区大小
    if (rx_setlen > BSP_UART_RX_BUF_SIZE)
    {
        rx_setlen = BSP_UART_RX_BUF_SIZE;
    }

    // 保存回调函数和期望接收长度，并标记为已注册
    this->rx_callback = rx_callback;
    this->rx_setlen_ = rx_setlen;
    this->rx_registered_ = 1;

    // 启动DMA接收
    memset(this->rx_buffer_, 0, sizeof(this->rx_buffer_));
    if (HAL_UARTEx_ReceiveToIdle_DMA(this->huart, this->rx_buffer_, this->rx_setlen_) != HAL_OK)
    {
        BspLog_LogError("[Bsp] Try Enable HAL_IdleRxCallback_DMA, but Failed!");
        this->rx_registered_ = 0;
        this->rx_setlen_ = 0;
        this->rx_callback = nullptr;
        return false;
    }

    // 关闭DMA半满中断，避免在接收过程中被过早触发
    if (this->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
    }

    return true;
}

/**
 * @brief 尝试启动DMA发送，如果当前DMA空闲且FIFO中有数据
 */
void Instance::TryStartTxDMA()
{
    if (this->huart == NULL)
    {
        BspLog_LogWarning("[Bsp] Unable to Start Tx DMA, Empty Instance!\n");
        return;
    }

    if (this->tx_fifo_.is_busy != 0)
    {
        // DMA正在发送中，等待当前发送完成后会自动检查并继续发送剩余数据
        return;
    }

    if (this->tx_fifo_.head == this->tx_fifo_.tail)
    {
        // FIFO中没有数据需要发送
        return;
    }

    // 计算本次最大连续发送长度（处理环形回绕）
    uint16_t send_len = 0;

    // 如果头部在尾部前面，直接计算差值
    if (this->tx_fifo_.head > this->tx_fifo_.tail)
    {
        send_len = this->tx_fifo_.head - this->tx_fifo_.tail;
    }
    // 如果头部在尾部后面，说明环形回绕了，需要计算从尾部到缓冲区末尾的长度
    else
    {
        send_len = BSP_UART_TX_BUF_SIZE - this->tx_fifo_.tail;
    }

    // 启动DMA发送
    this->tx_fifo_.sending_len = send_len;

    if (HAL_UART_Transmit_DMA(this->huart, &this->tx_fifo_.buffer[this->tx_fifo_.tail], send_len) == HAL_OK)
    {
        this->tx_fifo_.is_busy = 1;
    }
    else
    {
        BspLog_LogError("[Bsp] Failed to Start HAL_Tx_DMA!\n");
        this->tx_fifo_.sending_len = 0;
    }
}

/**
 * @brief HAL UART DMA发送完成回调入口，更新FIFO状态并尝试发送剩余数据
 */
void Instance::OnTxCplt()
{
    // 确保不是空调用
    if (this->huart == NULL)
    {
        BspLog_LogError("[Bsp] Meet Empty UartInst in TxCplt Callback!\n");
        return;
    }

    // 更新FIFO状态，释放已发送的数据空间
    this->tx_fifo_.tail = (uint16_t)((this->tx_fifo_.tail + this->tx_fifo_.sending_len) % BSP_UART_TX_BUF_SIZE);
    this->tx_fifo_.sending_len = 0;

    // 检查FIFO中是否还有数据需要发送，如果有则继续发送
    if (this->tx_fifo_.head == this->tx_fifo_.tail)
    {
        this->tx_fifo_.is_busy = 0;
        return;
    }

    // 计算下一段连续数据长度（处理环形回绕）
    uint16_t send_len = 0;
    if (this->tx_fifo_.head > this->tx_fifo_.tail)
    {
        send_len = this->tx_fifo_.head - this->tx_fifo_.tail;
    }
    else
    {
        send_len = BSP_UART_TX_BUF_SIZE - this->tx_fifo_.tail;
    }

    // 启动下一段DMA发送
    this->tx_fifo_.sending_len = send_len;
    if (HAL_UART_Transmit_DMA(this->huart, &this->tx_fifo_.buffer[this->tx_fifo_.tail], send_len) == HAL_OK)
    {
        this->tx_fifo_.is_busy = 1;
    }
    else
    {
        this->tx_fifo_.sending_len = 0;
        this->tx_fifo_.is_busy = 0;
    }
}

void Instance::OnRxEvent(uint16_t size)
{
    if (this->huart == NULL || this->rx_registered_ == 0 || this->rx_callback == nullptr)
    {
        return;
    }

    uint16_t real_size = size;
    if (real_size > this->rx_setlen_)
    {
        real_size = this->rx_setlen_;
    }

    this->rx_callback(this->huart, this->rx_buffer_, real_size);

    memset(this->rx_buffer_, 0, this->rx_setlen_);
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, this->rx_buffer_, this->rx_setlen_);
    if (this->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
    }
}



/****-------    覆写HAL库的中断函数     -------****/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Instance *inst = FindInstByHuart(huart);
    if (inst == nullptr)
    {
        return;
    }

    inst->OnRxEvent(Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Instance *inst = FindInstByHuart(huart);
    if (inst == nullptr)
    {
        return;
    }

    inst->OnTxCplt();
}