#include "farcon.hpp"

//使用时只需添加如下两句且引用头文件即可，如现在System.cpp所写：
//Farcon farcon;
//farcon.init(&huart3);


Farcon *Farcon::self_instance = nullptr;

void Farcon::init(UART_HandleTypeDef *huart)
{
    // 注册串口实例
    uart_inst = BSP::UART::Apply(huart);

    // 注册接收回调
    uart_inst.RegisterRx(17, Farcon_RxCallback); 

    // BspUart_InstRegist(&this->uart_inst, huart, 17, BspUartType_DMA, BspUartType_Normal, Farcon_RxCallback);
}
// 回传数据，告诉遥控器当前主控状态以及其要求
void Farcon::Farcon_Back_message(Farcon *farcon_instance)
{
// BspUart_Transmit(&farcon_instance->uart_inst, (uint8_t *)"up70", 4);
}