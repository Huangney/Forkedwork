/**
 * @author 328dragon
 * @note 遥控器接收端代码
 */

#ifndef _FARCON_HPP_
#define _FARCON_HPP_

#include "bsp_uart.hpp"

#define pos_data_length 10
class Farcon
{

    // 与通信相关的，较为私密
private:
    uint8_t toggle_origin_data;
    uint8_t button_first_half_origin;
    uint8_t button_second_half_origin;
    uint8_t dest_kfs_origin[3];

    static Farcon *self_instance;

    int jy_data_conver(int jy_data)
    {
        if (jy_data > 100)
        {
            jy_data = jy_data - 256;
        }
        return jy_data;
    }

    void DecodeData(uint8_t *rxData)
    {

        toggle_origin_data = rxData[4];
        button_first_half_origin = rxData[5];
        button_second_half_origin = rxData[6];

        dest_kfs_origin[0] = rxData[7];
        dest_kfs_origin[1] = rxData[8];
        dest_kfs_origin[2] = rxData[9];

        jy_data_origin[0] = jy_data_conver(rxData[10]);
        jy_data_origin[1] = jy_data_conver(rxData[11]);
        jy_data_origin[2] = jy_data_conver(rxData[12]);
        jy_data_origin[3] = jy_data_conver(rxData[13]);

        for (int i = 0; i < 4; i++)
        {
            toggle[i] = (toggle_origin_data >> i) & 0x01;
        }
        for (int i = 0; i < 8; i++)
        {
            button_first_half[i] = (button_first_half_origin >> i) & 0x01;
            button_second_half[i] = (button_second_half_origin >> i) & 0x01;
        }
        for (int i = 0; i < 12; i++)
        {
            if (i < 4)
                KFS_values[i] = dest_kfs_origin[0] >> (2 * i) & 0x03;
            else if (i < 8)
                KFS_values[i] = dest_kfs_origin[1] >> (2 * (i - 4)) & 0x03;
            else
                KFS_values[i] = dest_kfs_origin[2] >> (2 * (i - 8)) & 0x03;
        }
    }
    // 只与数据有关，安全
public:
    uint32_t timestamp;
    BSP::UART::Handler uart_inst;

    Farcon()
    {
        self_instance = this; 
        timestamp = 0;
    }
    static void Farcon_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
    {
        // 校验包头和包尾
        if (rxData[0] == 0x75 && rxData[1] == 0x70 && rxData[2] == 0x37 && rxData[3] == 0x30 &&
            rxData[pos_data_length + 4] == 0x36 &&
            rxData[pos_data_length + 5] == 0x36 &&
            rxData[pos_data_length + 6] == 0x36)
        {
            // 如果对象存在，调用它的解码函数
            if (self_instance != nullptr)
            {
                self_instance->DecodeData(rxData);
            }
        }
    }

    //用户只需要关心如下数据
    uint8_t toggle[4];//拨杆的状态，0表示外位置，1表示内位置（拨杆向板外、向板内）
    uint8_t button_first_half[8];//KEY0到KEY8的状态，按下为1，松开是0
    uint8_t button_second_half[8];//KEY9到KEY16的状态，按下为1，松开是0
    uint8_t KFS_values[12];// KFS值分为四个：0表示初始状态，1表示R1块，2表示R2块，3表示假块
    int jy_data_origin[4];//摇杆映射值，都是从-100到100，依次是LX,LY,RX,RY
    void init(UART_HandleTypeDef *huart);
    void Farcon_Back_message(Farcon *farcon_instance);

    ~Farcon() {};
};

#endif
