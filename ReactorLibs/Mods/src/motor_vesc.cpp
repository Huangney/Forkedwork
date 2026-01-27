#include "motor_vesc.hpp"
#include "bsp_can.h"
#include <string.h>
#include "can.h"
#include "std_math.hpp"


int motor_can_id_list[16] = {0}, motor_can_id_count = 0,motor_count=0;//用来存储vesc的can_id，同时motor_id一定要从0开始，按顺序0—15进行初始化，不要跳号
static MotorVESC* MotorList[16]= {nullptr};//用来存储电机实例指针，便于回调函数查表处理

// 发送CAN消息
void MotorVESC_SendCanTXBuffer(MotorVESC *motor,CanPacketType cmd_type, float values);
void Motor_vesc_RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData, CAN_HandleTypeDef *hcan);


/// @brief 接收回调函数
/// @param RxHeader 
/// @param RxData 
 void Motor_vesc_RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData, CAN_HandleTypeDef *hcan)
{
    
    MotorVescRecvData vesc_rx;
    vesc_rx.rx_header = *rxHeader;
    memcpy(vesc_rx.recv_data,rxData,8);

    for (int  i = 0; i < motor_can_id_count; i++)
    {
        if (motor_can_id_list[i]==vesc_rx.rx_header.ExtId & 0xff)
        {
           MotorVESC* targ_motor_vesc = MotorList[i];
           targ_motor_vesc->RxHandle(vesc_rx);
           break;
        }
    }
}


/// @name GetRPM
/// @brief 返回指定标签电机的真实转速
/// @param motor_id ：电机编号
/// @return 该电机的实际转速
int MotorVESC::GetRPM(int motor_id)
{
    if (motor_id == this->motor_id)
    {
        
        return (this->motor_rpm_real);
    }

    else return 0;
}

/// @name Init
/// @brief 初始化MotorVESC结构体,同时注册can实例
void MotorVESC::Init( CAN_HandleTypeDef *can_n, int motor_id, int motor_can_id)
{
    this->motor_id = motor_id;
    this->motor_can_id = motor_can_id;
    this->targ_can_n = can_n;
    this->motor_duty_real = 0;
    this->motor_rpm_real = 0;
    this->motor_current_real = 0;
    motor_can_id_list[motor_can_id_count++] = motor_can_id;
    MotorList[motor_count++] = this;
    uint32_t motor_rx_id = ((CAN_PACKET_STATUS << 8) | this->motor_can_id);
    uint32_t motor_tx_id = ((CAN_PACKET_SET_RPM << 8) | this->motor_can_id); 
    BspCan_InstRegist(&this->bspcan_inst, this->targ_can_n, motor_rx_id, motor_tx_id, 1, 1, Motor_vesc_RxCallback);
}


/**
 * @name RxHandle
 * @brief 这个函数处理了收到的信息并回复电机
 * @note 
 * @details 本函数内完成了回复电机（设置电机速度等）的操作；如果有需要的操作可以在此更改
 * @warning 
 */
void MotorVESC::RxHandle(MotorVescRecvData vesc_recvs)
{

    // 获取电调上报的消息类型
    CanPacketType vesc_status_type = (CanPacketType)(vesc_recvs.rx_header.ExtId >> 8);
    // 解码
    switch (vesc_status_type)
    {
        case CAN_PACKET_STATUS: // 第一类上报
        {
            this->motor_duty_real = (vesc_recvs.recv_data[6] * 256 + vesc_recvs.recv_data[7]) / 1000.0;

            this->motor_current_real = (vesc_recvs.recv_data[4] * 256 + vesc_recvs.recv_data[5]) / 10.0;
            
            int32_t temp = (int32_t)(vesc_recvs.recv_data[0] << 24 | vesc_recvs.recv_data[1] << 16
                | vesc_recvs.recv_data[2] << 8 | vesc_recvs.recv_data[3]);
                
            this->motor_rpm_real = temp;
            break;
        }
        default:
        break;
    }
}


// 设置电机转速
void  MotorVESC::SetRPM(int RPM,float limit_mx)
{
    int rpm = Limit_ABS((float)RPM,limit_mx);
   MotorVESC_SendCanTXBuffer( this,CAN_PACKET_SET_RPM, rpm);
}


// 设置电机占空比
void  MotorVESC::SetDuty( float duty)
{
  MotorVESC_SendCanTXBuffer(this,CAN_PACKET_SET_DUTY, duty);
}


// 发送CAN消息
void  MotorVESC_SendCanTXBuffer(MotorVESC *motor,CanPacketType cmd_type, float values)
{
int ExtId=(cmd_type << 8 | motor->motor_can_id);

  BspCan_TxConfig custom_tx_conf=BspCan_GetTxConfig(motor->targ_can_n,ExtId,1,0,8,50);
    uint8_t txbuf[8] = {0};
    switch (cmd_type)
    {
        case CAN_PACKET_SET_DUTY:
        {
            int32_t data;
            data = (int32_t)(values * 100000);
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        case CAN_PACKET_SET_RPM:
        {
            int32_t data;
            data = (int32_t)(values) ;
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        default:
            break;
    }
   BspCan_Transmit(custom_tx_conf,txbuf);
	memcpy(motor->canTx_text,txbuf,8);
}



