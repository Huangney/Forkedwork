#ifndef MOTOR_VESC
#define MOTOR_VESC
#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_can.h"
#include "can.h"


/// @brief 电调上报的操作数ID类型
typedef enum
{
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2,
    CAN_PACKET_STATUS_3,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_PING,
    CAN_PACKET_PONG,
    CAN_PACKET_DETECT_APPLY_ALL_FOC,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_FOC_ERPMS,
    CAN_PACKET_CONF_STORE_FOC_ERPMS,
    CAN_PACKET_STATUS_5
} CanPacketType;

typedef struct MotorVescRecvData
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t recv_data[8];
} MotorVescRecvData;

class MotorVESC
{
public:
    /** 	  属性		**/
    int motor_id;                  // 总线上的电机编号，第几个
    int motor_can_id;              // 总线上的电机的CANID
    float motor_duty_real;         // 电机实际占空比
    int32_t motor_rpm_real;        // 电机实际每分钟转速
    float motor_current_real;         //读取电机的实际电流
    uint8_t canTx_text[8];         //保存发送数据
    CAN_HandleTypeDef *targ_can_n; // 哪一条CAN总线
    BspCan_Instance bspcan_inst; // 电机的CAN实例


    /** 	  方法		**/
    // 初始化MotorVESC结构体,同时注册can实例
    void Init( CAN_HandleTypeDef *can_n, int motor_id, int motor_can_id);
    // 设置电机占空比
    void SetDuty( float duty);
    // 设置电机转速
    void SetRPM(int RPM,float limit_mx);
    // 获取电机转速
    int GetRPM(int motor_id);
    // 收到的信息并回复电机
    void RxHandle(MotorVescRecvData vesc_recvs);
};


#ifdef __cplusplus
}
#endif
#endif 
