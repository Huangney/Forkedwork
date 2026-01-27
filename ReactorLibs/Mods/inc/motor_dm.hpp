#ifndef MOTOR_DM_HPP
#define MOTOR_DM_HPP

#include "stm32f4xx_hal.h"
#include "bsp_can.h"


//   // 解析错误码
//   uint8_t err_code = buf[0] >> 4;

//   // 解析POS（位置）
//   int16_t pos_code = (buf[1] << 8) | buf[2];
  
//   // 解析SPD（速度）（12位）
//   int16_t spd_code = (buf[3] << 4) | (buf[4] >> 4);

//   // 解析扭矩信息 （12位）
//   int16_t toq_code = ((buf[4] & 0x0f) << 8) | buf[5];

//   // 解析电机MOSFET温度
//   uint8_t temp_mos = buf[6];

//   // 解析电机绕组温度
//   uint8_t temp_wind = buf[7];

// 一个达妙电机类
class MotorDM
{   
    private:
        // 接收数据相关
        // uint8_t err_code;
        // int16_t pos_code;
        // int16_t spd_code;
        // int16_t torq_code;
        // uint8_t temp_mos;
        // uint8_t temp_wind;

    public:
        MotorDM() {};
        ~MotorDM() {};

        uint8_t motor_id = 0;              // 电机ID
        uint8_t master_id = 0;             // 主控ID
        float posi = 0;              // 位置，单位：弧度
        float posi_max =  0;         // 位置上限，单位：弧度
        float posi_min =  0;         // 位置下限，单位：弧度
        uint8_t recv_buf[8] = {0}; 
        uint32_t error_recv = 0;      // 接收错误计数
        bool enabled = false;            // 是否使能
        BspCan_Instance bspcan_inst;	// 电机的CAN实例

        float real_pos = 0;         // 实际位置，单位：弧度
        float real_spd = 0;         // 实际速度，单位：弧度/秒
        float real_torque = 0;      // 实际扭矩，单位：Nm

        
        // 初始化
        void Init(CAN_HandleTypeDef* hcan, uint8_t motor_id, uint8_t master_id);
        // 使能
        void Enable();
        // 设置位置
        void SetPosi(float pos);
        // 设置位置和速度
        void SetPosi(float pos, float spd);
        // 设置位置限制
        void SetPosiLim(float posi_max, float posi_min);
};







static float DM_LinearRef(uint16_t code, float max);
static void DM_GetPosiSpdBuf(uint8_t buf[], float posi, float speed);
static void DM_GetPosiSpdBufLim(uint8_t buf[], float posi, float speed, float lim_H, float lim_L);
static void DM_DecodeMsg(MotorDM* motor, uint8_t buf[]);

#endif