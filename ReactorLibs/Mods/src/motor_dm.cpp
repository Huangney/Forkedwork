#include "motor_dm.hpp"
#include "string.h"

// 达妙用数据,使能
const uint8_t DM_ENBUF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
// 电机旋转的速度
#define DM_HalfRs 1.57        // 每秒半圈
#define DM_OneRs 3.14        // 每秒一圈
#define DM_TwoRs 6.28        // 每秒两圈
#define DM_PI 3.1415926f

// 不同的消息类型，需要不同的ID偏置量
#define DM_MESTYPE_INIT 0x000
#define DM_MESTYPE_POSITION 0x100
void MotorDM_RxCallBack(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, CAN_HandleTypeDef *hcan);

// 用于解码反馈信息的参数
#define DM_POS_MAX 12.5f        // 位置最大值，单位：弧度
#define DM_SPD_MAX 30.0f        // 速度最大值，单位：弧度/秒
#define DM_TOQ_MAX 10.0f        // 扭矩最大值，单位：Nm


// 最多支持同时驱动6个达妙电机（可以自己改大，一般用不到这么多）
MotorDM *MotorDM_List[6] = {nullptr};
uint8_t MotorDM_Count = 0;  // 记录已经注册的达妙电机数量

/**
 * @brief 初始化达妙电机
 * @param hcan 使用的CAN总线
 */
void MotorDM::Init(CAN_HandleTypeDef* hcan, uint8_t motor_id, uint8_t master_id)
{
  // 记录数据
  this->motor_id = motor_id;
  this->master_id = master_id;
  
  // 初始化（即注册）该电机的接收CAN实例
	BspCan_InstRegist(&bspcan_inst, hcan, master_id, motor_id, 0, 0, MotorDM_RxCallBack);

  // 记录到全局列表
  if (MotorDM_Count < 6)
  {
    MotorDM_List[MotorDM_Count++] = this;
  } 
}

/**
 * @brief 使能电机
 */
void MotorDM::Enable()
{
  // 发送使能指令
  BspCan_TxConfig txconf = BspCan_GetTxConfigFast(bspcan_inst.hcan, motor_id + DM_MESTYPE_INIT);
  BspCan_Transmit(txconf, (uint8_t*)DM_ENBUF);
}

/**
 * @brief 设置位置（限制默认速度）
 */
void MotorDM::SetPosi(float pos)
{
  posi = pos;

  

  // 获取数据指令
  uint8_t buf[8];
  // 如果限制了电机的转动，启用lim
  if (posi_max != 0 || posi_min != 0)
  {
    posi = posi > posi_max ? posi_max : posi;
    posi = posi < posi_min ? posi_min : posi;
    DM_GetPosiSpdBufLim(buf, posi, DM_OneRs, posi_max, posi_min);
  }
  else
  {
    DM_GetPosiSpdBuf(buf, posi, DM_OneRs);
  }
  
  // 发送数据
  BspCan_TxConfig txconf = BspCan_GetTxConfigFast(bspcan_inst.hcan, motor_id + DM_MESTYPE_POSITION);
  BspCan_Transmit(txconf, buf);
}

/**
 * @brief 设置位置（限制给定速度）
 */
void MotorDM::SetPosi(float pos, float spd)
{
  posi = pos;

  // 获取数据指令
  uint8_t buf[8];
  // 如果限制了电机的转动，启用lim
  if (posi_max != 0 || posi_min != 0)
  {
    posi = posi > posi_max ? posi_max : posi;
    posi = posi < posi_min ? posi_min : posi;
    DM_GetPosiSpdBufLim(buf, posi, spd, posi_max, posi_min);
  }
  else
  {
    DM_GetPosiSpdBuf(buf, posi, spd);
  }

  // 发送数据
  BspCan_TxConfig txconf = BspCan_GetTxConfigFast(bspcan_inst.hcan, motor_id + DM_MESTYPE_POSITION);
  BspCan_Transmit(txconf, buf);
}


/**
 * @brief 设置位置限制
 */
void MotorDM::SetPosiLim(float posi_max, float posi_min)
{
  this->posi_max = posi_max;
  this->posi_min = posi_min;
}


/**
 * @brief 处理达妙电机反馈信息的回调函数
 */
void MotorDM_RxCallBack(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, CAN_HandleTypeDef *hcan)
{
  // 不同的电机，反馈的MasterID不同，所以需要根据ID来区分
	uint8_t targ_master_id = ((RxHeader->StdId) & 0xff);
  
  MotorDM *targ_motor = nullptr;

	// 利用全局的电机实例列表 来获取对应的电机实例
  for (uint8_t i = 0; i < MotorDM_Count; i++)
  {
    targ_motor = MotorDM_List[i];
    if (targ_motor->master_id == targ_master_id) break;
  }

  // 解析数据
  DM_DecodeMsg(targ_motor, RxData);
}


/******************    库内部的STATIC函数    *****************************/
/**
 * @brief 线性映射uint16_t到float
 * @param 
 */
static float DM_LinearRef(uint16_t code, float max)
{
  float min = -max;
  return (float)code / 65535.0f * (max - min) + min;
}


/**
 * @name 获得位置速度数组
 * @details 输入位置和速度（均为RAD），获得一个可以发送给达妙电机的数组
 */
static void DM_GetPosiSpdBuf(uint8_t buf[], float posi, float speed)
{
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&posi;
	vbuf=(uint8_t*)&speed;

    buf[0] = *pbuf;
    buf[1] = *(pbuf+1);
    buf[2] = *(pbuf+2);
    buf[3] = *(pbuf+3);
    
    buf[4] = *vbuf;
    buf[5] = *(vbuf+1);
    buf[6] = *(vbuf+2);
    buf[7] = *(vbuf+3);
}

/**
 * @name 获得位置速度数组
 * @details 输入位置和速度（均为RAD），获得一个可以发送给达妙电机的数组
 */
static void DM_GetPosiSpdBufLim(uint8_t buf[], float posi, float speed, float lim_H, float lim_L)
{
    if (posi > lim_H)
    {
      posi = lim_H;
    }
    if (posi < lim_L)
    {
      posi = lim_L;
    }
    
    
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&posi;
	  vbuf=(uint8_t*)&speed;


    buf[0] = *pbuf;
    buf[1] = *(pbuf+1);
    buf[2] = *(pbuf+2);
    buf[3] = *(pbuf+3);
    
    buf[4] = *vbuf;
    buf[5] = *(vbuf+1);
    buf[6] = *(vbuf+2);
    buf[7] = *(vbuf+3);
}

/**
 * @brief 解析达妙回传数据
 * @note 
 */
static void DM_DecodeMsg(MotorDM* motor, uint8_t buf[])
{
  // 解析数据
  memcpy(motor->recv_buf, buf, 8);
  uint8_t testmotor_id = (buf[0] & 0x0f);

  // 验证是否进入了正确的回调
  if (testmotor_id != (motor->motor_id & 0x0f))
  {
    motor->error_recv++;
    return;
  }

  // 解析错误码
  uint8_t err_code = buf[0] >> 4;

  // 解析POS（位置）（经测试，映射关系为：零位时，输出32767；每转动PI，Code + 8235,；每转动-PI，Code - 8235）
  int16_t pos_code = ((buf[1] << 8) | buf[2]);
  motor->real_pos = DM_LinearRef(pos_code, DM_POS_MAX);
  
  // 解析SPD（速度）（12位）
  int16_t spd_code = (buf[3] << 4) | (buf[4] >> 4);
  motor->real_spd = DM_LinearRef(spd_code, DM_SPD_MAX);

  // 解析扭矩信息 （12位）
  int16_t torq_code = ((buf[4] & 0x0f) << 8) | buf[5];
  motor->real_torque = DM_LinearRef(torq_code, DM_TOQ_MAX);

  // 解析电机MOSFET温度
  uint8_t temp_mos = buf[6];

  // 解析电机绕组温度
  uint8_t temp_wind = buf[7];

  // 存储数据
  // motor->err_code = err_code;
  // motor->pos_code = pos_code;
  // motor->spd_code = spd_code;
  // motor->torq_code = torq_code;
  // motor->temp_mos = temp_mos;
  // motor->temp_wind = temp_wind;
}