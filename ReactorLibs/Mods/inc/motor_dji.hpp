/**
 * @file motor_dji.hpp
 * @author https://github.com/Huangney
 * @date 2025-9-7
 */
#pragma once

// #ifdef __cplusplus
// extern "C" {
// #endif

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "pids.hpp"
#include "adrc.hpp"

#define ABS(x) ((x > 0) ? (x) : (-x))
#define Lim_ABS(x, y) \
if (x > y) 			\
{		   			\
	x = y; 			\
}					\
else if (x < -y) 	\
{					\
	x = -y;			\
}

// 电机反馈信息结构体变量定义
typedef struct
{
	int16_t speed_rpm;
	int16_t real_current;
	uint8_t temprature;			
	uint16_t angle;		 		// 范围为[0, 8191]
	uint16_t last_angle;
	uint16_t offset_angle;
	int32_t round_cnt;
	int32_t total_angle;
	uint32_t msg_cnt;
} moto_measure_t;


typedef enum
{
	PID_PosControl,
	PID_SpeedControl,
	ADRC_PosControl,
	ADRC_SpeedControl,
	None_Control,
	Identify_Mode,
}MotorDJIMode;

typedef enum
{
	Realtime,	// 速度较快，但是存在稳定性问题
	Dynamic,	// 速度适中，稳定性较好
	Fluid,		// 速度较慢，不容易发散
	Stable,		// 速度慢，但负载识别速度慢
	Static,		// 适用于静态负载
}MotorIdentifyIntensity;


/// @brief 电机类：C620 / C610
class MotorDJI
{
private:
	friend void _MotorDJI_DecodeMeasure(MotorDJI* motor_p, uint8_t *Data);
	// 是否启用电机控制，若为否，发送0电流指令或不发送指令
	bool _enabled = false;
	uint8_t _motor_id = 0;			// 电机ID（1 ~ 8）

	/// @brief 电机是否在线（私有）
	bool _online_priv = false;
	int _online_cnt = 0; 
	/// @brief 电流限幅		
	uint16_t _current_limit = 14800;
	/// @brief 速度限幅 	(减速比前的RPM)
	uint16_t _speed_limit = 20000;
	/// @brief 爬坡率限制	(单位：current/s)
	uint32_t _sloperate = 900000;
	

	/// @brief 前10次接收数据的平均时间间隔(最小单位：0.1ms，uint16精度)，用于判断电机在线质量
	uint32_t _recv_tick = 0;
	uint32_t _recv_sum_interval = 0;
	uint16_t _recv_interval[10] = {0};
	uint8_t _recv_last_index = 0;
	bool _recv_looped = false;
	/// @brief 根据前十次间隔计算的到的频率 (单位Hz)
	float _recv_freq = 0.0f;

	/// @brief 电机转速低通滤波系数（转速的低通滤波会导致严重的延迟）
	float _read_rpm_lpf_rate = 1.0f; 
	/// @brief 电机电流低通滤波系数
	float _read_current_lpf_rate = 0.5f; 

	bool dynamic_identify = false;
	MotorIdentifyIntensity idtf_intensity = Stable;

	float idtf_interval = 5.0f;
	float idtf_coeff = 0.1f;

	/** 	  方法		**/
	/// @brief 电机速度环控制 
	void _MotorDJI_SpeedLoop();
	void _MotorDJI_ADRCSpdLoop();

	/// @brief 电机位置环控制
	void _MotorDJI_PosLoop();
	void _MotorDJI_ADRCPosLoop();

	/// @brief 自整定过程
	void SelfIdentify();

	float _GetDelayedCurrent(uint8_t delay_tick);		// 获取历史电流值，用于系统环路延时补偿

	/// @brief 获取电机所在的CAN段，用于发送
	uint8_t _GetCanSeg(uint8_t motor_id);
	
public:
	MotorDJI(){};

	/** 	  方法		**/
	void Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id, MotorDJIMode djimode);
	void Dynamicle(MotorIdentifyIntensity intensity);
	void SwitchMode(MotorDJIMode new_mode);
	void SetSpeed(float rpm, float redu_ratio = 19.0f);			// 3508的默认减速比（用2006的时候记得改！）
	void SetPos(float pos);
	void Neutral();
	void Disable();
	void Enable();
	bool IsEnabled();											// 返回电机控制是否启用
	int16_t Control();

	/** 	静态方法	**/
	static void ControlAllMotors();
		


	/** 	信息流变量	**/
	/// @brief 电机反馈信息结构体
	moto_measure_t measure;	

	/// @brief 电机是否在线（公开但只读）
	const bool& online = _online_priv;

	const bool& enabled = _enabled;

	/// @brief 电机的CAN实例
	BspCan_Instance bspcan_inst;

	/// @brief 当前电机所在的CAN段，用于发送电流 注：0-3分别对应CAN1的1-4号电机，5-8号电机，CAN2的1-4号电机，5-8号电机
	uint8_t at_can_seg = 0;
	
	/** 	控制用变量	**/
	Pids speed_pid; 					// 速度环PID
	Pids position_pid; 					// 位置环PID
	float targ_position = 0;			// 目标位置
	float targ_speed = 0;		    	// 目标速度
	float targ_current = 0;				// 目标电流

	float history_current[5] = {0.0f};	// 历史电流值，用于系统环路延时补偿
	uint8_t history_index = 0;			// 历史电流值下标(0 ~ 4)

	/**		属性类变量	**/
	MotorDJIMode mode = None_Control;	// 电机当前控制模式


	/**		测试用		**/
	// KalmanObserver<1, 3, 1> kalman_ob;		// 卡尔曼观测器
	// float kalman_rpm = 0.0f;				// 卡尔曼观测器估计的转速

	ADRC motor_adrc;			// 电机用的 ADRC 控制器	


	// 假设 b0 = 100, 采样率 1000Hz, 高通截止 4Hz, 记忆时间 2.0秒
	IVIdentifier g_Identifier = IVIdentifier(19.62, 1000.0f, 4.0f, 1.0f);	// 惯量辨识器

	SquareInjector square_injector_; // 方波激励器
};

namespace MotorDJIConst
{
	const static float redu_M3508 = 19.0f;	// M3508电机的减速比
	const static float redu_M2006 = 36.0f;	// M2006电机的减速比
}

typedef MotorDJI MotorC610;		// 本库对C620和C610的支持是通用的
typedef MotorDJI MotorC620;		



// #ifdef __cplusplus
// }
// #endif
