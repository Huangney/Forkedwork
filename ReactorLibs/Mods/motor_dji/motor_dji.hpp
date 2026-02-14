/**
 * @file motor_dji.hpp
 * @author https://github.com/Huangney
 * @date 2025-9-7
 */
#pragma once
#include "motor_dji_driver.hpp"
#include "loop_ctrler.hpp"
#include "std_math.hpp"
// #include "pids.hpp"
// #include "adrc.hpp"

#define ABS(x) ((x > 0) ? (x) : (-x))


typedef enum
{
	NoneC,
	SpeedC,
	PosC,
}MotorDJIMode;


typedef enum
{
	Realtime,	// 速度较快，但是存在稳定性问题
	Dynamic,	// 速度适中，稳定性较好
	Fluid,		// 速度较慢，不容易发散
	Stable,		// 速度慢，但负载识别速度慢
	Static,		// 适用于静态负载
}MotorIdentifyIntensity;


/// @brief 电机总成类：C620 / C610
class MotorDJI
{
private:
	/// @brief 电流限幅		
	uint16_t _current_limit = 14800;
	/// @brief 速度限幅 	(减速比前的RPM)
	uint16_t _speed_limit = 20000;
	/// @brief 爬坡率限制	(单位：current/s)
	uint32_t _sloperate = 900000;

	// bool dynamic_identify = false;
	// MotorIdentifyIntensity idtf_intensity = Stable;

	// float idtf_interval = 5.0f;
	// float idtf_coeff = 0.1f;

	/** 	  方法		**/
	/// @brief 电机闭环控制计算
	void _CalcLoop();

	/// @brief 自整定过程
	// void SelfIdentify();
	
public:
	MotorDJI(){};
	MotorDJI_Driver entity;

	/** 	  方法		**/
	void Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id);

	// void Dynamicle(MotorIdentifyIntensity intensity);

	void SetSpeed(float rpm, float redu_ratio = 19.0f);			// 3508的默认减速比（用2006的时候记得改！）
	void SetPos(float pos);
	void Neutral();

	int16_t Control();

	static float AngCodeToRad(uint16_t ang_code);
	static float AmpToICode(uint16_t I_Ampere);

	/** 	控制用变量	**/
	// Pids speed_pid; 					// 速度环PID
	// Pids position_pid; 					// 位置环PID

	LoopCtrler ctrler;						 

	float targ_position = 0;			// 目标位置
	float targ_speed = 0;		    	// 目标速度
	float targ_current = 0;				// 目标电流


	/**		属性类变量	**/
	MotorDJIMode mode = NoneC;	// 电机当前控制模式


	/**		测试用		**/
	// ADRC motor_adrc;			// 电机用的 ADRC 控制器	


	// 假设 b0 = 100, 采样率 1000Hz, 高通截止 4Hz, 记忆时间 2.0秒
	// IVIdentifier g_Identifier = IVIdentifier();	// 惯量辨识器
};



// #ifdef __cplusplus
// }
// #endif
