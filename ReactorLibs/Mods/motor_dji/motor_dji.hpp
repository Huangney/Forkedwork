/**
 * @file motor_dji.hpp
 * @author https://github.com/Huangney
 * @date 2025-9-7
 */
#pragma once
#include "motor_dji_driver.hpp"
#include "std_math.hpp"
#include "pids.hpp"
#include "adrc.hpp"

#define ABS(x) ((x > 0) ? (x) : (-x))

typedef enum
{
	NoneC,
	SpeedC,
	PosC,
}MotorDJIMode;

class MotorDJI : public MotorDJI_Driver
{
private:
	/// @brief 电流限幅		
	uint16_t _current_limit = 14800;
	/// @brief 速度限幅 	(减速比前的RPM)
	uint16_t _speed_limit = 20000;
	/// @brief 爬坡率限制	(单位：current/s)
	uint32_t _sloperate = 900000;
	/// @brief 采样周期 (单位：s)
	float _dt = 0.001f;
	/// @brief 是否已经配置过控制器
	bool _ctrl_configured = false;
	/// @brief 配置掩码，用于追踪配置进度
	uint16_t _config_mask = 0;
	/// @brief 当前配置是否为 ADRC
	bool _is_adrc_config = false;

	/** 	  方法		**/
	/// @brief 电机闭环控制计算
	void _CalcLoop();
	
public:
	MotorDJI(){};

	/** 	  方法		**/
	void Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id);

	void SetDt(float dt) { _dt = dt; }

	/// @brief 配置PID控制器（开启链式调用）
	MotorDJI& ConfigPID();
	/// @brief 配置ADRC控制器（开启链式调用）
	MotorDJI& ConfigADRC();

	// --- 链式调用接口 ---
	MotorDJI& AsSpeedC();
	MotorDJI& AsPosC();
	
	MotorDJI& SPD_PID(float kp, float ki, float kd);
	MotorDJI& SPD_Limit(float i_lim, float out_lim);
	
	MotorDJI& POS_PID(float kp, float ki, float kd);
	MotorDJI& POS_Limit(float i_lim, float out_lim);

	MotorDJI& ADRC_BW(float wo, float wc);
	MotorDJI& ADRC_Inertia(float J, float Kt, float B = 0.0f);

	/// @brief 应用配置并锁定
	void Apply();
	// ------------------

	void SetSpeed(float rpm);
	void SetPos(float pos);

	/// @brief 挂空档
	void Neutral();
	/// @brief 解除空档
	void Uneutral();

	/// @brief 解码后回调，用于实时观测更新
	void PostDecodeCallback() override;

	int16_t Control() override;

	static float AngCodeToRad(uint16_t ang_code);
	static float AmpToICode(float I_Ampere);

	/** 	控制变量	**/
	bool use_adrc = false;				// 是否使用 ADRC 控制
	Pids speed_pid; 					// 速度环PID
	Pids position_pid; 					// 位置环PID

	ADRC motor_adrc;					// 电机用的 ADRC 控制器	

	float targ_position = 0;			// 目标位置
	float targ_speed = 0;		    	// 目标速度
	float targ_current = 0;				// 目标电流

	/**		属性类变量	**/
	MotorDJIMode mode = NoneC;	// 电机当前控制模式
};

