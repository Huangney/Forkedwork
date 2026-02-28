/**
 * @file motor_dji.cpp
 * @brief C620 / C610 电机控制类实现文件，我们战队不用GM6020（笑）
 * @author Huangney
 * @date 2025-8-21
 */
#include "motor_dji.hpp"
#include "RtosCpp.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_log.hpp"
#include "string.h"
#include "stdio.h"

void MotorDJI_SlopeLim(float& cur, float& cur_temp, float slope_value, float dt);
/**
 * @brief C620 / C610 电机额外初始化
 * @param Esc_Id 电机 ID（请查看C620 / C610说明书，注意其从 1 开始！）
 */
void MotorDJI::Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id)
{
	MotorDJI_Driver::Init(hcan, motorESC_id); 	// 初始化电机实体
}

/// @brief 设置速度
/// @param rpm 
void MotorDJI::SetSpeed(float rpm)
{
	if (mode != SpeedC) return; 	// 不是速度模式就不执行
	targ_speed = StdMath::fclamp(rpm, (float)_speed_limit); // 应用速度限幅
}
/// @brief 设置位置
/// @param pos 
void MotorDJI::SetPos(float pos)
{
	if (mode != PosC) return; 	// 不是位置模式就不执行
	targ_position = pos;
}

/**
 * @brief 空档
 * @details 将电机设置为空档状态，强制要求电机不输出任何力矩
 * 类似于汽车的空挡
 * @warning 该函数会将控制模式切换为 None_Control, 使用后，需要重新设置控制模式
 */
void MotorDJI::Neutral()
{
	targ_current = 0;
	mode = NoneC;
}

void MotorDJI::Uneutral()
{
	mode = SpeedC; 					// 默认恢复速度模式
	speed_pid.Reset();				// 重置状态防止积分跳跃
	position_pid.Reset();
}

MotorDJI& MotorDJI::ConfigPID()
{
	_ctrl_configured = false; 	// 重新配置时先锁定
	_config_mask = 0;
	_is_adrc_config = false;
	use_adrc = false;
	return *this;
}

MotorDJI& MotorDJI::ConfigADRC()
{
	_ctrl_configured = false;
	_config_mask = 0;
	_is_adrc_config = true;
	use_adrc = true;
	return *this;
}

MotorDJI& MotorDJI::AsSpeedC()
{
	mode = SpeedC;
	_config_mask |= (1 << 0);
	return *this;
}

MotorDJI& MotorDJI::AsPosC()
{
	mode = PosC;
	_config_mask |= (1 << 0);
	return *this;
}

MotorDJI& MotorDJI::SPD_PID(float kp, float ki, float kd)
{
	speed_pid.Init(kp, ki, kd);
	_config_mask |= (1 << 1);
	return *this;
}

MotorDJI& MotorDJI::SPD_Limit(float i_lim, float out_lim)
{
	speed_pid.SetLimit(i_lim, out_lim, 0.9f);
	_config_mask |= (1 << 2);
	return *this;
}

MotorDJI& MotorDJI::POS_PID(float kp, float ki, float kd)
{
	position_pid.Init(kp, ki, kd);
	_config_mask |= (1 << 3);
	return *this;
}

MotorDJI& MotorDJI::POS_Limit(float i_lim, float out_lim)
{
	position_pid.SetLimit(i_lim, out_lim, 0.9f);
	_config_mask |= (1 << 4);
	return *this;
}

MotorDJI& MotorDJI::ADRC_BW(float wo, float wc)
{
	motor_adrc.kp = wc * wc;
	motor_adrc.kd = 2.0f * wc;
	motor_adrc.eso.UpdateOmega_O(wo);
	_config_mask |= (1 << 5);
	return *this;
}

MotorDJI& MotorDJI::ADRC_Inertia(float J, float Kt, float B)
{
	motor_adrc.J = J;
	motor_adrc.Kt = Kt;
	motor_adrc.B = B;
	// 重新计算相关的 b0 指标
	motor_adrc.eso.J = J;
	motor_adrc.eso.b0 = Kt / J;
	_config_mask |= (1 << 6);
	return *this;
}

void MotorDJI::Apply()
{
	if (!(_config_mask & (1 << 0))) {
		BspLog_LogError("MotorDJI: Mode not specified (AsSpeedC/AsPosC)!");
		return;
	}

	if (!_is_adrc_config) {
		// --- PID 校验 ---
		if (mode == SpeedC) {
			if (!(_config_mask & (1 << 1)) || !(_config_mask & (1 << 2))) {
				BspLog_LogError("MotorDJI: Speed Loop PID or Limit not set!");
				return;
			}
		} else if (mode == PosC) {
			bool pos_ok = (_config_mask & (1 << 3)) && (_config_mask & (1 << 4));
			bool spd_ok = (_config_mask & (1 << 1)) && (_config_mask & (1 << 2));
			if (!pos_ok || !spd_ok) {
				BspLog_LogError("MotorDJI: Cascade PID requires BOTH Pos and Speed loop config!");
				return;
			}
		}
	} else {
		// --- ADRC 校验 ---
		if (!(_config_mask & (1 << 5)) || !(_config_mask & (1 << 6))) {
			BspLog_LogError("MotorDJI: ADRC BW or Inertia not set!");
			return;
		}
		// 根据模式初始化 ADRC 内部阶次
		if (mode == SpeedC) motor_adrc.Init(ADRC::Sec_Ord, motor_adrc.eso._base_omega_o, sqrtf(motor_adrc.kp), motor_adrc.J, motor_adrc.B, motor_adrc.Kt, _dt, motor_adrc.max_current);
		else if (mode == PosC) motor_adrc.Init(ADRC::Thr_Ord, motor_adrc.eso._base_omega_o, sqrtf(motor_adrc.kp), motor_adrc.J, motor_adrc.B, motor_adrc.Kt, _dt, motor_adrc.max_current);
	}

	_ctrl_configured = true;
	BspLog_LogSuccess("MotorDJI ID:%d Configured successfully.", motorESC_id);
}

void MotorDJI::PostDecodeCallback()
{
	if (use_adrc)
	{
		// ADRC 异步观测：更新 ESO 状态
		float real_angle = (float)measure.total_angle / 8192.0f * (2.0f * 3.14159265f);
		float real_current_amp = (float)targ_current / 16384.0f * 20.0f; // 上一次的输出电流 (A)
		motor_adrc.Observe(real_current_amp, real_angle);
	}
}

int16_t MotorDJI::Control()
{
	if (!_ctrl_configured) return 0; // 如果未显式配置控制器，不允许力矩输出
	
	if (enabled)
	{
		if (mode == NoneC)
		{
			targ_current = 0; 				// 目标电流清零
		}
		else
		{
			_CalcLoop();
		}
	}
	else
	{
		targ_current = 0; // 目标电流清零
	}
	
	return (int16_t)targ_current;
}

void MotorDJI::_CalcLoop()
{
	// 获取反馈（国际单位制）
	float real_speed = StdMath::RpmToRadS((float)measure.speed_rpm);
	float real_pos = (float)measure.total_angle / 8192.0f * (2.0f * 3.14159265f);

	float out_amp = 0;

	if (use_adrc)
	{
		// ADRC 控制模式
		if (mode == SpeedC) out_amp = motor_adrc.CalcSpeed(StdMath::RpmToRadS(targ_speed), real_pos);
		else if (mode == PosC) out_amp = motor_adrc.CalcPos(targ_position, real_pos);
	}
	else
	{
		// PID 控制模式
		if (mode == SpeedC)
		{
			out_amp = speed_pid.Calc(StdMath::RpmToRadS(targ_speed), real_speed);
		}
		else if (mode == PosC)
		{
			// 串级 PID: 位置环 -> 速度环
			float v_targ_rpm = position_pid.Calc(targ_position, real_pos);
			out_amp = speed_pid.Calc(StdMath::RpmToRadS(v_targ_rpm), real_speed);
		}
	}

	// 转换为电流指令值
	float cur_temp = AmpToICode(out_amp);

	// 限制爬坡率
	MotorDJI_SlopeLim(targ_current, cur_temp, _sloperate, _dt);

	// 电流限幅
	targ_current = StdMath::fclamp(targ_current, (float)_current_limit);
}

void MotorDJI_SlopeLim(float& cur, float& cur_temp, float slope_value, float dt)
{
	// 限制爬坡率
	float delta_current = cur_temp - cur;
	float slope_rate = slope_value * dt;

	if (delta_current > slope_rate)
	{
		cur += slope_rate;
	}
	else if (delta_current < -slope_rate)
	{
		cur -= slope_rate;
	}
	else
	{
		cur = cur_temp;
	}
}

float MotorDJI::AngCodeToRad(uint16_t ang_code)
{
	return (float)ang_code / 8192.0f * (2.0f * 3.1415926f);
}

float MotorDJI::AmpToICode(float I_Ampere)
{
	return I_Ampere * 16384.0f / 20.0f;
}