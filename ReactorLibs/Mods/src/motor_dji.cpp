/**
 * @file motor_dji.cpp
 * @brief C620 / C610 电机控制类实现文件，我们战队不用GM6020（笑）
 * @author Huangney
 * @date 2025-8-21
 */
#include "motor_dji.hpp"
#include "RtosCpp.hpp"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"
#include "stdio.h"

const char* target_prefix = "mdji";



#define CAN_OFFSET (hcan == &hcan1 ? 0 : 8)
static void MotorDji_RxCallback(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, CAN_HandleTypeDef *hcan);
static void MotorDji_SendCurrent(CAN_HandleTypeDef *hcan, int16_t motor_0, int16_t motor_1, int16_t motor_2, int16_t motor_3, bool more = false);
static void MotorDJI_CmdHandler(int argc, char **argv);

/// @brief 存储已经注册的电机实例的指针，便于回调函数查表处理
static MotorDJI* MotorPointList[1 + 16] = {nullptr}; 	 // 电机ID从1开始，所以0号不使用
// 顺序记录已经注册的电机ID，便于遍历所有电机（比如3号和6号注册了，但是位置很散，查这个表有助于提升遍历效率）
uint8_t MotorIndexList[16] = {0}, MotorIndexCount = 0; 

/**
 * @brief C620 / C610 电机额外初始化
 * @param Esc_Id 电机 ID（请查看C620 / C610说明书，注意其从 1 开始！）
 */
void MotorDJI::Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id, MotorDJIMode djimode)
{
	mode = djimode; // 设置控制模式
	
	square_injector_.InitHz(100.0f, 0.01f); 		// 方波激励器初始化，幅值20000位置单位，频率0.2Hz

	/// @brief 根据电机 ID 和 CAN路线，将其存储到全局的电机实例列表中，同时顺序记录其ID
	/// 一般一根CAN总线上最多有8个电机，所以CAN1分配到1-8号电机，CAN2分配到9-16号电机
	if (motorESC_id <= 8 && motorESC_id >= 1)
	{
		MotorPointList[motorESC_id + CAN_OFFSET] = this; 					// 根据CAN总线分配到0-7或8-15
		MotorIndexList[MotorIndexCount++] = motorESC_id + CAN_OFFSET; 		// 记录电机ID
		at_can_seg = _GetCanSeg(motorESC_id + CAN_OFFSET); 					// 记录电机所在的CAN段
	}

	_motor_id = motorESC_id; 		// 记录电机ID

	// 初始化（即注册）该电机的CAN实例
	BspCan_InstRegist(&bspcan_inst, hcan, 0x200 + motorESC_id, 0x200 + motorESC_id, 0, 0, MotorDji_RxCallback);

	if (MotorIndexCount == 1) // 仅在注册第一个电机时注册命令，避免重复注册
	{
		BspLog_RegistCMD(target_prefix, MotorDJI_CmdHandler, nullptr); // 注册 RTT 命令
	}
	
	BspLog_LogInfo("MDJI ID:%d Inited at can%d", motorESC_id, (hcan == &hcan1 ? 1 : 2));


}


/// @brief 更改电机控制模式
void MotorDJI::SwitchMode(MotorDJIMode new_mode)
{
	// 为确保安全，切换模式时，速度清零，目标位置设置为当前位置
	targ_speed = 0;
	targ_position = measure.total_angle;

	mode = new_mode;
}

/// @brief 设置速度
/// @param rpm 
void MotorDJI::SetSpeed(float rpm, float redu_ratio)
{
	if (mode != PID_SpeedControl && mode != ADRC_SpeedControl) return; 	// 不是速度模式就不执行
	targ_speed = rpm;
}

/// @brief 设置位置
/// @param pos 
void MotorDJI::SetPos(float pos)
{
	if (mode != PID_PosControl && mode != ADRC_PosControl) return; 	// 不是位置模式就不执行
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
	mode = None_Control;
}

/**
 * @brief 关闭电机控制
 * @warning 如果电机正在旋转，Disable之后指令停止发送，但是电流会保持，很容易导致电机疯转
 */
void MotorDJI::Disable()
{
	_enabled = false; 		// 禁用电机控制
}
/// @brief 开启电机控制
void MotorDJI::Enable()
{
	_enabled = true; 		// 启用电机控制
}
/// @brief 检查电机控制是否启用
bool MotorDJI::IsEnabled()
{
	return enabled; 		// 返回电机控制是否启用
}


/// @brief 判断电机所在的CAN段，用于发送
/// @param motor_id 
/// @return 所在的CAN段
uint8_t MotorDJI::_GetCanSeg(uint8_t motor_id)
{
	if (motor_id >=1 && motor_id <= 4) return 0; // CAN1的1-4号电机
	else if (motor_id >= 5 && motor_id <= 8) return 1; // CAN1的5-8号电机
	else if (motor_id >= 9 && motor_id <= 12) return 2;
	else if (motor_id >= 13 && motor_id <= 16) return 3; // CAN2的5-8号电机
	return 0; // 默认返回0
}


float MotorDJI::_GetDelayedCurrent(uint8_t delay_tick)
{
	if (delay_tick >= 5) delay_tick = 4; // 最大只能取到4个tick前的数据
	float history_u = history_current[(history_index + 5 - (delay_tick + 1)) % 5];

	return history_u;
}



/**
 * @brief 控制所有已注册电机
 * @details 该函数会遍历所有已注册的电机实例，并调用其Control方法，最后根据返回的电流值发送CAN指令
 */
void MotorDJI::ControlAllMotors()
{
	// 分四段：CAN1的1-4号电机，CAN1的5-8号电机，CAN2的1-4号电机，CAN2的5-8号电机
	bool send_can_seg[4] = {false, false, false, false};
	 // 存储所有电机的目标电流 
	int16_t motor_currents[16] = {0};

	// 遍历所有注册的电机实例，调用其Control方法，并获得本次发送的电流值
	for (uint8_t i = 0; i < MotorIndexCount; i++)
	{
		uint8_t motor_id = MotorIndexList[i];
		if (MotorPointList[motor_id] != nullptr)
		{
			MotorDJI& mt = *MotorPointList[motor_id]; 
			int16_t targ_motor_current = mt.Control();

			mt._online_cnt -= 1; 				// 在线计时器递减
			if (mt._online_cnt <= 0)	mt._online_priv = false; 		// 计时器到0，认为电机离
			else mt._online_priv = true; 								// 计时器未到0，认为电机在线

			// 激活对应的CAN段
			send_can_seg[MotorPointList[motor_id]->at_can_seg] = true;

			// 记录电流值，注意motor_id从1开始，而这里的数组是从0开始的，所以需要减1
			motor_currents[motor_id - 1] = targ_motor_current; 
		}
	}

	// 根据send_can_seg数组，判断是否需要发送CAN指令
	if (send_can_seg[0]) 
	{
		// 发送CAN1的1-4号电机的电流指令	
		MotorDji_SendCurrent(&hcan1, motor_currents[0], motor_currents[1], motor_currents[2], motor_currents[3]);
	}

	if (send_can_seg[1])
	{
		// 发送CAN1的5-8号电机的电流指令
		MotorDji_SendCurrent(&hcan1, motor_currents[4], motor_currents[5], motor_currents[6], motor_currents[7], true);
	}

	if (send_can_seg[2])
	{
		// 发送CAN2的1-4号电机的电流指令
		MotorDji_SendCurrent(&hcan2, motor_currents[8], motor_currents[9], motor_currents[10], motor_currents[11]);
	}

	if (send_can_seg[3])
	{
		// 发送CAN2的5-8号电机的电流指令
		MotorDji_SendCurrent(&hcan2, motor_currents[12], motor_currents[13], motor_currents[14], motor_currents[15], true);
	}
}


/**
 * @brief 控制本电机
 * @warning 不含发送指令！！
 */
int16_t MotorDJI::Control()
{
	if (enabled)
	{
		if (mode == None_Control)
		{
			targ_current = 0; 				// 目标电流清零
		}
		else if (mode == PID_SpeedControl)
		{
			_MotorDJI_SpeedLoop();			// 速度环控制 得到电流
		}
		else if (mode == PID_PosControl)
		{
			_MotorDJI_PosLoop();				// 位置环控制 得到速度
			_MotorDJI_SpeedLoop();			// 速度环控制 得到电流
		}
		else if (mode == ADRC_SpeedControl)
		{
			if (!motor_adrc._inited)
			{
				BspLog_LogWarning("Don't use uninited ADRC: can%d,id:%d\n", (bspcan_inst.hcan == &hcan1 ? 1 : 2), _motor_id);
				_enabled = false; 	// 禁用电机控制
				targ_current = 0; 	// 目标电流清零
			}
			else
			{
				if (motor_adrc.ad_t != ADRC::Sec_Ord)
				{
					BspLog_LogWarning("Uncompatible Order of ADRC! can%d,id:%d\n", (bspcan_inst.hcan == &hcan1 ? 1 : 2), _motor_id);
					_enabled = false; 	// 禁用电机控制
					targ_current = 0; 	// 目标电流清零
				}
				else
				{
					_MotorDJI_ADRCSpdLoop();				// 位置环控制 得到速度
					if (dynamic_identify)	SelfIdentify();
				}
			}
		}
		else if (mode == ADRC_PosControl)
		{
			if (!motor_adrc._inited)
			{
				BspLog_LogWarning("Don't use uninited ADRC: can%d,id:%d\n", (bspcan_inst.hcan == &hcan1 ? 1 : 2), _motor_id);
				_enabled = false; 	// 禁用电机控制
				targ_current = 0; 	// 目标电流清零
			}
			else
			{
				if (motor_adrc.ad_t != ADRC::Thr_Ord)
				{
					BspLog_LogWarning("Uncompatible Order of ADRC! can%d,id:%d\n", (bspcan_inst.hcan == &hcan1 ? 1 : 2), _motor_id);
					_enabled = false; 	// 禁用电机控制
					targ_current = 0; 	// 目标电流清零
				}
				else
				{
					_MotorDJI_ADRCPosLoop();				// 位置环控制 得到速度
					if (dynamic_identify)	SelfIdentify();
				}
			}
		}
		else if (mode == Identify_Mode)
		{
			// 惯量辨识模式
			// 先利用方波激励器产生激励位置
			if (motor_adrc.ad_t == ADRC::Sec_Ord)
			{
				square_injector_.amplitude = 1000.0f;
				square_injector_.period = 5.0f;
				targ_speed = square_injector_.AutoGetValue(); // 获取当前方波激励值
				// 速度环控制 得到电流
				_MotorDJI_ADRCSpdLoop();
			}
			else if (motor_adrc.ad_t == ADRC::Thr_Ord)
			{
				square_injector_.amplitude = 40000.0f;
				square_injector_.period = 2.5f;
				targ_position = square_injector_.AutoGetValue(); // 获取当前方波激励值
				// 位置环控制 得到电流
				_MotorDJI_ADRCPosLoop();
			}
				
			// 执行自整定
			SelfIdentify();
		}
	}
	else
	{
		targ_current = 0; // 目标电流清零
	}
	
	return (int16_t)targ_current;
}

/**
 * @name C620 / C610 速度环控制
 * @details 计算RPM对应的控制电流
 */
void MotorDJI::_MotorDJI_SpeedLoop()
{
	// 获取测量结构体
	moto_measure_t *ptr = &measure;

	// 计算目标的 速度PID输出（输出为电流）
	float targ_current_temp = speed_pid.Calc(targ_speed, ptr->speed_rpm, _current_limit);

	// 限制爬坡率
	float delta_current = targ_current_temp - targ_current;
	float slope_value = _sloperate * speed_pid.GetDt();

	if (delta_current > slope_value)
	{
		targ_current += slope_value;
	}
	else if (delta_current < -slope_value)
	{
		targ_current -= slope_value;
	}
	else
	{
		targ_current = targ_current_temp;
	}

	// 最终电流限幅
	Lim_ABS(targ_current, _current_limit)
}


/**
 * @name C620 / C610 速度环控制
 * @details 计算RPM对应的控制电流
 */
void MotorDJI::_MotorDJI_ADRCSpdLoop()
{
	// 获取测量结构体
	moto_measure_t *ptr = &measure;

	// 计算目标的 速度PID输出（输出为电流）
	float targ_current_temp = motor_adrc.Calc(targ_speed * (2.0f * 3.1415926f) / (60.0f), ptr->total_angle / (8192.0f) * (2.0f * 3.1415926f));
	targ_current_temp = targ_current_temp * 16384.0f / 20.0f; // 转换为电流指令值（3508将-20A~20A映射到了-16384~16384）

	// 限制爬坡率
	float delta_current = targ_current_temp - targ_current;
	float slope_value = _sloperate * 0.001f;

	if (delta_current > slope_value)
	{
		targ_current += slope_value;
	}
	else if (delta_current < -slope_value)
	{
		targ_current -= slope_value;
	}
	else
	{
		targ_current = targ_current_temp;
	}

	// 最终电流限幅
	Lim_ABS(targ_current, _current_limit)

	// 记录历史电流值，用于系统延时补偿
	history_current[history_index++] = targ_current;
	if (history_index >= 5) history_index = 0;
}

/**
 * @name C620 / C610 位置环控制
 * @details 计算位置对应的控制电流
 */
void MotorDJI::_MotorDJI_PosLoop()
{
	// 获取测量结构体
	moto_measure_t *ptr = &measure;
	// 计算目标的 位置PID输出（输出为速度）	
	targ_speed = position_pid.Calc(targ_position, ptr->total_angle, _speed_limit);
	
	// 最终速度限幅
	Lim_ABS(targ_speed, _speed_limit)
}

/**
 * @brief 自整定过程
 */
void MotorDJI::SelfIdentify()
{
	// 根据不同的辨识强度，采取不同的更新策略
	static float last_update_tick = DWT_GetTimeline_Sec();
	float current_tick = DWT_GetTimeline_Sec();

	// 如果频率到了，并且系统有足够的动态
	if (current_tick - last_update_tick >= idtf_interval && fabs(g_Identifier.rho_ru) > 0.1f)
	{
		float new_J = (1 - idtf_coeff) * motor_adrc.J + idtf_coeff * g_Identifier.J_hat_;

		motor_adrc.J = new_J; 										// 更新ADRC的b0参数
		motor_adrc.input_nltd_3rd.ResetR(fmax((motor_adrc.Kt * motor_adrc.max_current - motor_adrc.B) * 0.5f, 1e-5f) / motor_adrc.J);
		motor_adrc.eso.b0 = motor_adrc.Kt / motor_adrc.J;

		g_Identifier.b0_ = motor_adrc.eso.b0;			// 更新辨识器的b0参数
		last_update_tick = current_tick;
	}
}

/**
 * @brief 设置电机动态辨识模式
 */
void MotorDJI::Dynamicle(MotorIdentifyIntensity intensity)
{
	dynamic_identify = true;
	idtf_intensity = intensity;
	switch (idtf_intensity)
	{
		case Realtime:
		{
			idtf_interval = 0.5f;
			idtf_coeff = 0.08f;
			break;
		}
		case Dynamic:
		{
			idtf_interval = 0.5f;
			idtf_coeff = 0.033f;
			break;
		}
		case Fluid:
		{
			idtf_interval = 1.0f;
			idtf_coeff = 0.02f;
			break;
		}
		case Stable:
		{
			idtf_interval = 1.0f;
			idtf_coeff = 0.01f;
			break;
		}
		case Static:
		{
			idtf_interval = 1.0f;
			idtf_coeff = 0.005f;
			break;
		}
	}
}

/**
 * @name C620 / C610 速度环控制
 * @details 计算RPM对应的控制电流
 */
void MotorDJI::_MotorDJI_ADRCPosLoop()
{
	// 获取测量结构体
	moto_measure_t *ptr = &measure;

	// 计算目标的当前位置（国际单位制）
	int total_angle_code = ptr->total_angle;
	float total_angle_rad = total_angle_code / 8192.0f * (2.0f * 3.1415926f);

	// 输入三阶ADRC，计算目标电流
	float targ_current_temp = motor_adrc.Calc(targ_position / 8192.0f * (2.0f * 3.1415926f), total_angle_rad);

	// 转换为电流指令值（3508将-20A~20A映射到了-16384~16384）
	targ_current_temp = targ_current_temp * 16384.0f / 20.0f;

	// 限制爬坡率
	float delta_current = targ_current_temp - targ_current;
	float slope_value = _sloperate * 0.001f;

	if (delta_current > slope_value)
	{
		targ_current += slope_value;
	}
	else if (delta_current < -slope_value)
	{
		targ_current -= slope_value;
	}
	else
	{
		targ_current = targ_current_temp;
	}

	// 最终电流限幅
	Lim_ABS(targ_current, _current_limit)

	// 记录历史电流值，用于系统延时补偿
	history_current[history_index++] = targ_current;
	if (history_index >= 5) history_index = 0;
}



/**
 * @description: 获取电机反馈信息
 * @param {moto_measure_t} *ptr电机反馈信息结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
void _MotorDJI_DecodeMeasure(MotorDJI* motor_p, uint8_t *Data)
{
	// 获取测量信息的指针和其他参数
	moto_measure_t *ptr = &motor_p->measure;
	float RPM_LPF_rate = motor_p->_read_rpm_lpf_rate;
	float Current_LPF_rate = motor_p->_read_current_lpf_rate;

	// 更新上一次的角度
	ptr->last_angle = ptr->angle;

	// 更新电机转子位置
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);

	// 更新电机转速（带低通滤波）
	float speed_read = (int16_t)(Data[2] << 8 | Data[3]);
	ptr->speed_rpm = ((1.0f - RPM_LPF_rate) * ptr->speed_rpm) + (RPM_LPF_rate * speed_read);

	// 更新电机电流（带低通滤波）
	float current_read = (int16_t)(Data[4] << 8 | Data[5]) / -5;
	ptr->real_current = ((1.0f - Current_LPF_rate) * ptr->real_current) + (Current_LPF_rate * current_read);

	// 更新电机温度
	ptr->temprature = Data[6];


	// 更新圈数统计 (使用的angle而非speed_rpm)
	if (ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt--;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;

	/**********************		电机在线安全管理	**********************/
	// 重置在线计时器（倒计时100ms）
	motor_p->_online_cnt = 100;

	// 管理十次总共的时间间隔（先弹出最久的一次）
	if (motor_p->_recv_looped) motor_p->_recv_sum_interval -= motor_p->_recv_interval[motor_p->_recv_last_index];

	// 更新接收时间间隔数组和频率，并转换为0.1ms单位
	motor_p->_recv_interval[motor_p->_recv_last_index] = static_cast<uint16_t>(DWT_GetDeltaTime(&(motor_p->_recv_tick)) * 10000);

	// 将本次间隔加入总和
	motor_p->_recv_sum_interval += motor_p->_recv_interval[motor_p->_recv_last_index++];
	
	// 更新下标，指向下一个位置，同时防止数组越界
	if (motor_p->_recv_last_index >= 10)
	{
		motor_p->_recv_last_index = 0;
		motor_p->_recv_looped = true;
	}

	// 计算平均接收时间间隔和频率
	motor_p->_recv_freq = motor_p->_recv_sum_interval > 0 ?(10000.0f / (motor_p->_recv_sum_interval / 10.0f)) : 0.0f;

	/**********************		电机状态观测	**********************/
	// 输入为电流，单位A
	float current_ampero = motor_p->_GetDelayedCurrent(0) / 16384.0f * 20.0f;
	// 观测变量为total_angle，但其单位为SI的rad
	float angle_rad = motor_p->measure.total_angle / 8192.0f * 2.0f * 3.1415926f;

	// 输入数据到观测器
	motor_p->motor_adrc.Observe(current_ampero, angle_rad);
	
	// 如果系统需要动态自整定
	if (motor_p->dynamic_identify || motor_p->mode == Identify_Mode)
	{
		float r_cmd = 0.0f;
		// 获取系统当前状态
		if (motor_p->motor_adrc.ad_t == ADRC::Thr_Ord)	r_cmd = motor_p->motor_adrc.input_nltd_3rd.v3; 		// 使用加速度规划作为参考输入
		else if (motor_p->motor_adrc.ad_t == ADRC::Sec_Ord)	r_cmd = motor_p->motor_adrc.input_td.v2; 	// 使用加速度规划作为参考

		float u_out = current_ampero ; 								// ADRC输出的 u (电流/转矩)
		float z3    = motor_p->motor_adrc.eso.z3;        			// ESO观测到的扰动

		// 喂数据给算法
		motor_p->g_Identifier.Update(r_cmd, u_out, z3);
	}
}


/**
 * @description: 电机上电角度=0， 之后用这个函数更新3508电机的相对开机后（为0）的相对角度。
 * @param {moto_measure_t} *ptr电机结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
static void _MotorDJI_DecodeInitOffset(MotorDJI* motor_p, uint8_t *Data)
{
	moto_measure_t *ptr = &motor_p->measure;
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);
	ptr->offset_angle = ptr->angle;
}

/**
 * @description: 发送电机电流控制
 * @return {*}无
 */
static void MotorDji_SendCurrent(CAN_HandleTypeDef *hcan, int16_t motor_0, int16_t motor_1, int16_t motor_2, int16_t motor_3, bool more)
{
	uint8_t motor_current_data[8] = {0};
	motor_current_data[0] = motor_0 >> 8;
	motor_current_data[1] = motor_0;
	motor_current_data[2] = motor_1 >> 8;
	motor_current_data[3] = motor_1;
	motor_current_data[4] = motor_2 >> 8;
	motor_current_data[5] = motor_2;
	motor_current_data[6] = motor_3 >> 8;
	motor_current_data[7] = motor_3;

	// 根据是否需要发送到ID大于4的电机，选择不同的ID
	if (more)
	{
		BspCan_TxConfig txconf = BspCan_GetTxConfig(hcan, 0x1ff, BSPCAN_STD, BSPCAN_DATA, 8, 50);
		BspCan_Transmit(txconf, motor_current_data);
	}
	else
	{
		BspCan_TxConfig txconf = BspCan_GetTxConfig(hcan, 0x200, BSPCAN_STD, BSPCAN_DATA, 8, 50);
		BspCan_Transmit(txconf, motor_current_data);
	}
}


/// @brief 接收回调函数
/// @param RxHeader 
/// @param RxData 
static void MotorDji_RxCallback(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, CAN_HandleTypeDef *hcan)
{
	// 注意 motor_id 和 motorESC_id 的区别
	uint8_t motor_id = ((RxHeader->StdId) & 0xff) - 0x200 + CAN_OFFSET; // 获取电机ID（带CAN偏置，以区分 CAN1和CAN2的电机ID）

	// 利用全局的电机实例列表 来获取对应的电机实例
	if (motor_id > 16 || MotorPointList[motor_id] == nullptr) return; 			// 如果电机ID不合法或未注册，直接返回
	MotorDJI &motor = *MotorPointList[motor_id]; 								// 获取对应的电机实例

	// 更新电机反馈信息
	motor.measure.msg_cnt++ <= 50 ? _MotorDJI_DecodeInitOffset(&motor, RxData) : _MotorDJI_DecodeMeasure(&motor, RxData);
}



/***----------------			MotorDJI RTT命令行			------------*/
/**
 * @brief DJI 电机调试命令回调函数
 * @note 命令格式: mdji -id=1 -e (使能) | mdji -id=1 -d (失能)
 */
static void MotorDJI_CmdHandler(int argc, char **argv)
{
    static int motor_id = 1;       // 默认一号电机
    int action = 0;    // 0:无动作, 1:使能, 2:失能
	size_t prefix_len = strlen(target_prefix);

    // argv[0] 是 "mdji"，前缀一定是匹配的，所以从 argv[1] 开始遍历参数
    for (int i = 1; i < argc; i++)
    {
        // 尝试解析ID
        if (strncmp(argv[i], "-id=", 4) == 0)
        {
            // sscanf 返回 1 表示成功解析到一个整数
			if (sscanf(argv[i] + prefix_len, "%d", &motor_id) != 1)
			{
				BspLog_LogError("Invalid motor ID format: %s\n", argv[i]);
				return;
			}
        }

        // 尝试解析 动作 参数 (-e 或 -d)
        else if (strcmp(argv[i], "-e") == 0)
        {
            action = 1; // 标记为使能
        }
        else if (strcmp(argv[i], "-d") == 0)
        {
            action = 2; // 标记为失能
        }
    }
	
	// 利用全局的电机实例列表 来获取对应的电机实例
	if (motor_id > 16 || MotorPointList[motor_id] == nullptr)
	{
		BspLog_LogError("MDJI ID not found: %d\n", motor_id);
		return;
	}

	// 获取对应的电机实例
	MotorDJI &motor = *MotorPointList[motor_id];

    // 根据动作执行
    switch (action)
    {
    case 1:
		motor.Enable();
		BspLog_LogRespond("MotorDJI%d enabled\n", motor_id);
        break;
    case 2:
        motor.Disable();
		BspLog_LogRespond("MotorDJI%d disabled\n", motor_id);
        break;
    default:
        BspLog_LogWarning("No action has down\n");
        break;
    }
}