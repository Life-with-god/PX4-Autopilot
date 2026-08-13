/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file FailureDetector.hpp
* Base class for failure detection logic based on vehicle states
* for failsafe triggering.
*
* @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
*
*/

#pragma once

#include "FailureInjector.hpp"

#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/trajectory_setpoint.h>

union failure_detector_status_u {
	struct {
		uint16_t roll : 1;
		uint16_t pitch : 1;
		uint16_t alt : 1;
		uint16_t ext : 1;
		uint16_t arm_escs : 1;
		uint16_t battery : 1;
		uint16_t imbalanced_prop : 1;
		uint16_t motor : 1;
	} flags;
	uint16_t value {0};
};

using uORB::SubscriptionData;

class FailureDetector : public ModuleParams
{
public:
	FailureDetector(ModuleParams *parent);
	~FailureDetector() = default;

	bool update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode);
	const failure_detector_status_u &getStatus() const { return _status; }
	const decltype(failure_detector_status_u::flags) &getStatusFlags() const { return _status.flags; }
	float getImbalancedPropMetric() const { return _imbalanced_prop_lpf.getState(); }
	uint16_t getMotorFailures() const { return _motor_failure_esc_timed_out_mask | _motor_failure_esc_under_current_mask; }
	uint16_t getMotorStopMask() { return _failure_injector.getMotorStopMask(); }

	// ============================================================
	// LOCP (Loss-of-Control Protection) —— 失控保护系统
	// ============================================================
	// 设计目标：在无遥控器、地面站指令无效的情况下，自动检测飞行器失控状态
	// 检测维度：姿态变化率(ARD)、速度变化率(VRD)、位置高度变化率(PRD)、
	//          电流异常(COD)、MAVLink消息超时(MTO)、
	//          碰撞/撞击检测、Offboard Setpoint异常(OBS)
	// 动作分配（固定，无等级参数）：
	//   ARD/VRD/PRD/COD/Crash → 停桨 Disarm（飞机自身失控）
	//   MTO/OBS → 降落 Land（飞机自身正常，仅外部输入异常）

	/** ARD: 姿态变化率异常检测是否触发 (角速度/角加速度超限) */
	bool getLOCP_ARD() const { return _locp_ard_triggered; }
	/** VRD: 速度变化率异常检测是否触发 (水平加速度/自由落体/jerk超限) */
	bool getLOCP_VRD() const { return _locp_vrd_triggered; }
	/** PRD: 位置变化率异常检测是否触发 (急降/高度振荡/水平漂移) */
	bool getLOCP_PRD() const { return _locp_prd_triggered; }
	/** COD: 电流异常检测是否触发 (总电流突增/dI/dt尖峰) */
	bool getLOCP_COD() const { return _locp_cod_triggered; }
	/** MTO: MAVLink消息超时检测是否触发 (心跳丢失/指令超时/消息速率骤降) */
	bool getLOCP_MTO() const { return _locp_mto_triggered; }
	/** 飞机自身状态健康：MTO/OBS 降落动作的安全门槛 (不受各维度EN开关控制) */
	bool getLOCP_VehicleHealthy() const { return _locp_vehicle_healthy; }
	/** 碰撞/撞击瞬时检测是否触发 (基于IMU加速度尖峰) */
	bool getCrashDetected() const { return _crash_detected; }
	/** OBS: Offboard Setpoint 异常检测是否触发 (数值跳变/NaN注入) */
	bool getLOCP_OBS() const { return _locp_obs_triggered; }
	/** TRD: 动力响应校验是否触发 (高油门但垂直加速度不足，卡网/动力丢失/桨损坏) */
	bool getLOCP_TRD() const { return _locp_trd_triggered; }
	/** TRD 分级动作：低高度请求降落 (≤LOCP_TRD_LAND_H)，否则直接停桨 */
	bool getLOCP_TRD_RequestLand() const { return _locp_trd_land; }
	/** TRD 接管不可用：RC失联 且 QGC无指令（无人可接管，必须立即动作） */
	bool getLOCP_TRD_NoTakeover() const { return _locp_trd_no_takeover; }

private:
	void updateAttitudeStatus(const vehicle_status_s &vehicle_status);
	void updateExternalAtsStatus();
	void updateEscsStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status);
	void updateMotorStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status);
	void updateImbalancedPropStatus();

	// ============================================================
	// LOCP 检测函数
	// ============================================================

	/** LOCP 主更新入口：依次调用各维度检测，最后综合评估严重等级 */
	void updateLOCP(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &mode);
	/** ARD 姿态变化率异常检测：角加速度尖峰 + 持续高角速率 */
	bool checkAttitudeRateAnomaly(const vehicle_angular_velocity_s &ang_vel);
	/** VRD 速度变化率异常检测：水平加速度/自由落体/jerk */
	bool checkVelocityRateAnomaly(const vehicle_local_position_s &loc);
	/** PRD 位置变化率异常检测：急降/高度振荡/水平漂移 */
	bool checkPositionRateAnomaly(const vehicle_local_position_s &loc, const home_position_s &home);
	/** COD 电流异常检测：总电流突增/dI/dt尖峰 */
	bool checkCurrentAnomaly(const battery_status_s &bat);
	/** TRD 动力响应校验：高油门但垂直加速度不足（卡网/动力丢失/桨损坏） */
	bool checkThrustResponse(const vehicle_status_s &vehicle_status);
	/** 飞机自身状态健康检查：MTO/OBS 降落动作的安全门槛 (实时状态确认，不受EN控制) */
	bool checkVehicleHealthy();
	/** MTO MAVLink超时检测：心跳超时/指令超时/消息速率骤降 */
	bool checkMavlinkTimeout();
	/** 碰撞/撞击瞬时检测：基于IMU加速度范数阈值判断 */
	bool checkCrashImpact();
	/** OBS Offboard Setpoint 异常检测：数值跳变/NaN注入 */
	bool checkOffboardSetpointSanity();

	failure_detector_status_u _status{};

	systemlib::Hysteresis _roll_failure_hysteresis{false};
	systemlib::Hysteresis _pitch_failure_hysteresis{false};
	systemlib::Hysteresis _ext_ats_failure_hysteresis{false};
	systemlib::Hysteresis _esc_failure_hysteresis{false};

	static constexpr float _imbalanced_prop_lpf_time_constant{5.f};
	AlphaFilter<float> _imbalanced_prop_lpf{};
	uint32_t _selected_accel_device_id{0};
	hrt_abstime _imu_status_timestamp_prev{0};

	// Motor failure check
	uint8_t _motor_failure_esc_valid_current_mask{};  // ESC 1-8, true if ESC telemetry was valid at some point
	uint8_t _motor_failure_esc_timed_out_mask{};      // ESC telemetry no longer available -> failure
	uint8_t _motor_failure_esc_under_current_mask{};  // ESC drawing too little current -> failure
	bool _motor_failure_esc_has_current[actuator_motors_s::NUM_CONTROLS] {false}; // true if some ESC had non-zero current (some don't support it)
	hrt_abstime _motor_failure_undercurrent_start_time[actuator_motors_s::NUM_CONTROLS] {};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)}; // TODO: multi-instance
	uORB::Subscription _pwm_input_sub{ORB_ID(pwm_input)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

	// ============================================================
	// LOCP 订阅的 uORB 话题
	// ============================================================
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};  // 本地位置/速度（VRD、PRD用）
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};                  // 电池电流（COD用）
	uORB::Subscription _telemetry_status_sub{ORB_ID(telemetry_status)};              // 遥测心跳状态（MTO用）
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};                // 地面站指令（MTO用）
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; // RC 摇杆输入（TRD 接管检查用，持续更新=RC在线）
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};                    // Home点位置（PRD用）
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};        // Offboard setpoint（OBS用）

	// ============================================================
	// LOCP 内部状态变量
	// ============================================================

	// --- ARD (Attitude Rate Detection) 姿态变化率检测 ---
	float _rollspeed_prev{0.f};              // 上一帧 Roll 角速度 (rad/s)，用于计算角加速度
	float _pitchspeed_prev{0.f};             // 上一帧 Pitch 角速度 (rad/s)
	float _yawspeed_prev{0.f};               // 上一帧 Yaw 角速度 (rad/s)
	hrt_abstime _att_rate_high_start{0};     // 持续高角速率开始时刻（用于判断是否超 LOCP_ARD_DUR）
	hrt_abstime _att_rate_timestamp_prev{0}; // 上一帧角速度时间戳 (us)，用于计算真实角加速度 dt
	systemlib::Hysteresis _ard_hysteresis{false}; // ARD 迟滞滤波器，防止瞬时抖动误触发
	bool _locp_ard_triggered{false};         // ARD 检测最终触发标志

	// --- VRD (Velocity Rate Detection) 速度变化率检测 ---
	float _acc_horiz_prev{0.f};              // 上一帧水平加速度幅值 (m/s²)，用于计算 jerk
	hrt_abstime _acc_horiz_timestamp_prev{0};// 上一帧位置时间戳 (us)，用于计算真实 jerk dt
	hrt_abstime _horiz_spd_high_start{0};    // 水平速度超阈值开始时刻（温和乱飞持续检测）
	systemlib::Hysteresis _vrd_hysteresis{false}; // VRD 迟滞滤波器
	bool _locp_vrd_triggered{false};         // VRD 检测最终触发标志

	// --- PRD (Position Rate Detection) 位置变化率检测 ---
	float _alt_history[10]{};                // 高度滑动窗口（10帧历史），用于计算高度振荡标准差
	uint8_t _alt_history_idx{0};             // 高度历史写入索引（环形缓冲）
	uint8_t _alt_history_count{0};           // 已累积的高度历史有效帧数
	systemlib::Hysteresis _prd_hysteresis{false}; // PRD 迟滞滤波器
	bool _locp_prd_triggered{false};         // PRD 检测最终触发标志

	// --- COD (Current Overdraw Detection) 电流异常检测 ---
	float _current_sliding_window[20]{};     // 电流滑动窗口（20帧），用于计算移动平均
	uint8_t _current_window_idx{0};          // 电流窗口写入索引（环形缓冲）
	uint8_t _current_window_count{0};        // 已累积的电流窗口有效帧数
	float _current_prev{0.f};                // 上一帧总电流 (A)，用于计算 dI/dt
	hrt_abstime _current_timestamp_prev{0};  // 上一帧电流时间戳 (us)，用于计算真实 dI/dt
	systemlib::Hysteresis _cod_hysteresis{false}; // COD 迟滞滤波器
	bool _locp_cod_triggered{false};         // COD 检测最终触发标志

	// --- MTO (MAVLink Timeout) MAVLink 消息超时检测 ---
	hrt_abstime _last_mavlink_heartbeat{0};  // 最后一次收到 MAVLink 心跳的时间戳
	hrt_abstime _last_vehicle_command{0};    // 最后一次收到车辆指令的时间戳
	uint32_t _mavlink_msg_counter{0};        // 当前速率窗口内接收的 MAVLink 消息计数
	hrt_abstime _rate_window_start{0};       // 当前速率窗口起始时间
	bool _locp_mto_triggered{false};         // MTO 检测最终触发标志

	// --- LOCP 公共状态 ---
	hrt_abstime _locp_arm_time{0};           // 解锁时刻 (us)，用于启动保护（避免解锁瞬间电流/姿态误判）
	bool _locp_vehicle_healthy{true};        // 飞机自身状态健康（MTO/OBS 降落安全门槛）

	// --- Crash/Impact 碰撞/撞击检测 ---
	hrt_abstime _crash_impact_time{0};       // 碰撞检测触发时刻（用于2秒锁存后自动清除）
	bool _crash_detected{false};             // 碰撞/撞击检测标志（瞬时触发，无迟滞）

	// --- OBS (Offboard Setpoint Sanity) 机载计算机Setpoint异常检测 ---
	float _sp_pos_prev[3]{0.f};              // 上一帧位置 setpoint (m)，用于跳变检测
	float _sp_vel_prev[3]{0.f};              // 上一帧速度 setpoint (m/s)
	float _sp_yaw_prev{0.f};                 // 上一帧 yaw setpoint (rad)
	bool _sp_was_valid{false};               // 上一帧是否有有效 setpoint
	systemlib::Hysteresis _obs_hysteresis{false}; // OBS 迟滞滤波器
	bool _locp_obs_triggered{false};         // OBS 检测最终触发标志

	// --- TRD (Thrust Response Detection) 动力响应校验 ---
	hrt_abstime _trd_high_thr_start{0};      // 高油门无响应开始时刻（去抖计时）
	hrt_abstime _trd_normal_start{0};        // 连续正常开始时刻（锁存复位计时）
	hrt_abstime _trd_land_try_start{0};      // 降落尝试开始时刻（超时转停桨计时）
	hrt_abstime _trd_takeover_watch_start{0};// 接管观察窗口开始时刻（接管无效检测）
	bool _locp_trd_triggered{false};         // 动力无响应确认触发标志（锁存）
	bool _locp_trd_land{false};              // 分级动作：低高度请求降落
	bool _locp_trd_no_takeover{false};       // 接管不可用/无效（RC失联且QGC无指令，或接管观察超时）
	hrt_abstime _last_gcs_cmd{0};            // 最后一次收到地面站指令的时间戳
	hrt_abstime _last_rc_input{0};           // 最后一次收到 RC 摇杆输入（manual_control_setpoint）的时间戳

	FailureInjector _failure_injector;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FD_FAIL_P>) _param_fd_fail_p,
		(ParamInt<px4::params::FD_FAIL_R>) _param_fd_fail_r,
		(ParamFloat<px4::params::FD_FAIL_R_TTRI>) _param_fd_fail_r_ttri,
		(ParamFloat<px4::params::FD_FAIL_P_TTRI>) _param_fd_fail_p_ttri,
		(ParamBool<px4::params::FD_EXT_ATS_EN>) _param_fd_ext_ats_en,
		(ParamInt<px4::params::FD_EXT_ATS_TRIG>) _param_fd_ext_ats_trig,
		(ParamInt<px4::params::FD_ESCS_EN>) _param_escs_en,
		(ParamInt<px4::params::FD_IMB_PROP_THR>) _param_fd_imb_prop_thr,

		// Actuator failure
		(ParamBool<px4::params::FD_ACT_EN>) _param_fd_actuator_en,
		(ParamFloat<px4::params::FD_ACT_MOT_THR>) _param_fd_motor_throttle_thres,
		(ParamFloat<px4::params::FD_ACT_MOT_C2T>) _param_fd_motor_current2throttle_thres,
		(ParamInt<px4::params::FD_ACT_MOT_TOUT>) _param_fd_motor_time_thres,

		// ============================================================
		// LOCP (Loss-of-Control Protection) 参数
		// ============================================================

		// --- LOCP 总开关 ---
		(ParamBool<px4::params::LOCP_EN>) _param_locp_en,                  // LOCP 失控保护总开关

		// --- ARD (Attitude Rate Detection) 姿态变化率检测 ---
		(ParamBool<px4::params::LOCP_ARD_EN>) _param_locp_ard_en,           // ARD 检测使能开关
		(ParamBool<px4::params::LOCP_ARD_ACC_EN>) _param_locp_ard_acc_en,   // ARD 角加速度检测使能开关
		(ParamBool<px4::params::LOCP_ARD_RATE_EN>) _param_locp_ard_rate_en, // ARD 持续高角速率检测使能开关
		(ParamFloat<px4::params::LOCP_ARD_R_MAX>) _param_locp_ard_r_max,    // Roll 角加速度最大阈值 (rad/s²)
		(ParamFloat<px4::params::LOCP_ARD_P_MAX>) _param_locp_ard_p_max,    // Pitch 角加速度最大阈值 (rad/s²)
		(ParamFloat<px4::params::LOCP_ARD_Y_MAX>) _param_locp_ard_y_max,    // Yaw 角加速度最大阈值 (rad/s²)
		(ParamFloat<px4::params::LOCP_ARD_T>) _param_locp_ard_t,            // ARD 迟滞确认时间 (秒)，故障需持续此时长才确认
		(ParamFloat<px4::params::LOCP_ARD_RSP>) _param_locp_ard_rsp,        // Roll 持续高角速率设定点 (rad/s)
		(ParamFloat<px4::params::LOCP_ARD_PSP>) _param_locp_ard_psp,        // Pitch 持续高角速率设定点 (rad/s)
		(ParamFloat<px4::params::LOCP_ARD_YSP>) _param_locp_ard_ysp,        // Yaw 持续高角速率设定点 (rad/s)
		(ParamFloat<px4::params::LOCP_ARD_DUR>) _param_locp_ard_dur,        // 持续高角速率最小持续时间 (秒)

		// --- VRD (Velocity Rate Detection) 速度变化率检测 ---
		(ParamBool<px4::params::LOCP_VRD_EN>) _param_locp_vrd_en,           // VRD 检测使能开关
		(ParamBool<px4::params::LOCP_VRD_AH_EN>) _param_locp_vrd_ah_en,     // VRD 水平加速度检测使能开关
		(ParamBool<px4::params::LOCP_VRD_FF_EN>) _param_locp_vrd_ff_en,     // VRD 自由落体检测使能开关
		(ParamBool<px4::params::LOCP_VRD_JK_EN>) _param_locp_vrd_jk_en,     // VRD Jerk 检测使能开关
		(ParamBool<px4::params::LOCP_VRD_HS_EN>) _param_locp_vrd_hs_en,     // VRD 水平速度持续检测使能开关
		(ParamFloat<px4::params::LOCP_VRD_AH_MAX>) _param_locp_vrd_ah_max,  // 水平加速度最大阈值 (m/s²)
		(ParamFloat<px4::params::LOCP_VRD_AD_MAX>) _param_locp_vrd_ad_max,  // 垂直向下加速度阈值 (m/s²)，自由落体判断
		(ParamFloat<px4::params::LOCP_VRD_VZD_MAX>) _param_locp_vrd_vzd_max,// 垂直下降速度阈值 (m/s)，配合自由落体判断
		(ParamFloat<px4::params::LOCP_VRD_JERK>) _param_locp_vrd_jerk,      // 水平 Jerk（加加速度）阈值 (m/s³)
		(ParamFloat<px4::params::LOCP_VRD_T>) _param_locp_vrd_t,            // VRD 迟滞确认时间 (秒)
		(ParamFloat<px4::params::LOCP_VRD_HS_MAX>) _param_locp_vrd_hs_max,  // 水平速度持续阈值 (m/s)，温和乱飞
		(ParamFloat<px4::params::LOCP_VRD_HS_DUR>) _param_locp_vrd_hs_dur,  // 水平速度持续时长 (秒)，温和乱飞

		// --- PRD (Position Rate Detection) 位置变化率检测 ---
		(ParamBool<px4::params::LOCP_PRD_EN>) _param_locp_prd_en,           // PRD 检测使能开关
		(ParamBool<px4::params::LOCP_PRD_DES_EN>) _param_locp_prd_des_en,   // PRD 急降检测使能开关
		(ParamBool<px4::params::LOCP_PRD_OSC_EN>) _param_locp_prd_osc_en,   // PRD 高度振荡检测使能开关
		(ParamBool<px4::params::LOCP_PRD_DRF_EN>) _param_locp_prd_drf_en,   // PRD 水平漂移检测使能开关
		(ParamFloat<px4::params::LOCP_PRD_VZ_MAX>) _param_locp_prd_vz_max,  // 垂直下降速度阈值 (m/s)，急降判断
		(ParamFloat<px4::params::LOCP_PRD_ADROP>) _param_locp_prd_adrop,    // 高度下降量阈值 (m)，相对于 Home 点
		(ParamFloat<px4::params::LOCP_PRD_ASTD>) _param_locp_prd_astd,      // 高度振荡标准差阈值 (m)，滑动窗口法
		(ParamFloat<px4::params::LOCP_PRD_HSPD>) _param_locp_prd_hspd,      // 水平漂移速度阈值 (m/s)
		(ParamFloat<px4::params::LOCP_PRD_T>) _param_locp_prd_t,            // PRD 迟滞确认时间 (秒)

		// --- COD (Current Overdraw Detection) 电流异常检测 ---
		(ParamBool<px4::params::LOCP_COD_EN>) _param_locp_cod_en,           // COD 检测使能开关
		(ParamBool<px4::params::LOCP_COD_SRG_EN>) _param_locp_cod_srg_en,   // COD 电流突增检测使能开关
		(ParamBool<px4::params::LOCP_COD_SPK_EN>) _param_locp_cod_spk_en,   // COD dI/dt 尖峰检测使能开关
		(ParamFloat<px4::params::LOCP_COD_DELTA_I>) _param_locp_cod_delta_i,// 总电流偏离滑动均值阈值 (A)，突增判断
		(ParamFloat<px4::params::LOCP_COD_MAX_I>) _param_locp_cod_max_i,    // 总电流绝对最大阈值 (A)
		(ParamFloat<px4::params::LOCP_COD_DI_DT>) _param_locp_cod_di_dt,    // 电流变化率 dI/dt 阈值 (A/s)，尖峰判断
		(ParamFloat<px4::params::LOCP_COD_T>) _param_locp_cod_t,            // COD 迟滞确认时间 (秒)
		(ParamFloat<px4::params::LOCP_COD_ARM_DLY>) _param_locp_cod_arm_dly, // 解锁后 COD 启动保护延迟 (秒)，期间不检测避免启动电流误判

		// --- MTO (MAVLink Timeout) MAVLink 消息超时检测 ---
		(ParamBool<px4::params::LOCP_MTO_EN>) _param_locp_mto_en,           // MTO 检测使能开关
		(ParamBool<px4::params::LOCP_MTO_HB_EN>) _param_locp_mto_hb_en,     // MTO 心跳超时检测使能开关
		(ParamBool<px4::params::LOCP_MTO_CMD_EN>) _param_locp_mto_cmd_en,   // MTO 指令超时检测使能开关
		(ParamFloat<px4::params::LOCP_MTO_HB_T>) _param_locp_mto_hb_t,      // 心跳超时阈值 (秒)
		(ParamFloat<px4::params::LOCP_MTO_CMD_T>) _param_locp_mto_cmd_t,    // 指令超时阈值 (秒)
		(ParamFloat<px4::params::LOCP_MTO_RATE>) _param_locp_mto_rate,      // 消息速率最低阈值 (Hz)，1秒滑动窗口统计

		// --- TRD (Thrust Response Detection) 动力响应校验 ---
		(ParamBool<px4::params::LOCP_TRD_EN>) _param_locp_trd_en,           // TRD 检测使能开关
		(ParamFloat<px4::params::LOCP_TRD_THR_H>) _param_locp_trd_thr_high, // 高油门判定阈值 (0~1)
		(ParamFloat<px4::params::LOCP_TRD_T>) _param_locp_trd_t,               // 高油门无响应确认时间 (秒)
		(ParamFloat<px4::params::LOCP_TRD_AZ_MIN>) _param_locp_trd_az_min,     // 垂直加速度下限 (m/s²)，无向上加速判定
		(ParamFloat<px4::params::LOCP_TRD_LAND_H>) _param_locp_trd_land_h,     // 分级动作：低高度降落阈值 (m)
		(ParamFloat<px4::params::LOCP_TRD_LTOUT>) _param_locp_trd_land_tout, // 降落尝试超时 (秒)，超时转停桨
		(ParamFloat<px4::params::LOCP_TRD_WATCH>) _param_locp_trd_to_watch, // 接管观察窗口 (秒)，接管无效检测
		(ParamFloat<px4::params::LOCP_TRD_VZ_MAX>) _param_locp_trd_vz_max,  // 垂直速度上限 (m/s)，无显著升降判定
		(ParamFloat<px4::params::LOCP_TRD_HS_MAX>) _param_locp_trd_hs_max,  // 水平速度上限 (m/s)，无水平移动判定
		(ParamFloat<px4::params::LOCP_TRD_MIN_H>) _param_locp_trd_min_h,    // 最低检测高度 (m)，排除地面测试

		// --- 飞机自身健康检查门槛（checkVehicleHealthy，独立于各检测维度参数） ---
		(ParamFloat<px4::params::LOCP_HC_RATE_MAX>) _param_locp_hc_rate_max, // Roll/Pitch 角速率健康门槛 (rad/s)
		(ParamFloat<px4::params::LOCP_HC_YAW_MAX>) _param_locp_hc_yaw_max,   // Yaw 角速率健康门槛 (rad/s)
		(ParamFloat<px4::params::LOCP_HC_HS_MAX>) _param_locp_hc_hs_max,     // 水平速度健康门槛 (m/s)
		(ParamFloat<px4::params::LOCP_HC_VZ_MAX>) _param_locp_hc_vz_max,     // 垂直下降速度健康门槛 (m/s)

		// --- Crash/Impact 碰撞检测 ---
		(ParamBool<px4::params::LOCP_CRASH_EN>) _param_locp_crash_en,       // 碰撞检测使能开关
		(ParamFloat<px4::params::LOCP_CRASH_THR>) _param_locp_crash_thr,    // 碰撞加速度阈值 (m/s²)，默认 ~80m/s² (~8g)

		// --- OBS (Offboard Setpoint Sanity) 机载计算机Setpoint异常检测 ---
		(ParamBool<px4::params::LOCP_OBS_EN>) _param_locp_obs_en,           // OBS 检测使能开关
		(ParamBool<px4::params::LOCP_OBS_JMP_EN>) _param_locp_obs_jmp_en,   // OBS 数值跳变检测使能开关
		(ParamBool<px4::params::LOCP_OBS_NAN_EN>) _param_locp_obs_nan_en,   // OBS NaN 注入检测使能开关
		(ParamFloat<px4::params::LOCP_OBS_J_POS>) _param_locp_obs_jump_pos, // 位置跳变阈值 (m)
		(ParamFloat<px4::params::LOCP_OBS_J_VEL>) _param_locp_obs_jump_vel, // 速度跳变阈值 (m/s)
		(ParamFloat<px4::params::LOCP_OBS_J_YAW>) _param_locp_obs_jump_yaw, // Yaw 跳变阈值 (rad)
		(ParamFloat<px4::params::LOCP_OBS_T>) _param_locp_obs_t             // OBS 迟滞确认时间 (秒)
	)
};
