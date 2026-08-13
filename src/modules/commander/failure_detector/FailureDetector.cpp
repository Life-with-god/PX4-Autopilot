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
* @file FailureDetector.cpp
*
* @author Mathieu Bresciani	<brescianimathieu@gmail.com>
*
*/

#include "FailureDetector.hpp"

using namespace time_literals;

FailureDetector::FailureDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FailureDetector::update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode)
{
	_failure_injector.update();

	failure_detector_status_u status_prev = _status;

	if (vehicle_control_mode.flag_control_attitude_enabled) {
		updateAttitudeStatus(vehicle_status);

		if (_param_fd_ext_ats_en.get()) {
			updateExternalAtsStatus();
		}

	} else {
		_status.flags.roll = false;
		_status.flags.pitch = false;
		_status.flags.alt = false;
		_status.flags.ext = false;
	}

	// esc_status subscriber is shared between subroutines
	esc_status_s esc_status;

	if (_esc_status_sub.update(&esc_status)) {
		_failure_injector.manipulateEscStatus(esc_status);

		if (_param_escs_en.get()) {
			updateEscsStatus(vehicle_status, esc_status);
		}

		if (_param_fd_actuator_en.get()) {
			updateMotorStatus(vehicle_status, esc_status);
		}
	}

	if (_param_fd_imb_prop_thr.get() > 0) {
		updateImbalancedPropStatus();
	}

	// ============================================================
	// LOCP (Loss-of-Control Protection) —— 失控保护系统
	// 在完成原有故障检测后，运行 LOCP 多维度检测。
	// 检测维度：姿态变化率(ARD)、速度变化率(VRD)、位置变化率(PRD)、
	//          电流异常(COD)、MAVLink超时(MTO)、碰撞检测
	// 严重等级：0=NONE, 1=LEVEL_1(降落), 2=LEVEL_2(急降), 3=LEVEL_3(终止)
	// ============================================================
	updateLOCP(vehicle_status, vehicle_control_mode);

	return _status.value != status_prev.value;
}

void FailureDetector::updateAttitudeStatus(const vehicle_status_s &vehicle_status)
{
	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.update(&attitude)) {

		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		float roll(euler.phi());
		float pitch(euler.theta());

		// special handling for tailsitter
		if (vehicle_status.is_vtol_tailsitter) {
			if (vehicle_status.in_transition_mode) {
				// disable attitude check during tailsitter transition
				roll = 0.f;
				pitch = 0.f;

			} else if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				// in FW flight rotate the attitude by 90° around pitch (level FW flight = 0° pitch)
				const matrix::Eulerf euler_rotated = matrix::Eulerf(matrix::Quatf(attitude.q) * matrix::Quatf(matrix::Eulerf(0.f,
								     M_PI_2_F, 0.f)));
				roll = euler_rotated.phi();
				pitch = euler_rotated.theta();
			}
		}

		const float max_roll_deg = _param_fd_fail_r.get();
		const float max_pitch_deg = _param_fd_fail_p.get();
		const float max_roll(fabsf(math::radians(max_roll_deg)));
		const float max_pitch(fabsf(math::radians(max_pitch_deg)));

		const bool roll_status = (max_roll > FLT_EPSILON) && (fabsf(roll) > max_roll);
		const bool pitch_status = (max_pitch > FLT_EPSILON) && (fabsf(pitch) > max_pitch);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_roll_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_r_ttri.get()));
		_pitch_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_p_ttri.get()));
		_roll_failure_hysteresis.set_state_and_update(roll_status, time_now);
		_pitch_failure_hysteresis.set_state_and_update(pitch_status, time_now);

		// Update status
		_status.flags.roll = _roll_failure_hysteresis.get_state();
		_status.flags.pitch = _pitch_failure_hysteresis.get_state();
	}
}

void FailureDetector::updateExternalAtsStatus()
{
	pwm_input_s pwm_input;

	if (_pwm_input_sub.update(&pwm_input)) {

		uint32_t pulse_width = pwm_input.pulse_width;
		bool ats_trigger_status = (pulse_width >= (uint32_t)_param_fd_ext_ats_trig.get()) && (pulse_width < 3_ms);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_ext_ats_failure_hysteresis.set_hysteresis_time_from(false, 100_ms); // 5 consecutive pulses at 50hz
		_ext_ats_failure_hysteresis.set_state_and_update(ats_trigger_status, time_now);

		_status.flags.ext = _ext_ats_failure_hysteresis.get_state();
	}
}

void FailureDetector::updateEscsStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status)
{
	hrt_abstime time_now = hrt_absolute_time();

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);
		const int all_escs_armed_mask = (1 << limited_esc_count) - 1;
		const bool is_all_escs_armed = (all_escs_armed_mask == esc_status.esc_armed_flags);

		bool is_esc_failure = !is_all_escs_armed;

		for (int i = 0; i < limited_esc_count; i++) {
			is_esc_failure = is_esc_failure || (esc_status.esc[i].failures > 0);
		}

		_esc_failure_hysteresis.set_hysteresis_time_from(false, 300_ms);
		_esc_failure_hysteresis.set_state_and_update(is_esc_failure, time_now);

		if (_esc_failure_hysteresis.get_state()) {
			_status.flags.arm_escs = true;
		}

	} else {
		// reset ESC bitfield
		_esc_failure_hysteresis.set_state_and_update(false, time_now);
		_status.flags.arm_escs = false;
	}
}

void FailureDetector::updateImbalancedPropStatus()
{

	if (_sensor_selection_sub.updated()) {
		sensor_selection_s selection;

		if (_sensor_selection_sub.copy(&selection)) {
			_selected_accel_device_id = selection.accel_device_id;
		}
	}

	const bool updated = _vehicle_imu_status_sub.updated(); // save before doing a copy

	// Find the imu_status instance corresponding to the selected accelerometer
	vehicle_imu_status_s imu_status{};
	_vehicle_imu_status_sub.copy(&imu_status);

	if (imu_status.accel_device_id != _selected_accel_device_id) {

		for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (!_vehicle_imu_status_sub.ChangeInstance(i)) {
				continue;
			}

			if (_vehicle_imu_status_sub.copy(&imu_status)
			    && (imu_status.accel_device_id == _selected_accel_device_id)) {
				// instance found
				break;
			}
		}
	}

	if (updated) {

		if (_vehicle_imu_status_sub.copy(&imu_status)) {

			if ((imu_status.accel_device_id != 0)
			    && (imu_status.accel_device_id == _selected_accel_device_id)) {
				const float dt = math::constrain((imu_status.timestamp - _imu_status_timestamp_prev) * 1e-6f, 0.01f, 1.f);
				_imu_status_timestamp_prev = imu_status.timestamp;

				_imbalanced_prop_lpf.setParameters(dt, _imbalanced_prop_lpf_time_constant);

				const float std_x = sqrtf(math::max(imu_status.var_accel[0], 0.f));
				const float std_y = sqrtf(math::max(imu_status.var_accel[1], 0.f));
				const float std_z = sqrtf(math::max(imu_status.var_accel[2], 0.f));

				// Note: the metric is done using standard deviations instead of variances to be linear
				const float metric = (std_x + std_y) / 2.f - std_z;
				const float metric_lpf = _imbalanced_prop_lpf.update(metric);

				const bool is_imbalanced = metric_lpf > _param_fd_imb_prop_thr.get();
				_status.flags.imbalanced_prop = is_imbalanced;
			}
		}
	}
}

void FailureDetector::updateMotorStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status)
{
	// What need to be checked:
	//
	// 1. ESC telemetry disappears completely -> dead ESC or power loss on that ESC
	// 2. ESC failures like overvoltage, overcurrent etc. But DShot driver for example is not populating the field 'esc_report.failures'
	// 3. Motor current too low. Compare drawn motor current to expected value from a parameter
	// -- ESC voltage does not really make sense and is highly dependent on the setup

	// First wait for some ESC telemetry that has the required fields. Before that happens, don't check this ESC
	// Then check

	// Only check while armed
	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const hrt_abstime now = hrt_absolute_time();
		const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);

		actuator_motors_s actuator_motors{};
		_actuator_motors_sub.copy(&actuator_motors);

		// Check individual ESC reports
		for (int esc_status_idx = 0; esc_status_idx < limited_esc_count; esc_status_idx++) {

			const esc_report_s &cur_esc_report = esc_status.esc[esc_status_idx];

			// Map the esc status index to the actuator function index
			const unsigned i_esc = cur_esc_report.actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

			if (i_esc >= actuator_motors_s::NUM_CONTROLS) {
				continue;
			}

			// Check if ESC telemetry was available and valid at some point. This is a prerequisite for the failure detection.
			if (!(_motor_failure_esc_valid_current_mask & (1 << i_esc)) && cur_esc_report.esc_current > 0.0f) {
				_motor_failure_esc_valid_current_mask |= (1 << i_esc);
			}

			// Check for telemetry timeout
			const bool esc_timed_out = now > cur_esc_report.timestamp + 300_ms;
			const bool esc_was_valid = _motor_failure_esc_valid_current_mask & (1 << i_esc);
			const bool esc_timeout_currently_flagged = _motor_failure_esc_timed_out_mask & (1 << i_esc);

			if (esc_was_valid && esc_timed_out && !esc_timeout_currently_flagged) {
				// Set flag
				_motor_failure_esc_timed_out_mask |= (1 << i_esc);

			} else if (!esc_timed_out && esc_timeout_currently_flagged) {
				// Reset flag
				_motor_failure_esc_timed_out_mask &= ~(1 << i_esc);
			}

			// Check if ESC current is too low
			if (cur_esc_report.esc_current > FLT_EPSILON) {
				_motor_failure_esc_has_current[i_esc] = true;
			}

			if (_motor_failure_esc_has_current[i_esc]) {
				float esc_throttle = 0.f;

				if (PX4_ISFINITE(actuator_motors.control[i_esc])) {
					esc_throttle = fabsf(actuator_motors.control[i_esc]);
				}

				const bool throttle_above_threshold = esc_throttle > _param_fd_motor_throttle_thres.get();
				const bool current_too_low = cur_esc_report.esc_current < esc_throttle *
							     _param_fd_motor_current2throttle_thres.get();

				if (throttle_above_threshold && current_too_low && !esc_timed_out) {
					if (_motor_failure_undercurrent_start_time[i_esc] == 0) {
						_motor_failure_undercurrent_start_time[i_esc] = now;
					}

				} else {
					if (_motor_failure_undercurrent_start_time[i_esc] != 0) {
						_motor_failure_undercurrent_start_time[i_esc] = 0;
					}
				}

				if (_motor_failure_undercurrent_start_time[i_esc] != 0
				    && now > (_motor_failure_undercurrent_start_time[i_esc] + (_param_fd_motor_time_thres.get() * 1_ms))
				    && (_motor_failure_esc_under_current_mask & (1 << i_esc)) == 0) {
					// Set flag
					_motor_failure_esc_under_current_mask |= (1 << i_esc);

				} // else: this flag is never cleared, as the motor is stopped, so throttle < threshold
			}
		}

		bool critical_esc_failure = (_motor_failure_esc_timed_out_mask != 0 || _motor_failure_esc_under_current_mask != 0);

		if (critical_esc_failure && !(_status.flags.motor)) {
			// Add motor failure flag to bitfield
			_status.flags.motor = true;

		} else if (!critical_esc_failure && _status.flags.motor) {
			// Reset motor failure flag
			_status.flags.motor = false;
		}

	} else { // Disarmed
		// reset ESC bitfield
		for (int i_esc = 0; i_esc < actuator_motors_s::NUM_CONTROLS; i_esc++) {
			_motor_failure_undercurrent_start_time[i_esc] = 0;
		}

		_motor_failure_esc_under_current_mask = 0;
		_status.flags.motor = false;
	}
}

// ============================================================
// LOCP (Loss-of-Control Protection) —— 失控保护系统实现
// ============================================================
//
// 设计背景：在无遥控器（或遥控器失效）、地面站指令无效的情况下，
// 飞行器可能因传感器故障、动力系统故障、外部撞击等原因进入不可控状态。
// LOCP 通过多维度传感器数据融合分析，自动检测失控状态并触发相应级别的保护动作。
//
// 检测维度说明：
//   ARD (Attitude Rate Detection):    姿态角速度/角加速度异常
//   VRD (Velocity Rate Detection):    速度变化率异常（水平加速度/自由落体/jerk）
//   PRD (Position Rate Detection):    位置/高度变化率异常（急降/振荡/漂移）
//   COD (Current Overdraw Detection): 电流异常（总电流突增/dI/dt尖峰）
//   MTO (MAVLink Timeout):            MAVLink通信超时（心跳/指令/速率）
//   Crash:                             碰撞/撞击瞬时检测（IMU加速度尖峰）
//
// 保护动作分配（动作由 failsafe 状态机硬编码执行，无等级参数）：
//   停桨 (Disarm): ARD / VRD / PRD / COD / Crash —— 飞机自身已失控，
//                  控制回路不可信，"降落"无法可靠执行，直接停桨。
//   降落 (Land):   MTO / OBS —— 飞机自身正常，仅外部输入（链路/机载指令）
//                  异常，控制仍可信，可安全降落。
//
// 全量日志标定（2026-08-13）：6-8 月 136 份纯净日志（已排除 EGO 故障 13 份、
// 卡网 2 份、翻倒 1 份、碰撞 1 份、地面假速度 5 份）各维度阈值裕度 1.3~11 倍；
// 6-8 月 17 份失控日志全部被现有维度覆盖：
//   EGO 故障(13) → VRD_HS 通道（47~107 m/s ≫ 阈值 15，持续 3s+）
//   卡网(309/311) → Crash(107/123) + ARD（ω>5 持续 0.26s/0.14s）
//   翻倒(233) → Crash(105) + ARD（ω>5 持续 0.96s）
//   碰撞(199) → Crash(77)
// ============================================================

void FailureDetector::updateLOCP(const vehicle_status_s &vehicle_status,
				 const vehicle_control_mode_s &vehicle_control_mode)
{
	// 未解锁 或 LOCP 总开关禁用(LOCP_EN=0) 时重置所有 LOCP 状态，不做检测
	if ((vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED)
	    || !_param_locp_en.get()) {
		const hrt_abstime now = hrt_absolute_time();

		_locp_ard_triggered = false;
		_locp_vrd_triggered = false;
		_locp_prd_triggered = false;
		_locp_cod_triggered = false;
		_locp_mto_triggered = false;
		_locp_obs_triggered = false;
		_crash_detected = false;
		_locp_vehicle_healthy = true;
		_rate_window_start = 0;
		_mavlink_msg_counter = 0;
		_sp_was_valid = false;
		_locp_arm_time = 0;

		// --- 重置各维度检测内部状态（计时器 / 历史缓冲 / 前一帧值） ---
		// ARD: 姿态变化率检测
		_rollspeed_prev = 0.f;
		_pitchspeed_prev = 0.f;
		_yawspeed_prev = 0.f;
		_att_rate_high_start = 0;
		_att_rate_timestamp_prev = 0;

		// VRD: 速度变化率检测
		_acc_horiz_prev = 0.f;
		_acc_horiz_timestamp_prev = 0;
		_horiz_spd_high_start = 0;

		// PRD: 位置变化率检测
		for (uint8_t i = 0; i < 10; i++) { _alt_history[i] = 0.f; }
		_alt_history_idx = 0;
		_alt_history_count = 0;

		// COD: 电流异常检测
		for (uint8_t i = 0; i < 20; i++) { _current_sliding_window[i] = 0.f; }
		_current_window_idx = 0;
		_current_window_count = 0;
		_current_prev = 0.f;
		_current_timestamp_prev = 0;

		// MTO: MAVLink 消息超时检测
		_last_mavlink_heartbeat = 0;
		_last_vehicle_command = 0;

		// Crash: 碰撞/撞击检测
		_crash_impact_time = 0;

		// OBS: Offboard Setpoint 异常检测
		_sp_pos_prev[0] = _sp_pos_prev[1] = _sp_pos_prev[2] = 0.f;
		_sp_vel_prev[0] = _sp_vel_prev[1] = _sp_vel_prev[2] = 0.f;
		_sp_yaw_prev = 0.f;

		// 重置 TRD 动力响应校验状态
		_trd_high_thr_start = 0;
		_trd_normal_start = 0;
		_trd_land_try_start = 0;
		_trd_takeover_watch_start = 0;
		_locp_trd_triggered = false;
		_locp_trd_land = false;
		_locp_trd_no_takeover = false;
		_last_gcs_cmd = 0;
		_last_rc_input = 0;

		// 重置各维度迟滞滤波器（防止上次飞行的触发状态残留到下次解锁）
		_ard_hysteresis.set_state_and_update(false, now);
		_vrd_hysteresis.set_state_and_update(false, now);
		_prd_hysteresis.set_state_and_update(false, now);
		_cod_hysteresis.set_state_and_update(false, now);
		_obs_hysteresis.set_state_and_update(false, now);

		return;
	}

	// 记录解锁时刻（用于 COD 启动保护，避免解锁瞬间电流爬升误判）
	if (_locp_arm_time == 0) {
		_locp_arm_time = hrt_absolute_time();
	}

	// 1) ARD: 姿态变化率异常检测（角加速度尖峰 + 持续高角速率）
	vehicle_angular_velocity_s ang_vel;
	if (_vehicle_angular_velocity_sub.update(&ang_vel)) {
		_locp_ard_triggered = _param_locp_ard_en.get() && checkAttitudeRateAnomaly(ang_vel);
	}

	// 2) VRD: 速度变化率异常检测（水平加速度/自由落体/jerk）
	// 3) PRD: 位置变化率异常检测（急降/高度振荡/水平漂移）
	//      VRD 和 PRD 共用同一份 local_position 数据，放在同一个 if 块中
	vehicle_local_position_s loc;
	if (_vehicle_local_position_sub.update(&loc)) {
		_locp_vrd_triggered = _param_locp_vrd_en.get() && checkVelocityRateAnomaly(loc);

		home_position_s home;
		_home_position_sub.copy(&home);
		_locp_prd_triggered = _param_locp_prd_en.get() && checkPositionRateAnomaly(loc, home);
	}

	// 4) COD: 电流异常检测（总电流突增/dI/dt尖峰）
	// 启动保护：解锁后 LOCP_COD_ARM_DLY 秒内跳过 COD
	// （电机启动瞬间电流从 0 爬升到悬停电流是正常行为，不应触发保护）
	const bool cod_startup_guard = (_locp_arm_time != 0)
				      && (hrt_absolute_time() - _locp_arm_time <
					  static_cast<hrt_abstime>(_param_locp_cod_arm_dly.get() * 1_s));
	battery_status_s bat;
	if (_battery_status_sub.update(&bat)) {
		_locp_cod_triggered = _param_locp_cod_en.get() && !cod_startup_guard && checkCurrentAnomaly(bat);
	}

	// 5) MTO: MAVLink 消息超时检测（心跳超时/指令超时/消息速率骤降）
	_locp_mto_triggered = _param_locp_mto_en.get() && checkMavlinkTimeout();

	// 6) OBS: Offboard Setpoint 异常检测（数值跳变/NaN注入）
	_locp_obs_triggered = _param_locp_obs_en.get() && checkOffboardSetpointSanity();

	// 7) Crash: 碰撞/撞击瞬时检测（基于 IMU 加速度范数阈值，无迟滞）
	// 由 LOCP_CRASH_EN 控制开关（0=禁用碰撞检测）
	_crash_detected = _param_locp_crash_en.get() && checkCrashImpact();

	// 8) TRD: 动力响应校验（高油门但垂直加速度不足）
	// 检测"指标矛盾"：油门指令高（期望大推力），但实际垂直加速度不足。
	// 覆盖：卡网（顶部/侧边）、动力丢失、桨损坏、控制失效。
	// 分级动作：≤LOCP_TRD_LAND_H 请求降落（柔和），否则由 failsafe 直接停桨。
	// 锁存防空窗：确认触发后不因单帧恢复而复位，仅连续正常 3s 才复位。
	// 由 LOCP_TRD_EN 控制开关（0=禁用整个 TRD 检测）
	_locp_trd_triggered = _param_locp_trd_en.get() && checkThrustResponse(vehicle_status);

	// 各维度触发标志（locp_*_triggered）直接由 failsafe 状态机按固定动作执行：
	//   ARD/VRD/PRD/COD/Crash → 停桨 (Disarm)
	//   MTO/OBS → 健康则降落，不健康则停桨
	// 无等级仲裁，动作硬编码，行为可预测。

	// 飞机自身状态健康检查：MTO/OBS 降落动作的安全门槛
	// （独立于各维度 EN 开关的实时状态确认，见 checkVehicleHealthy()）
	_locp_vehicle_healthy = checkVehicleHealthy();
}

bool FailureDetector::checkAttitudeRateAnomaly(const vehicle_angular_velocity_s &ang_vel)
{
	const hrt_abstime now = hrt_absolute_time();
	// 使用真实时间戳间隔计算 dt，避免硬编码 100Hz 假设（角速度话题可能非 100Hz 更新）
	float dt = (ang_vel.timestamp - _att_rate_timestamp_prev) * 1e-6f;
	if (_att_rate_timestamp_prev == 0 || dt <= 0.f || dt > 1.f) { dt = 0.01f; }
	_att_rate_timestamp_prev = ang_vel.timestamp;

	// --- 角加速度检测：三轴角加速度分别与阈值比较 ---
	// 通过前后两帧角速度差分计算角加速度 (rad/s²)
	float d_roll  = fabsf(ang_vel.xyz[0] - _rollspeed_prev)  / dt;
	float d_pitch = fabsf(ang_vel.xyz[1] - _pitchspeed_prev) / dt;
	float d_yaw   = fabsf(ang_vel.xyz[2] - _yawspeed_prev)   / dt;

	// 保存当前角速度，供下一帧计算角加速度
	_rollspeed_prev  = ang_vel.xyz[0];
	_pitchspeed_prev = ang_vel.xyz[1];
	_yawspeed_prev   = ang_vel.xyz[2];

	// 角加速度超限判定：任意轴角加速度 > 对应阈值即触发（LOCP_ARD_ACC_EN 控制）
	bool accel_fault = _param_locp_ard_acc_en.get()
			&& ((d_roll  > _param_locp_ard_r_max.get())
			    || (d_pitch > _param_locp_ard_p_max.get())
			    || (d_yaw   > _param_locp_ard_y_max.get()));

	// --- 持续高角速率检测：角速率超过设定值并持续超过设定时间 ---
	// 仅检测 Roll 和 Pitch（Yaw 通常变化范围大，不作为持续判断条件）
	// 注意：必须检查 _att_rate_high_start != 0，否则计时器未启动时
	// (now - 0) 为系统运行时间（巨大），会导致单帧角速率超限即误判。
	// 由 LOCP_ARD_RATE_EN 控制开关。
	bool sustained_high =
		_param_locp_ard_rate_en.get() && (
		(fabsf(ang_vel.xyz[0]) > _param_locp_ard_rsp.get()
		 && _att_rate_high_start != 0
		 && (now - _att_rate_high_start) > static_cast<hrt_abstime>(_param_locp_ard_dur.get() * 1_s))
		||
		(fabsf(ang_vel.xyz[1]) > _param_locp_ard_psp.get()
		 && _att_rate_high_start != 0
		 && (now - _att_rate_high_start) > static_cast<hrt_abstime>(_param_locp_ard_dur.get() * 1_s)));

	// 更新持续高角速率计时器：
	// 当任意轴角速率超出设定点时开始计时，全部回落到阈值以下时清零
	if (fabsf(ang_vel.xyz[0]) > _param_locp_ard_rsp.get()
	    || fabsf(ang_vel.xyz[1]) > _param_locp_ard_psp.get()
	    || fabsf(ang_vel.xyz[2]) > _param_locp_ard_ysp.get()) {
		if (_att_rate_high_start == 0) {
			_att_rate_high_start = now;
		}
	} else {
		_att_rate_high_start = 0;
	}

	bool triggered = accel_fault || sustained_high;

	// 迟滞滤波：故障条件必须持续超过 LOCP_ARD_T 秒才确认触发
	// 防止传感器瞬时噪声或短时扰动导致误判
	_ard_hysteresis.set_hysteresis_time_from(false,
		static_cast<hrt_abstime>(_param_locp_ard_t.get() * 1_s));
	_ard_hysteresis.set_state_and_update(triggered, now);

	return _ard_hysteresis.get_state();
}

bool FailureDetector::checkCrashImpact()
{
	// 碰撞/撞击瞬时检测：基于 IMU 三轴加速度范数与阈值比较
	// 不同于其他检测维度，碰撞检测是瞬时触发、无迟滞滤波的
	// 触发后锁存 2 秒，然后自动清除
	vehicle_acceleration_s accel;
	if (!_vehicle_acceleration_sub.update(&accel)) {
		return _crash_detected; // 无新数据时保持上一帧状态
	}

	// 计算加速度范数（三轴合成加速度幅值）
	const float accel_norm = sqrtf(accel.xyz[0] * accel.xyz[0] +
				       accel.xyz[1] * accel.xyz[1] +
				       accel.xyz[2] * accel.xyz[2]);

	// 碰撞阈值：约 ~80 m/s² (~8g)，可通过 LOCP_CRASH_THR 参数调节
	const float threshold = _param_locp_crash_thr.get();

	if (accel_norm > threshold) {
		_crash_impact_time = hrt_absolute_time();
		return true;
	}

	// 锁存 2 秒后自动清除：防止碰撞后持续触发，同时给予足够时间让飞控响应
	if (_crash_detected && (hrt_absolute_time() - _crash_impact_time > 2_s)) {
		return false;
	}

	return _crash_detected;
}

bool FailureDetector::checkVelocityRateAnomaly(const vehicle_local_position_s &loc)
{
	const hrt_abstime now = hrt_absolute_time();

	// --- 水平加速度异常检测 ---
	// 计算水平面合成加速度 (ax² + ay²) 的平方根，与阈值比较
	float acc_horiz = sqrtf(loc.ax * loc.ax + loc.ay * loc.ay);
	bool horiz_fault = _param_locp_vrd_ah_en.get() && (acc_horiz > _param_locp_vrd_ah_max.get());

	// --- 自由落体/动力急降检测 ---
	// 条件1：垂直加速度 az 超出阈值（向下加速）
	// 条件2：垂直速度 vz 已经超过阈值（正在快速下降）
	// 两个条件同时满足才判定为自由落体/急降（LOCP_VRD_FF_EN 控制）
	bool freefall = _param_locp_vrd_ff_en.get()
		     && (loc.az > _param_locp_vrd_ad_max.get())
		     && (loc.vz > _param_locp_vrd_vzd_max.get());

	// --- Jerk（加加速度）检测 ---
	// Jerk = 水平加速度的变化率 = |Δa_horiz| / dt
	// 检测加速度的突变（急加速或急减速）
	// 使用真实时间戳间隔计算 dt，避免硬编码 100Hz 假设导致误判
	float dt = (loc.timestamp - _acc_horiz_timestamp_prev) * 1e-6f;
	if (_acc_horiz_timestamp_prev == 0 || dt <= 0.f || dt > 1.f) { dt = 0.01f; }
	_acc_horiz_timestamp_prev = loc.timestamp;
	float jerk = fabsf(acc_horiz - _acc_horiz_prev) / dt;
	bool jerk_fault = _param_locp_vrd_jk_en.get() && (jerk > _param_locp_vrd_jerk.get());
	_acc_horiz_prev = acc_horiz;

	// --- 水平速度持续检测（温和乱飞） ---
	// 水平面合成速度超过低阈值（LOCP_VRD_HS_MAX）并持续达到
	// LOCP_VRD_HS_DUR 秒 → 检测飞控发疯乱飞 / EKF 发散导致的意外漂移。
	// 标定依据（2026-08-13，6-8 月 136 份纯净日志，已排除地面假速度日志
	// log_215~219）：正常飞行 P99.9=3.19 m/s（本机最大 10 m/s，阈值 15 满速
	// 也不触发）；EGO 视觉故障（7-8 月 13 份同源日志）速度 47~107 m/s
	// 持续 9~330s，可稳定捕获。
	float horiz_spd = sqrtf(loc.vx * loc.vx + loc.vy * loc.vy);
	const bool spd_high = horiz_spd > _param_locp_vrd_hs_max.get();

	if (spd_high) {
		if (_horiz_spd_high_start == 0) {
			_horiz_spd_high_start = now;
		}

	} else {
		_horiz_spd_high_start = 0;
	}

	const bool spd_fault = _param_locp_vrd_hs_en.get() && spd_high
			       && (_horiz_spd_high_start != 0)
			       && (now - _horiz_spd_high_start) >
			       static_cast<hrt_abstime>(_param_locp_vrd_hs_dur.get() * 1_s);

	bool triggered = horiz_fault || freefall || jerk_fault || spd_fault;

	// 迟滞滤波：故障条件需持续 LOCP_VRD_T 秒
	_vrd_hysteresis.set_hysteresis_time_from(false,
		static_cast<hrt_abstime>(_param_locp_vrd_t.get() * 1_s));
	_vrd_hysteresis.set_state_and_update(triggered, now);

	return _vrd_hysteresis.get_state();
}

bool FailureDetector::checkPositionRateAnomaly(const vehicle_local_position_s &loc,
	const home_position_s &home)
{
	const hrt_abstime now = hrt_absolute_time();

	// --- 急降检测 ---
	// 计算相对于 Home 点的高度下降量
	float alt_drop = loc.z - home.z;
	// 条件1：垂直速度超过阈值（正在快速下降）
	// 条件2：高度下降量超过阈值（已经下降了足够多）
	bool rapid_descent = _param_locp_prd_des_en.get()
			  && (loc.vz > _param_locp_prd_vz_max.get())
			  && (alt_drop > _param_locp_prd_adrop.get());

	// --- 高度振荡检测（滑动窗口标准差法） ---
	// 维护最近 10 帧高度数据的环形缓冲，计算标准差判断高度是否剧烈波动
	_alt_history[_alt_history_idx] = loc.z;
	_alt_history_idx = (_alt_history_idx + 1) % 10;  // 环形索引递增
	if (_alt_history_count < 10) { _alt_history_count++; }

	// 计算滑动窗口均值
	float mean = 0.f;
	for (uint8_t i = 0; i < _alt_history_count; i++) { mean += _alt_history[i]; }
	mean /= _alt_history_count;

	// 计算滑动窗口方差和标准差
	float variance = 0.f;
	for (uint8_t i = 0; i < _alt_history_count; i++) {
		float d = _alt_history[i] - mean;
		variance += d * d;
	}
	variance /= _alt_history_count;
	float alt_std = sqrtf(variance);
	// 至少积累 5 帧数据才开始判断，标准差超过阈值即触发
	bool alt_oscillation = _param_locp_prd_osc_en.get()
			      && (_alt_history_count >= 5) && (alt_std > _param_locp_prd_astd.get());

	// --- 水平漂移检测 ---
	// 计算水平面合成速度 (vx² + vy²) 的平方根
	float horiz_spd = sqrtf(loc.vx * loc.vx + loc.vy * loc.vy);
	bool position_drift = _param_locp_prd_drf_en.get() && (horiz_spd > _param_locp_prd_hspd.get());

	bool triggered = rapid_descent || alt_oscillation || position_drift;

	// 迟滞滤波：故障条件需持续 LOCP_PRD_T 秒
	_prd_hysteresis.set_hysteresis_time_from(false,
		static_cast<hrt_abstime>(_param_locp_prd_t.get() * 1_s));
	_prd_hysteresis.set_state_and_update(triggered, now);

	return _prd_hysteresis.get_state();
}

bool FailureDetector::checkCurrentAnomaly(const battery_status_s &bat)
{
	const hrt_abstime now = hrt_absolute_time();

	// 数据有效性检查：剔除明显不合理的电流读数
	if (bat.current_a < -0.5f || bat.current_a > 200.f) { return false; } // 无效数据

	// --- 总电流突增检测（滑动窗口平均偏差法） ---
	// 维护最近 20 帧电流数据的环形缓冲，实时比较当前值与滑动均值的偏差
	_current_sliding_window[_current_window_idx] = bat.current_a;
	_current_window_idx = (_current_window_idx + 1) % 20;  // 环形索引递增
	if (_current_window_count < 20) { _current_window_count++; }

	// 计算滑动窗口平均电流
	float avg = 0.f;
	for (uint8_t i = 0; i < _current_window_count; i++) { avg += _current_sliding_window[i]; }
	avg /= _current_window_count;

	// 条件1：当前电流偏离滑动均值超过阈值（突增）
	// 条件2：当前电流绝对值超过最大允许值
	float deviation = bat.current_a - avg;
	bool total_surge = _param_locp_cod_srg_en.get()
			&& ((deviation > _param_locp_cod_delta_i.get())
			    || (bat.current_a > _param_locp_cod_max_i.get()));

	// --- 电流变化率 (dI/dt) 尖峰检测 ---
	// 通过前后两帧电流差分计算 dI/dt (A/s)，检测瞬间电流尖峰
	// 使用真实时间戳间隔计算 dt：battery_status 更新率通常不是 100Hz，
	// 硬编码 0.01s 会把 dI/dt 放大（解锁瞬间电流爬升被误判为尖峰）
	float dt = (bat.timestamp - _current_timestamp_prev) * 1e-6f;
	if (_current_timestamp_prev == 0 || dt <= 0.f || dt > 1.f) { dt = 0.01f; }
	_current_timestamp_prev = bat.timestamp;
	float di_dt = fabsf(bat.current_a - _current_prev) / dt;
	bool current_spike = _param_locp_cod_spk_en.get() && (di_dt > _param_locp_cod_di_dt.get());
	_current_prev = bat.current_a;

	bool triggered = total_surge || current_spike;

	// 迟滞滤波：故障条件需持续 LOCP_COD_T 秒
	_cod_hysteresis.set_hysteresis_time_from(false,
		static_cast<hrt_abstime>(_param_locp_cod_t.get() * 1_s));
	_cod_hysteresis.set_state_and_update(triggered, now);

	return _cod_hysteresis.get_state();
}

bool FailureDetector::checkThrustResponse(const vehicle_status_s &vehicle_status)
{
	// TRD (Thrust Response Detection) —— 动力响应校验
	//
	// 通用检测原则：检测"应相关的指标却互相矛盾"——
	// 油门指令高（期望大推力），但实际垂直加速度不足（该加速却没加速）。
	//
	// 覆盖场景：卡网（顶部/侧边）、动力丢失、桨损坏、控制失效。
	// 不特指某一故障，只看"高油门 ↔ 运动响应不足"的指标矛盾。
	//
	// 垂直加速度说明：使用 EKF 去重力后的运动加速度（vehicle_local_position.az，
	// NED 向下为正）。正常满油门爬升 az 约 -30 m/s²（大幅向上加速），
	// 被卡/动力失效时 az 接近 0 或为正。判定"无向上加速"：az > -LOCP_TRD_AZ_MIN。
	//
	// 状态机：确认触发后锁存（防空窗，不因单帧恢复而复位），
	// 仅当连续正常 3s 才复位（避免撞网瞬间乱飞检测复位造成空窗）。

	const hrt_abstime now = hrt_absolute_time();

	// --- 读取当前油门指令（4 电机最大通道） ---
	actuator_motors_s act;
	float throttle = 0.f;

	if (_actuator_motors_sub.copy(&act)) {
		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
			if (PX4_ISFINITE(act.control[i])) {
				throttle = math::max(throttle, fabsf(act.control[i]));
			}
		}
	}

	// --- 读取垂直加速度（EKF 去重力）与高度/速度 ---
	vehicle_local_position_s loc;
	float az = 0.f;
	float height = 0.f;
	float horiz_speed = 0.f;
	float vz = 0.f;

	if (_vehicle_local_position_sub.copy(&loc)) {
		az = loc.az;
		height = -loc.z;   // NED: z 负 = 高处，高度 = -z
		horiz_speed = sqrtf(loc.vx * loc.vx + loc.vy * loc.vy);
		vz = loc.vz;
	}

	// --- 指标矛盾判定：高油门 且 完全无运动响应 ---
	// 2026-08-13 按日志标定（卡网 log_309/311 vs 正常日志）：
	//   卡网特征：az≈0（无向上加速）、|vz|≈0（无升降）、水平速度≈0（挂住不动）
	//   满油门平飞：水平速度大 → 动力正常，不触发
	//   快速升降：|vz| 大 → 动力正常，不触发
	//   地面测试：高度≈0 → 排除
	const bool high_throttle = throttle > _param_locp_trd_thr_high.get();
	const bool no_upward_accel = az > -_param_locp_trd_az_min.get();   // 无足够向上加速
	const bool no_vertical_move = fabsf(vz) < _param_locp_trd_vz_max.get();      // 无显著升降
	const bool no_horiz_move = horiz_speed < _param_locp_trd_hs_max.get();        // 无水平移动
	const bool above_min_height = height > _param_locp_trd_min_h.get();           // 高于最低检测高度
	const bool anomaly = high_throttle && no_upward_accel
			     && no_vertical_move && no_horiz_move && above_min_height;

	// --- 状态机：去抖确认 + 锁存 + 复位 ---
	if (_locp_trd_triggered) {
		// 已锁存触发：检查连续正常 3s 复位
		if (!anomaly) {
			if (_trd_normal_start == 0) {
				_trd_normal_start = now;

			} else if (now - _trd_normal_start > 3_s) {
				// 连续正常 3 秒 → 解除锁存，恢复正常
				_locp_trd_triggered = false;
				_trd_normal_start = 0;
			}

		} else {
			_trd_normal_start = 0;
		}

	} else {
		_trd_normal_start = 0;

		if (anomaly) {
			// 去抖确认：异常需持续 LOCP_TRD_T 秒
			if (_trd_high_thr_start == 0) {
				_trd_high_thr_start = now;

			} else if (now - _trd_high_thr_start > static_cast<hrt_abstime>(_param_locp_trd_t.get() * 1_s)) {
				_locp_trd_triggered = true;   // 确认触发（锁存）
				_trd_high_thr_start = 0;
			}

		} else {
			_trd_high_thr_start = 0;
		}
	}

	// --- 分级动作决策 ---
	// ≤LOCP_TRD_LAND_H：请求降落（柔和），并跟踪降落尝试时间；
	//   >LOCP_TRD_LTOUT 仍触发（被网吊住降不下来）→ 转停桨（兜底）
	// >LOCP_TRD_LAND_H 或已抛飞：直接停桨
	if (_locp_trd_triggered) {
		if (height <= _param_locp_trd_land_h.get()) {
			// 低高度：请求降落，记录降落尝试开始时刻
			if (_trd_land_try_start == 0) {
				_trd_land_try_start = now;
			}

			_locp_trd_land = true;

			// 降落尝试超时仍触发（被网吊住）→ 转停桨
			if (_trd_land_try_start != 0
			    && (now - _trd_land_try_start) > static_cast<hrt_abstime>(_param_locp_trd_land_tout.get() * 1_s)) {
				_locp_trd_land = false;
			}

		} else {
			// 高高度 / 抛飞：直接停桨
			_trd_land_try_start = 0;
			_locp_trd_land = false;
		}

	} else {
		_trd_land_try_start = 0;
		_locp_trd_land = false;
	}

	// --- 接管状态：RC + QGC 双通道 + 接管观察窗口（接管无效检测） ---
	// 追踪最近 QGC 指令（vehicle_command），2 秒内有指令视为地面站在线
	vehicle_command_s cmd;

	while (_vehicle_command_sub.update(&cmd)) {
		_last_gcs_cmd = cmd.timestamp;
	}

	// RC 可用性：PX4 v1.17 的 vehicle_status 无 rc_signal_lost 字段，
	// 改用话题新鲜度判断——RC 输入经 Sticks 处理后发布 manual_control_setpoint
	// （约 50Hz 持续更新），RC 失联时该话题停止更新，1s 内无新数据视为失联。
	manual_control_setpoint_s mcs;

	if (_manual_control_setpoint_sub.update(&mcs)) {
		_last_rc_input = mcs.timestamp;
	}

	const bool rc_available = (_last_rc_input != 0) && (now - _last_rc_input) < 1_s;
	const bool gcs_active = (now - _last_gcs_cmd) < 2_s;
	const bool takeover_available = rc_available || gcs_active;

	if (_locp_trd_triggered) {
		if (takeover_available) {
			// 有人可接管（RC 在线 或 QGC 有指令）→ 观察窗口
			if (anomaly) {
				// 异常持续（仍高油门无响应）→ 观察窗口计时
				if (_trd_takeover_watch_start == 0) {
					_trd_takeover_watch_start = now;
				}

				// 观察窗口到期仍无响应 → 接管无效（真失控）→ 强制停桨
				_locp_trd_no_takeover = (now - _trd_takeover_watch_start)
							> static_cast<hrt_abstime>(_param_locp_trd_to_watch.get() * 1_s);

			} else {
				// 已恢复正常（用户接管成功，动力恢复）→ 清零窗口，等待锁存复位
				_trd_takeover_watch_start = 0;
				_locp_trd_no_takeover = false;
			}

		} else {
			// 无人可接管（RC 失联 且 QGC 无指令）→ 立即强制停桨
			_locp_trd_no_takeover = true;
			_trd_takeover_watch_start = 0;
		}

	} else {
		_locp_trd_no_takeover = false;
		_trd_takeover_watch_start = 0;
	}

	return _locp_trd_triggered;
}

bool FailureDetector::checkMavlinkTimeout()
{
	const hrt_abstime now = hrt_absolute_time();

	// --- 心跳超时检测 ---
	// 通过 telemetry_status 话题追踪最后一次 MAVLink 心跳包时间戳
	telemetry_status_s telemetry;
	if (_telemetry_status_sub.update(&telemetry)) {
		_last_mavlink_heartbeat = telemetry.timestamp;
	}

	// --- 指令超时检测 ---
	// 追踪最后一次收到 vehicle_command 的时间
	vehicle_command_s cmd;
	while (_vehicle_command_sub.update(&cmd)) {
		_last_vehicle_command = cmd.timestamp;
	}

	// 心跳超时判定：距离最后一次心跳超过 LOCP_MTO_HB_T 秒
	bool heartbeat_lost = _param_locp_mto_hb_en.get()
		&& (_last_mavlink_heartbeat != 0)
		&& (now - _last_mavlink_heartbeat >
		    static_cast<hrt_abstime>(_param_locp_mto_hb_t.get() * 1_s));

	// 指令超时判定：距离最后一次指令超过 LOCP_MTO_CMD_T 秒
	bool cmd_timeout = _param_locp_mto_cmd_en.get()
		&& (_last_vehicle_command != 0)
		&& (now - _last_vehicle_command >
		    static_cast<hrt_abstime>(_param_locp_mto_cmd_t.get() * 1_s));

	// --- 消息速率骤降检测 ---
	// 以 1 秒为滑动窗口统计 MAVLink 消息接收速率
	// 当速率低于 LOCP_MTO_RATE (Hz) 时判定为速率异常
	_mavlink_msg_counter++;
	if (_rate_window_start == 0) {
		_rate_window_start = now;
	}

	float window_elapsed = (now - _rate_window_start) * 1e-6f;
	bool rate_drop = false;
	if (window_elapsed > 1.0f) {
		// 窗口满 1 秒后计算速率并重置
		float msg_rate = _mavlink_msg_counter / window_elapsed;
		rate_drop = msg_rate < _param_locp_mto_rate.get();
		// 重置速率统计窗口
		_mavlink_msg_counter = 0;
		_rate_window_start = now;
	}

	// 触发条件：心跳丢失 或 (指令超时 且 消息速率骤降)
	return heartbeat_lost || (cmd_timeout && rate_drop);
}

bool FailureDetector::checkVehicleHealthy()
{
	// 飞机自身状态健康检查 —— MTO/OBS 降落动作的安全门槛
	//
	// 降落是"受控动作"：依赖正常的位置/姿态控制回路。仅当飞机自身
	// 状态正常时才允许执行降落；否则即使只有 MTO/OBS（外部输入异常）
	// 触发，也必须直接停桨。
	//
	// 为什么需要独立检查（而非直接依赖 ARD/VRD 等触发标志）：
	//   1. 各检测维度可能被用户禁用（EN=0），此时失控不会被标志位反映；
	//   2. 失控可能尚未达到维度的"持续确认"阈值（迟滞/DUR 未满）。
	// 因此本检查是【不受 EN 开关控制的实时状态确认】。
	//
	// 判定规则（任一项异常 → 不健康，门槛使用独立参数 LOCP_HC_*）：
	//   - 角速率异常：|ω_r/p| > LOCP_HC_RATE_MAX 或 |ω_yaw| > LOCP_HC_YAW_MAX
	//   - 水平速度异常：√(vx²+vy²) > LOCP_HC_HS_MAX（EGO 视觉故障等）
	//   - 垂直下降异常：vz > LOCP_HC_VZ_MAX（急坠）
	// 数据无效或 NaN 时视为不健康（保守：无法确认受控 → 停桨）。
	// 标定（2026-08-13，6-8 月 136 份纯净日志）：
	//   R/P 角速率 P99.9=1.13 → 阈值 2（裕度 1.8 倍）；
	//   Yaw 角速率 P99.9=1.38 → 阈值 5（正常旋转可达 3.14，裕度 3.6 倍）；
	//   水平速度 P99.9=3.19（已排除地面假速度日志）→ 阈值 15（裕度 4.7 倍）；
	//   |vz| P99.9=1.04 → 阈值 10（裕度约 5 倍）。

	bool healthy = true;

	// 1) 角速率健康检查（IMU 角速度）
	//    门槛使用独立参数 LOCP_HC_*（不复用 ARD_RSP/PSP/YSP，
	//    避免调整 ARD 检测阈值时间接改变健康门槛）
	vehicle_angular_velocity_s ang_vel;
	if (_vehicle_angular_velocity_sub.copy(&ang_vel)) {
		if (!PX4_ISFINITE(ang_vel.xyz[0]) || !PX4_ISFINITE(ang_vel.xyz[1]) || !PX4_ISFINITE(ang_vel.xyz[2])) {
			healthy = false;  // 角速度数据异常（NaN）

		} else if (fabsf(ang_vel.xyz[0]) > _param_locp_hc_rate_max.get()
			   || fabsf(ang_vel.xyz[1]) > _param_locp_hc_rate_max.get()
			   || fabsf(ang_vel.xyz[2]) > _param_locp_hc_yaw_max.get()) {
			healthy = false;  // 角速率超限（飞机在乱飞）
		}

	} else {
		healthy = false;  // 角速度数据不可用 → 保守判定不健康
	}

	// 2) 速度健康检查（EKF 位置/速度）
	//    门槛使用独立参数 LOCP_HC_*（不复用 VRD_HS_MAX/PRD_VZ_MAX）
	vehicle_local_position_s loc;
	if (_vehicle_local_position_sub.copy(&loc)) {
		if (!PX4_ISFINITE(loc.vx) || !PX4_ISFINITE(loc.vy) || !PX4_ISFINITE(loc.vz)) {
			healthy = false;  // 速度数据异常（NaN，如 EGO 视觉故障）

		} else {
			const float horiz_spd = sqrtf(loc.vx * loc.vx + loc.vy * loc.vy);
			if (horiz_spd > _param_locp_hc_hs_max.get()) {
				healthy = false;  // 水平速度异常（EGO 故障等）
			}
			if (loc.vz > _param_locp_hc_vz_max.get()) {
				healthy = false;  // 垂直下降速度异常（急坠）
			}
		}

	} else {
		healthy = false;  // 位置/速度数据不可用 → 保守判定不健康
	}

	return healthy;
}

bool FailureDetector::checkOffboardSetpointSanity()
{
	// OBS (Offboard Setpoint Sanity) —— 机载计算机 Setpoint 异常检测
	//
	// 检测目标：
	//   1. 数值跳变 (Spike)：相邻两帧 setpoint 中任意受控轴的变化量超过阈值
	//   2. NaN 注入：上一帧有效的数值字段，在当前帧突变为 NaN
	//
	// 适用数据源：trajectory_setpoint（由 MAVROS/机载计算机通过
	//   SET_POSITION_TARGET_LOCAL_NED MAVLink 消息发布）
	//
	// 注意：trajectory_setpoint 中用 NaN 表示"该轴不受控"，
	// 因此 NaN 本身是正常的 —— 只有从 非NaN→NaN 的突变才是异常。

	const hrt_abstime now = hrt_absolute_time();

	trajectory_setpoint_s sp;
	if (!_trajectory_setpoint_sub.update(&sp)) {
		// 无新数据：保持上一帧的 OBS 状态
		return _locp_obs_triggered;
	}

	bool anomaly = false;

	// 检查各轴 setpoint 是否为有效数值（非 NaN）
	bool pos_valid[3] = {
		PX4_ISFINITE(sp.position[0]),
		PX4_ISFINITE(sp.position[1]),
		PX4_ISFINITE(sp.position[2])
	};
	bool vel_valid[3] = {
		PX4_ISFINITE(sp.velocity[0]),
		PX4_ISFINITE(sp.velocity[1]),
		PX4_ISFINITE(sp.velocity[2])
	};
	bool yaw_valid = PX4_ISFINITE(sp.yaw);

	if (_sp_was_valid) {
		// --- 1) NaN 注入检测：上一帧有效 → 当前帧突变为 NaN（LOCP_OBS_NAN_EN 控制）---
		if (_param_locp_obs_nan_en.get()) {
			for (int i = 0; i < 3; i++) {
				if (PX4_ISFINITE(_sp_pos_prev[i]) && !pos_valid[i]) {
					anomaly = true;  // 位置 setpoint 突变为 NaN
				}
				if (PX4_ISFINITE(_sp_vel_prev[i]) && !vel_valid[i]) {
					anomaly = true;  // 速度 setpoint 突变为 NaN
				}
			}
			if (PX4_ISFINITE(_sp_yaw_prev) && !yaw_valid) {
				anomaly = true;      // Yaw setpoint 突变为 NaN
			}
		}

		// --- 2) 数值跳变检测：相邻帧间变化超过阈值（LOCP_OBS_JMP_EN 控制）---
		if (_param_locp_obs_jmp_en.get()) {
			for (int i = 0; i < 3; i++) {
				if (PX4_ISFINITE(_sp_pos_prev[i]) && pos_valid[i]) {
					if (fabsf(sp.position[i] - _sp_pos_prev[i]) > _param_locp_obs_jump_pos.get()) {
						anomaly = true;  // 位置跳变
					}
				}
				if (PX4_ISFINITE(_sp_vel_prev[i]) && vel_valid[i]) {
					if (fabsf(sp.velocity[i] - _sp_vel_prev[i]) > _param_locp_obs_jump_vel.get()) {
						anomaly = true;  // 速度跳变
					}
				}
			}
			if (PX4_ISFINITE(_sp_yaw_prev) && yaw_valid) {
				// Yaw 跳变需要处理 ±PI 环绕
				float yaw_diff = fabsf(sp.yaw - _sp_yaw_prev);
				if (yaw_diff > M_PI_F) {
					yaw_diff = 2.0f * M_PI_F - yaw_diff;
				}
				if (yaw_diff > _param_locp_obs_jump_yaw.get()) {
					anomaly = true;
				}
			}
		}
	}

	// 更新上一帧状态，供下次检测使用
	_sp_was_valid = true;
	for (int i = 0; i < 3; i++) {
		if (pos_valid[i]) { _sp_pos_prev[i] = sp.position[i]; }
		if (vel_valid[i]) { _sp_vel_prev[i] = sp.velocity[i]; }
	}
	if (yaw_valid) { _sp_yaw_prev = sp.yaw; }

	// 迟滞滤波
	_obs_hysteresis.set_hysteresis_time_from(false,
		static_cast<hrt_abstime>(_param_locp_obs_t.get() * 1_s));
	_obs_hysteresis.set_state_and_update(anomaly, now);

	return _obs_hysteresis.get_state();
}
