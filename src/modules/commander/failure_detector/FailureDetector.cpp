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

void FailureInjector::update()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);
		const int instance = static_cast<int>(vehicle_command.param3 + 0.5f);

		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SYSTEM_MOTOR) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, motors ok");
				supported = false;

				// 0 to signal all
				if (instance == 0) {
					supported = true;

					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d ok", i);
						_esc_blocked &= ~(1 << i);
						_esc_wrong &= ~(1 << i);
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					supported = true;

					PX4_INFO("CMD_INJECT_FAILURE, motor %d ok", instance - 1);
					_esc_blocked &= ~(1 << (instance - 1));
					_esc_wrong &= ~(1 << (instance - 1));
				}
			}

			else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, motors off");
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d off", i);
						_esc_blocked |= 1 << i;
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, motor %d off", instance - 1);
					_esc_blocked |= 1 << (instance - 1);
				}
			}

			else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				PX4_INFO("CMD_INJECT_FAILURE, motors wrong");
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d wrong", i);
						_esc_wrong |= 1 << i;
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, motor %d wrong", instance - 1);
					_esc_wrong |= 1 << (instance - 1);
				}
			}
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
	}

}

void FailureInjector::manipulateEscStatus(esc_status_s &status)
{
	if (_esc_blocked != 0 || _esc_wrong != 0) {
		unsigned offline = 0;

		for (int i = 0; i < status.esc_count; i++) {
			const unsigned i_esc = status.esc[i].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

			if (_esc_blocked & (1 << i_esc)) {
				unsigned function = status.esc[i].actuator_function;
				memset(&status.esc[i], 0, sizeof(status.esc[i]));
				status.esc[i].actuator_function = function;
				offline |= 1 << i;

			} else if (_esc_wrong & (1 << i_esc)) {
				// Create wrong rerport for this motor by scaling key values up and down
				status.esc[i].esc_voltage *= 0.1f;
				status.esc[i].esc_current *= 0.1f;
				status.esc[i].esc_rpm *= 10.0f;
			}
		}

		status.esc_online_flags &= ~offline;
	}
}

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

	const hrt_abstime time_now = hrt_absolute_time();

	// Only check while armed
	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
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
			const hrt_abstime telemetry_age = time_now - cur_esc_report.timestamp;
			const bool esc_timed_out = telemetry_age > 300_ms;

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
						_motor_failure_undercurrent_start_time[i_esc] = time_now;
					}

				} else {
					if (_motor_failure_undercurrent_start_time[i_esc] != 0) {
						_motor_failure_undercurrent_start_time[i_esc] = 0;
					}
				}

				if (_motor_failure_undercurrent_start_time[i_esc] != 0
				    && (time_now - _motor_failure_undercurrent_start_time[i_esc]) > _param_fd_motor_time_thres.get() * 1_ms
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
//   COD (Current Overdraw Detection): 电流异常（总电流突增/dI/dt尖峰/单路ESC过流）
//   MTO (MAVLink Timeout):            MAVLink通信超时（心跳/指令/速率）
//   Crash:                             碰撞/撞击瞬时检测（IMU加速度尖峰）
//
// 严重等级仲裁规则：
//   LEVEL_1 (降落):  任意 1 个维度触发
//   LEVEL_2 (急降):  任意 2 个维度触发，或 MTO + 1个异常
//   LEVEL_3 (终止):  3 个及以上维度触发，或 电流异常+姿态异常 同时触发
// ============================================================

void FailureDetector::updateLOCP(const vehicle_status_s &vehicle_status,
				 const vehicle_control_mode_s &vehicle_control_mode)
{
	// 未解锁时重置所有 LOCP 状态，不做检测
	if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		_locp_severity = 0;
		_locp_ard_triggered = false;
		_locp_vrd_triggered = false;
		_locp_prd_triggered = false;
		_locp_cod_triggered = false;
		_locp_mto_triggered = false;
		_locp_obs_triggered = false;
		_crash_detected = false;
		_rate_window_start = 0;
		_mavlink_msg_counter = 0;
		_sp_was_valid = false;
		return;
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

	// 4) COD: 电流异常检测（总电流突增/dI/dt尖峰/单路ESC过流）
	battery_status_s bat;
	esc_status_s esc;
	if (_battery_status_sub.update(&bat)) {
		_esc_status_sub.copy(&esc);
		_locp_cod_triggered = _param_locp_cod_en.get() && checkCurrentAnomaly(bat, esc);
	}

	// 5) MTO: MAVLink 消息超时检测（心跳超时/指令超时/消息速率骤降）
	_locp_mto_triggered = _param_locp_mto_en.get() && checkMavlinkTimeout();

	// 6) OBS: Offboard Setpoint 异常检测（数值跳变/NaN注入）
	_locp_obs_triggered = _param_locp_obs_en.get() && checkOffboardSetpointSanity();

	// 7) Crash: 碰撞/撞击瞬时检测（基于 IMU 加速度范数阈值，无迟滞）
	_crash_detected = checkCrashImpact();

	// 综合仲裁：根据各维度触发情况计算最终严重等级
	_locp_severity = evaluateLOCPSeverity();
}

bool FailureDetector::checkAttitudeRateAnomaly(const vehicle_angular_velocity_s &ang_vel)
{
	const hrt_abstime now = hrt_absolute_time();
	const float dt = 0.01f; // 假设更新频率为 100Hz，用于角加速度计算

	// --- 角加速度检测：三轴角加速度分别与阈值比较 ---
	// 通过前后两帧角速度差分计算角加速度 (rad/s²)
	float d_roll  = fabsf(ang_vel.xyz[0] - _rollspeed_prev)  / dt;
	float d_pitch = fabsf(ang_vel.xyz[1] - _pitchspeed_prev) / dt;
	float d_yaw   = fabsf(ang_vel.xyz[2] - _yawspeed_prev)   / dt;

	// 保存当前角速度，供下一帧计算角加速度
	_rollspeed_prev  = ang_vel.xyz[0];
	_pitchspeed_prev = ang_vel.xyz[1];
	_yawspeed_prev   = ang_vel.xyz[2];

	// 角加速度超限判定：任意轴角加速度 > 对应阈值即触发
	bool accel_fault = (d_roll  > _param_locp_ard_r_max.get())
			|| (d_pitch > _param_locp_ard_p_max.get())
			|| (d_yaw   > _param_locp_ard_y_max.get());

	// --- 持续高角速率检测：角速率超过设定值并持续超过设定时间 ---
	// 仅检测 Roll 和 Pitch（Yaw 通常变化范围大，不作为持续判断条件）
	bool sustained_high =
		(fabsf(ang_vel.xyz[0]) > _param_locp_ard_rsp.get()
		 && (now - _att_rate_high_start) > static_cast<hrt_abstime>(_param_locp_ard_dur.get() * 1_s))
		||
		(fabsf(ang_vel.xyz[1]) > _param_locp_ard_psp.get()
		 && (now - _att_rate_high_start) > static_cast<hrt_abstime>(_param_locp_ard_dur.get() * 1_s));

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
	bool horiz_fault = acc_horiz > _param_locp_vrd_ah_max.get();

	// --- 自由落体/动力急降检测 ---
	// 条件1：垂直加速度 az 超出阈值（向下加速）
	// 条件2：垂直速度 vz 已经超过阈值（正在快速下降）
	// 两个条件同时满足才判定为自由落体/急降
	bool freefall = (loc.az > _param_locp_vrd_ad_max.get())
		     && (loc.vz > _param_locp_vrd_vzd_max.get());

	// --- Jerk（加加速度）检测 ---
	// Jerk = 水平加速度的变化率 = |Δa_horiz| / dt
	// 检测加速度的突变（急加速或急减速）
	float dt = 0.01f;
	float jerk = fabsf(acc_horiz - _acc_horiz_prev) / dt;
	bool jerk_fault = jerk > _param_locp_vrd_jerk.get();
	_acc_horiz_prev = acc_horiz;

	bool triggered = horiz_fault || freefall || jerk_fault;

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
	bool rapid_descent = (loc.vz > _param_locp_prd_vz_max.get())
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
	bool alt_oscillation = (_alt_history_count >= 5) && (alt_std > _param_locp_prd_astd.get());

	// --- 水平漂移检测 ---
	// 计算水平面合成速度 (vx² + vy²) 的平方根
	float horiz_spd = sqrtf(loc.vx * loc.vx + loc.vy * loc.vy);
	bool position_drift = horiz_spd > _param_locp_prd_hspd.get();

	bool triggered = rapid_descent || alt_oscillation || position_drift;

	// 迟滞滤波：故障条件需持续 LOCP_PRD_T 秒
	_prd_hysteresis.set_hysteresis_time_from(false,
		static_cast<hrt_abstime>(_param_locp_prd_t.get() * 1_s));
	_prd_hysteresis.set_state_and_update(triggered, now);

	return _prd_hysteresis.get_state();
}

bool FailureDetector::checkCurrentAnomaly(const battery_status_s &bat, const esc_status_s &esc)
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
	bool total_surge = (deviation > _param_locp_cod_delta_i.get())
			|| (bat.current_a > _param_locp_cod_max_i.get());

	// --- 电流变化率 (dI/dt) 尖峰检测 ---
	// 通过前后两帧电流差分计算 dI/dt (A/s)，检测瞬间电流尖峰
	float di_dt = fabsf(bat.current_a - _current_prev) / 0.01f;
	bool current_spike = di_dt > _param_locp_cod_di_dt.get();
	_current_prev = bat.current_a;

	// --- 单路 ESC 过流检测 ---
	// 计算各 ESC 平均电流，检测是否有单路 ESC 电流异常偏高
	int esc_count = esc.esc_count;
	if (esc_count > 8) { esc_count = 8; }   // 最多支持 8 路 ESC
	if (esc_count < 1) { esc_count = 1; }   // 至少按 1 路处理，防止除以零

	float esc_avg = 0.f;
	for (int i = 0; i < esc_count; i++) { esc_avg += esc.esc[i].esc_current; }
	esc_avg /= esc_count;

	// 判定条件：单路电流 > 均值×2 且 > 绝对阈值
	bool single_esc_fault = false;
	for (int i = 0; i < esc_count; i++) {
		if (esc.esc[i].esc_current > esc_avg * 2.0f
		    && esc.esc[i].esc_current > _param_locp_cod_esc_max.get()) {
			single_esc_fault = true;
			break;
		}
	}

	bool triggered = total_surge || current_spike || single_esc_fault;

	// 迟滞滤波：故障条件需持续 LOCP_COD_T 秒
	_cod_hysteresis.set_hysteresis_time_from(false,
		static_cast<hrt_abstime>(_param_locp_cod_t.get() * 1_s));
	_cod_hysteresis.set_state_and_update(triggered, now);

	return _cod_hysteresis.get_state();
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
	bool heartbeat_lost = (_last_mavlink_heartbeat != 0)
		&& (now - _last_mavlink_heartbeat >
		    static_cast<hrt_abstime>(_param_locp_mto_hb_t.get() * 1_s));

	// 指令超时判定：距离最后一次指令超过 LOCP_MTO_CMD_T 秒
	bool cmd_timeout = (_last_vehicle_command != 0)
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

uint8_t FailureDetector::evaluateLOCPSeverity()
{
	// 统计当前触发的检测维度数量
	int count = (_locp_ard_triggered ? 1 : 0)
		    + (_locp_vrd_triggered ? 1 : 0)
		    + (_locp_prd_triggered ? 1 : 0)
		    + (_locp_cod_triggered ? 1 : 0)
		    + (_locp_mto_triggered ? 1 : 0)
		    + (_locp_obs_triggered ? 1 : 0);

	// ============================================================
	// 数据源相关性折扣 (Correlation Discount)
	// ============================================================
	// VRD (速度变化率) 和 PRD (位置变化率) 共用同一个 EKF 估计数据源
	// (vehicle_local_position)。如果 EKF 因 GPS/光流等外部源异常而发散，
	// VRD 和 PRD 会【同时】触发 —— 但这并非真正的双重故障证据，
	// 而是单一 EKF 数据源问题的连锁反应。
	//
	// 因此：当 VRD 和 PRD 同时触发时，按 1 个维度计算（折半），
	// 防止 EKF 源问题被误判为 LEVEL_2/3 并触发激进的 Disarm。
	if (_locp_vrd_triggered && _locp_prd_triggered) {
		count -= 1;
	}

	// 致命组合：电流异常 + 姿态异常同时触发（暗示动力系统严重故障）
	// 这两个维度来自独立的数据源（电池电流 + IMU 角速度），
	// 同时触发是真实的动力系统故障证据，不做折扣。
	bool fatal_combo = _locp_cod_triggered && _locp_ard_triggered;

	// 通信中断且有【其他（非 MTO 自身）】异常：
	// MAVLink 链路死亡 + 至少一种传感器异常 → 升级为 LEVEL_2
	// 注意：这里的 count 已排除 MTO 自身，避免"仅 MTO 触发"被误升为 LEVEL_2
	int count_excl_mto = count - (_locp_mto_triggered ? 1 : 0);
	bool comm_dead = _locp_mto_triggered && (count_excl_mto >= 1);

	// 严重等级仲裁规则：
	// LEVEL_3：3+ 维度触发 或 电流+姿态致命组合 → 立即终止飞行
	// LEVEL_2：2 个维度触发 或 通信中断+其他异常 → 紧急降落
	// LEVEL_1：1 个维度触发                       → 常规降落
	// NONE：   未触发任何维度                     → 正常
	if (count >= 3 || fatal_combo) {
		return 3; // LEVEL_3: 终止飞行（Terminate）
	}
	if (count >= 2 || comm_dead) {
		return 2; // LEVEL_2: 紧急降落（Descend）
	}
	if (count >= 1) {
		return 1; // LEVEL_1: 常规降落（Land）
	}
	return 0; // NONE: 正常飞行
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
		// --- 1) NaN 注入检测：上一帧有效 → 当前帧突变为 NaN ---
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

		// --- 2) 数值跳变检测：相邻帧间变化超过阈值 ---
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
