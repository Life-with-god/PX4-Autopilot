# Input flags for the failsafe state machine set by the arming & health checks.
#
# Flags must be named such that false == no failure (e.g. _invalid, _unhealthy, _lost)
# The flag comments are used as label for the failsafe state machine simulation

uint64 timestamp				# time since system start (microseconds)

# Per-mode requirements
uint32 mode_req_angular_velocity
uint32 mode_req_attitude
uint32 mode_req_local_alt
uint32 mode_req_local_position
uint32 mode_req_local_position_relaxed
uint32 mode_req_global_position
uint32 mode_req_mission
uint32 mode_req_offboard_signal
uint32 mode_req_home_position
uint32 mode_req_wind_and_flight_time_compliance # if set, mode cannot be entered if wind or flight time limit exceeded
uint32 mode_req_prevent_arming    # if set, cannot arm while in this mode
uint32 mode_req_manual_control
uint32 mode_req_other             # other requirements, not covered above (for external modes)


# Mode requirements
bool angular_velocity_invalid         # Angular velocity invalid
bool attitude_invalid                 # Attitude invalid
bool local_altitude_invalid           # Local altitude invalid
bool local_position_invalid           # Local position estimate invalid
bool local_position_invalid_relaxed   # Local position with reduced accuracy requirements invalid (e.g. flying with optical flow)
bool local_velocity_invalid           # Local velocity estimate invalid
bool global_position_invalid          # Global position estimate invalid
bool auto_mission_missing             # No mission available
bool offboard_control_signal_lost     # Offboard signal lost
bool home_position_invalid            # No home position available

# Control links
bool manual_control_signal_lost       # Manual control (RC) signal lost
bool gcs_connection_lost              # GCS connection lost

# Battery
uint8 battery_warning                 # Battery warning level (see BatteryStatus.msg)
bool battery_low_remaining_time       # Low battery based on remaining flight time
bool battery_unhealthy                # Battery unhealthy

# Other
bool geofence_breached        	      # Geofence breached (one or multiple)
bool mission_failure                  # Mission failure
bool vtol_fixed_wing_system_failure   # vehicle in fixed-wing system failure failsafe mode (after quad-chute)
bool wind_limit_exceeded              # Wind limit exceeded
bool flight_time_limit_exceeded       # Maximum flight time exceeded
bool local_position_accuracy_low      # Local position estimate has dropped below threshold, but is currently still declared valid
bool navigator_failure        	      # Navigator failed to execute a mode

# Failure detector
bool fd_critical_failure              # Critical failure (attitude/altitude limit exceeded, or external ATS)
bool fd_esc_arming_failure            # ESC failed to arm
bool fd_imbalanced_prop               # Imbalanced propeller detected
bool fd_motor_failure                 # Motor failure

# ============================================================
# LOCP (Loss-of-Control Protection) —— 失控保护系统标志位
# ============================================================
# 设计背景：在无遥控器/地面站失效的情况下，多维度传感器融合检测失控状态
# 检测维度：ARD(姿态变化率) VRD(速度变化率) PRD(位置变化率) COD(电流异常) MTO(MAVLink超时)
# 严重等级：0=NONE 1=LEVEL_1(降落) 2=LEVEL_2(急降) 3=LEVEL_3(终止)

# --- 各检测维度触发标志 ---
bool locp_ard_triggered               # ARD 姿态变化率异常触发（角加速度过大 或 持续高角速率）
bool locp_vrd_triggered               # VRD 速度变化率异常触发（水平加速度/自由落体/jerk 过大）
bool locp_prd_triggered               # PRD 位置变化率异常触发（急降/高度振荡/水平漂移）
bool locp_cod_triggered               # COD 电流异常触发（总电流突增/dI/dt尖峰/单路ESC过流）
bool locp_mto_triggered               # MTO MAVLink超时触发（心跳丢失 或 指令超时+速率骤降）
bool locp_obs_triggered               # OBS Offboard Setpoint异常触发（setpoint数值跳变/NaN注入）

# --- 综合严重等级 ---
uint8 locp_severity                   # LOCP 综合严重等级 (0=NONE, 1=LEVEL_1, 2=LEVEL_2, 3=LEVEL_3)

# --- 派生等级标志（用于 CHECK_FAILSAFE 宏，自动从 locp_severity 派生） ---
bool locp_level1                      # LOCP 严重等级 >= 1（任意维度触发即为真）
bool locp_level2                      # LOCP 严重等级 >= 2（2个维度 或 通信中断+异常）
bool locp_level3                      # LOCP 严重等级 >= 3（3个维度 或 电流+姿态致命组合）

# --- 碰撞/撞击检测 ---
bool crash_detected                   # 碰撞/撞击瞬时检测触发（IMU加速度范数尖峰 > 阈值, 默认 ~80m/s²）
