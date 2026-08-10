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
 * @file failure_detector_params.c
 *
 * Parameters used by the Failure Detector.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * FailureDetector Max Roll
 *
 * Maximum roll angle before FailureDetector triggers the attitude_failure flag.
 * The flag triggers flight termination (if @CBRK_FLIGHTTERM = 0),
 * which sets outputs to their failsafe values.
 * On takeoff the flag triggers lockdown (irrespective of @CBRK_FLIGHTTERM),
 * which disarms motors but does not set outputs to failsafe values.
 *
 * Setting this parameter to 0 disables the check
 *
 * @min 0
 * @max 180
 * @unit deg
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_FAIL_R, 60);

/**
 * FailureDetector Max Pitch
 *
 * Maximum pitch angle before FailureDetector triggers the attitude_failure flag.
 * The flag triggers flight termination (if @CBRK_FLIGHTTERM = 0),
 * which sets outputs to their failsafe values.
 * On takeoff the flag triggers lockdown (irrespective of @CBRK_FLIGHTTERM),
 * which disarms motors but does not set outputs to failsafe values.
 *
 * Setting this parameter to 0 disables the check
 *
 * @min 0
 * @max 180
 * @unit deg
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_FAIL_P, 60);

/**
 * Roll failure trigger time
 *
 * Seconds (decimal) that roll has to exceed FD_FAIL_R before being considered as a failure.
 *
 * @unit s
 * @min 0.02
 * @max 5
 * @decimal 2
 *
 * @group Failure Detector
 */
PARAM_DEFINE_FLOAT(FD_FAIL_R_TTRI, 0.3);

/**
 * Pitch failure trigger time
 *
 * Seconds (decimal) that pitch has to exceed FD_FAIL_P before being considered as a failure.
 *
 * @unit s
 * @min 0.02
 * @max 5
 * @decimal 2
 *
 * @group Failure Detector
 */
PARAM_DEFINE_FLOAT(FD_FAIL_P_TTRI, 0.3);

/**
 * Enable PWM input on for engaging failsafe from an external automatic trigger system (ATS).
 *
 * Enabled on either AUX5 or MAIN5 depending on board.
 * External ATS is required by ASTM F3322-18.
 *
 * @boolean
 * @reboot_required true
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_EXT_ATS_EN, 0);

/**
 * The PWM threshold from external automatic trigger system for engaging failsafe.
 *
 * External ATS is required by ASTM F3322-18.
 *
 * @unit us
 * @decimal 2
 *
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_EXT_ATS_TRIG, 1900);

/**
 * Enable checks on ESCs that report their arming state.
 *
 * If enabled, failure detector will verify that all the ESCs have successfully armed when the vehicle has transitioned to the armed state.
 * Timeout for receiving an acknowledgement from the ESCs is 0.3s, if no feedback is received the failure detector will auto disarm the vehicle.
 *
 * @boolean
 *
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_ESCS_EN, 1);

/**
 * Imbalanced propeller check threshold
 *
 * Value at which the imbalanced propeller metric (based on horizontal and
 * vertical acceleration variance) triggers a failure
 *
 * Setting this value to 0 disables the feature.
 *
 * @min 0
 * @max 1000
 * @increment 1
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_IMB_PROP_THR, 30);

/**
 * Enable Actuator Failure check
 *
 * If enabled, failure detector will verify that for motors, a minimum amount of ESC current per throttle
 * level is being consumed.
 * Otherwise this indicates an motor failure.
 *
 * @boolean
 * @reboot_required true
 *
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_ACT_EN, 1);

/**
 * Motor Failure Throttle Threshold
 *
 * Motor failure triggers only above this throttle value.
 *
 * @group Failure Detector
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FD_ACT_MOT_THR, 0.2f);

/**
 * Motor Failure Current/Throttle Threshold
 *
 * Motor failure triggers only below this current value
 *
 * @group Failure Detector
 * @min 0.0
 * @max 50.0
 * @unit A/%
 * @decimal 2
 * @increment 1
 */
PARAM_DEFINE_FLOAT(FD_ACT_MOT_C2T, 2.0f);

/**
 * Motor Failure Time Threshold
 *
 * Motor failure triggers only if the throttle threshold and the
 * current to throttle threshold are violated for this time.
 *
 * @group Failure Detector
 * @unit ms
 * @min 10
 * @max 10000
 * @increment 100
 */
PARAM_DEFINE_INT32(FD_ACT_MOT_TOUT, 100);

// ============================================================
// LOCP (Loss-of-Control Protection) —— 失控保护系统参数
// ============================================================
// 这些参数控制 LOCP 多维度失控检测的阈值和行为。
// 检测维度：ARD(姿态变化率) VRD(速度变化率) PRD(位置变化率) COD(电流异常) MTO(MAVLink超时)
// 严重等级：0=NONE 1=LEVEL_1(降落) 2=LEVEL_2(急降) 3=LEVEL_3(终止)
// ============================================================

// ============================================================
// ARD (Attitude Rate Detection) —— 姿态变化率检测
// ============================================================

/**
 * LOCP ARD 使能开关
 *
 * 启用姿态变化率异常检测。检测过大的角加速度或持续高角速率，
 * 作为飞行器失控的标志。
 *
 * @boolean
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_ARD_EN, 1);

/**
 * LOCP Roll 角加速度最大阈值
 *
 * Roll 轴角加速度 (d²roll/dt²) 超过此值触发 ARD 检测。
 *
 * @min 20
 * @max 500
 * @unit rad/s^2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_R_MAX, 80.0f);

/**
 * LOCP Pitch 角加速度最大阈值
 *
 * Pitch 轴角加速度 (d²pitch/dt²) 超过此值触发 ARD 检测。
 *
 * @min 20
 * @max 500
 * @unit rad/s^2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_P_MAX, 80.0f);

/**
 * LOCP Yaw 角加速度最大阈值
 *
 * Yaw 轴角加速度超过此值触发 ARD 检测。
 *
 * @min 20
 * @max 500
 * @unit rad/s^2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_Y_MAX, 60.0f);

/**
 * LOCP ARD 迟滞确认时间
 *
 * 角加速度故障条件必须持续超过该时长（秒）才确认触发 ARD，
 * 防止传感器瞬时噪声导致误判。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_T, 0.3f);

/**
 * LOCP Roll 持续高角速率设定点
 *
 * Roll 轴角速率持续超过此值并达到持续时间阈值后触发 ARD。
 *
 * @min 3
 * @max 30
 * @unit rad/s
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_RSP, 6.0f);

/**
 * LOCP Pitch 持续高角速率设定点
 *
 * Pitch 轴角速率持续超过此值并达到持续时间阈值后触发 ARD。
 *
 * @min 3
 * @max 30
 * @unit rad/s
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_PSP, 6.0f);

/**
 * LOCP Yaw 持续高角速率设定点
 *
 * Yaw 轴角速率持续超过此值并达到持续时间阈值后触发 ARD。
 *
 * @min 3
 * @max 30
 * @unit rad/s
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_YSP, 5.0f);

/**
 * LOCP 持续高角速率最小持续时间
 *
 * 角速率必须持续高于设定点的最短时长（秒），超过后才触发 ARD。
 * 用于过滤短暂的角速率尖峰。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_DUR, 0.3f);

// ============================================================
// VRD (Velocity Rate Detection) —— 速度变化率检测
// ============================================================

/**
 * LOCP VRD 使能开关
 *
 * 启用速度变化率异常检测。检测过大的水平加速度、垂直加速度
 * 或 jerk（加加速度），作为飞行器失控的标志。
 *
 * @boolean
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_VRD_EN, 1);

/**
 * LOCP 水平加速度最大阈值
 *
 * 水平面合成加速度超过此值触发 VRD 检测。
 *
 * @min 5
 * @max 50
 * @unit m/s^2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_AH_MAX, 8.0f);

/**
 * LOCP 垂直向下加速度阈值
 *
 * 垂直向下加速度（NED 坐标系 Z 轴正向）超过此值触发 VRD 检测。
 * 用于检测自由落体或动力急降。
 *
 * @min 3
 * @max 20
 * @unit m/s^2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_AD_MAX, 6.0f);

/**
 * LOCP 垂直下降速度阈值
 *
 * 垂直下降速度（NED 坐标系 VZ 正向）超过此值触发 VRD 检测。
 * 与 LOCP_VRD_AD_MAX 配合使用，两者同时满足才判定为急降。
 *
 * @min 1
 * @max 15
 * @unit m/s
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_VZD_MAX, 3.0f);

/**
 * LOCP Jerk（加加速度）阈值
 *
 * 水平加速度的变化率（jerk = da/dt）超过此值触发 VRD 检测。
 * 用于检测加速度的突变（急加速或急减速）。
 *
 * @min 10
 * @max 200
 * @unit m/s^3
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_JERK, 50.0f);

/**
 * LOCP VRD 迟滞确认时间
 *
 * VRD 故障条件必须持续超过该时长（秒）才确认触发。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_T, 0.3f);

// ============================================================
// PRD (Position Rate Detection) —— 位置变化率检测
// ============================================================

/**
 * LOCP PRD 使能开关
 *
 * 启用位置变化率异常检测。检测急降、高度振荡或水平漂移，
 * 作为飞行器失控的标志。
 *
 * @boolean
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_PRD_EN, 1);

/**
 * LOCP 下降速度阈值
 *
 * 垂直下降速度（NED 坐标系 VZ 正向）超过此值触发 PRD 急降检测。
 *
 * @min 1
 * @max 20
 * @unit m/s
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_PRD_VZ_MAX, 5.0f);

/**
 * LOCP 高度下降量阈值
 *
 * 相对于 Home 点的累计高度下降量超过此值触发 PRD 急降检测。
 * 与 LOCP_PRD_VZ_MAX 配合使用，两者同时满足才判定为急降。
 *
 * @min 1
 * @max 50
 * @unit m
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_PRD_ADROP, 3.0f);

/**
 * LOCP 高度振荡标准差阈值
 *
 * 1 秒滑动窗口内高度数据的标准差超过此值触发 PRD 振荡检测。
 * 用于检测高度控制回路失稳导致的剧烈波动。
 *
 * @min 0.5
 * @max 10
 * @unit m
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_PRD_ASTD, 2.0f);

/**
 * LOCP 水平漂移速度阈值
 *
 * 水平面合成速度超过此值触发 PRD 漂移检测。
 * 用于检测无位置控制时飞行器的意外水平漂移。
 *
 * @min 1
 * @max 30
 * @unit m/s
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_PRD_HSPD, 5.0f);

/**
 * LOCP PRD 迟滞确认时间
 *
 * PRD 故障条件必须持续超过该时长（秒）才确认触发。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_PRD_T, 0.5f);

// ============================================================
// COD (Current Overdraw Detection) —— 电流异常检测
// ============================================================

/**
 * LOCP COD 使能开关
 *
 * 启用电流异常检测。检测总电流突变、dI/dt 尖峰以及单路 ESC 过流，
 * 作为电机/电调故障的标志。
 *
 * @boolean
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_COD_EN, 1);

/**
 * LOCP 总电流偏离滑动均值阈值
 *
 * 当前总电流偏离 20 帧滑动窗口均值的幅度超过此值触发 COD 检测。
 * 用于检测总电流的突然增大。
 *
 * @min 5
 * @max 40
 * @unit A
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_DELTA_I, 10.0f);

/**
 * LOCP 总电流绝对最大阈值
 *
 * 总电流绝对值超过此值直接触发 COD 检测，与滑动均值无关。
 *
 * @min 10
 * @max 150
 * @unit A
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_MAX_I, 45.0f);

/**
 * LOCP 电流变化率 (dI/dt) 阈值
 *
 * 电流变化率超过此值触发 COD 检测。
 * 用于检测瞬间电流尖峰（如堵转）。
 *
 * @min 10
 * @max 200
 * @increment 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_DI_DT, 30.0f);

/**
 * LOCP 单路 ESC 电流最大阈值
 *
 * 单路 ESC 电流超过此值且大于各 ESC 平均值的 2 倍时触发 COD 检测。
 * 用于检测单路 ESC/电机异常（如短路、堵转）。
 *
 * @min 5
 * @max 50
 * @unit A
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_ESC_MAX, 10.0f);

/**
 * LOCP COD 迟滞确认时间
 *
 * COD 故障条件必须持续超过该时长（秒）才确认触发。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_T, 0.3f);

/**
 * LOCP COD 解锁启动保护延迟
 *
 * 解锁后该时长（秒）内不执行 COD 电流异常检测。
 * 电机启动瞬间电流从 0 爬升到悬停电流是正常行为，若不屏蔽会被误判为
 * 电流异常（dI/dt 尖峰/电流突增）而触发保护动作（如解锁后立即上锁）。
 * 建议保持默认值，待电流滑动窗口建立后再开始检测。
 *
 * @min 0.0
 * @max 10.0
 * @unit s
 * @decimal 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_ARM_DELAY, 2.0f);

// ============================================================
// MTO (MAVLink Timeout) —— MAVLink 消息超时检测
// ============================================================

/**
 * LOCP MTO 使能开关
 *
 * 启用 MAVLink 消息超时检测。检测心跳丢失、指令超时以及
 * 消息速率骤降，在无遥控器的情况下尤为关键。
 *
 * @boolean
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_MTO_EN, 1);

/**
 * LOCP MAVLink 心跳超时阈值
 *
 * 距离最后一次收到 MAVLink 心跳的最大允许时间（秒），
 * 超过此值触发 MTO 心跳丢失检测。
 *
 * @min 0.5
 * @max 10
 * @unit s
 * @decimal 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_MTO_HB_T, 1.5f);

/**
 * LOCP MAVLink 指令超时阈值
 *
 * 距离最后一次收到车辆指令消息的最大允许时间（秒），
 * 超过此值且消息速率同时下降时触发 MTO 指令超时检测。
 *
 * @min 0.5
 * @max 10
 * @unit s
 * @decimal 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_MTO_CMD_T, 2.0f);

/**
 * LOCP MAVLink 消息最低速率
 *
 * 1 秒滑动窗口内 MAVLink 消息的最低接收速率 (Hz)，
 * 低于此值且同时存在指令超时时触发 MTO 检测。
 *
 * @min 0.5
 * @max 50
 * @unit Hz
 * @decimal 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_MTO_RATE, 3.0f);

// ============================================================
// Crash/Impact Detection —— 碰撞/撞击检测
// ============================================================

// ============================================================
// OBS (Offboard Setpoint Sanity) —— 机载计算机Setpoint异常检测
// ============================================================
// 在 Offboard 模式下，MAVROS/机载计算机发送的 trajectory_setpoint
// 可能因为软件 bug 出现数值跳变(Spike)或 NaN 注入。OBS 检测这些
// 异常 setpoint，防止飞行器因错误指令而失控。
// 注意：OBS 不检测心跳超时（已有 offboard_control_signal_lost），
// 不检测软件卡死或控制周期变化，仅检测 setpoint 数值本身的合法性。

/**
 * LOCP OBS 使能开关
 *
 * 启用 Offboard Setpoint 异常检测。检测 MAVROS/机载计算机发送的
 * trajectory_setpoint 是否存在数值跳变或 NaN 注入。
 *
 * @boolean
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_OBS_EN, 1);

/**
 * LOCP OBS 位置跳变阈值
 *
 * trajectory_setpoint 中相邻两帧位置 setpoint 的最大允许变化量。
 * 任一轴（X/Y/Z）的变化量超过此值触发 OBS 跳变检测。
 *
 * @min 1
 * @max 50
 * @unit m
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_OBS_J_POS, 10.0f);

/**
 * LOCP OBS 速度跳变阈值
 *
 * trajectory_setpoint 中相邻两帧速度 setpoint 的最大允许变化量。
 * 任一轴（X/Y/Z）的变化量超过此值触发 OBS 跳变检测。
 *
 * @min 1
 * @max 20
 * @unit m/s
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_OBS_J_VEL, 5.0f);

/**
 * LOCP OBS Yaw 跳变阈值
 *
 * trajectory_setpoint 中相邻两帧偏航角 setpoint 的最大允许变化量（弧度）。
 * 自动处理 ±PI 环绕。
 *
 * @min 0.5
 * @max 6.28
 * @unit rad
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_OBS_J_YAW, 1.57f);

/**
 * LOCP OBS 迟滞确认时间
 *
 * OBS 故障条件必须持续超过该时长（秒）才确认触发。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_OBS_T, 0.3f);

/**
 * LOCP 碰撞加速度阈值
 *
 * IMU 加速度范数（三轴合成加速度幅值）的瞬时阈值。
 * 超过此值立即触发碰撞检测，无需迟滞确认，直接执行上锁动作。
 *
 * @min 30
 * @max 200
 * @unit m/s^2
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_CRASH_THR, 50.0f);

// ============================================================
// LOCP 严重等级触发动作配置
// ============================================================

/**
 * LOCP 等级 1 动作（轻度异常 → 默认降落）
 *
 * 单个检测维度触发时的保护动作（MAVLink 通信正常的前提下）。
 * 0=无动作 1=警告 2=悬停 3=降落 4=急降 5=返航 6=终止 7=上锁
 *
 * @min 0
 * @max 7
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_L1_ACT, 3);

/**
 * LOCP 等级 2 动作（中度异常 → 默认降落）
 *
 * 2 个及以上检测维度触发，或 MAVLink 通信中断 + 1 个异常时的保护动作。
 * 注意：VRD 和 PRD 共用 EKF 数据源，同时触发时按 1 维计算，
 * 因此 LEVEL_2 往往意味着真实的双重故障证据。
 * 0=无动作 1=警告 2=悬停 3=降落 4=急降 5=返航 6=终止 7=上锁
 *
 * @min 0
 * @max 7
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_L2_ACT, 3);

/**
 * LOCP 等级 3 动作（严重异常 → 默认上锁）
 *
 * 3 个及以上检测维度触发，或电流异常+姿态异常致命组合时的保护动作。
 * 0=无动作 1=警告 2=悬停 3=降落 4=急降 5=返航 6=终止 7=上锁
 *
 * @min 0
 * @max 7
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_L3_ACT, 7);

/**
 * LOCP OBS 触发动作（Offboard Setpoint 异常 → 默认降落）
 *
 * OBS 检测到机载计算机发送的 setpoint 数值跳变/NaN 注入时执行的动作。
 * 与 LOCP_L2_ACT 独立配置，避免 setpoint 单次异常触发过于激进的 Disarm。
 * 0=无动作 1=警告 2=悬停 3=降落 4=急降 5=返航 6=终止 7=上锁
 *
 * @min 0
 * @max 7
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_OBS_ACT, 3);
