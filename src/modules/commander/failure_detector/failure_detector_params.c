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
// 检测维度：ARD(姿态变化率) VRD(速度变化率) PRD(位置变化率) COD(电流异常)
//          MTO(MAVLink超时) TRD(动力响应校验) OBS(Offboard异常) Crash(碰撞)
// 动作硬编码（无等级参数）：失控维度→停桨；MTO/OBS→健康检查通过才降落
// ============================================================

/**
 * LOCP 失控保护总开关
 *
 * 一键禁用/启用整个 LOCP 失控保护系统。
 * 设置为 0 时，所有 LOCP 检测维度（ARD/VRD/PRD/COD/MTO/OBS/TRD/Crash）
 * 全部禁用，不触发任何保护动作。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_EN, 0);

// ============================================================
// ARD (Attitude Rate Detection) —— 姿态变化率检测
// ============================================================

/**
 * LOCP ARD 使能开关
 *
 * 启用姿态变化率异常检测。检测过大的角加速度或持续高角速率，
 * 作为飞行器失控的标志。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_ARD_EN, 0);

/**
 * LOCP ARD 角加速度检测使能开关
 *
 * 启用角加速度尖峰检测（d²roll/pitch/yaw/dt² 超阈值，LOCP_ARD_R_MAX/P_MAX/Y_MAX）。
 * 与持续高角速率检测独立开关，可单独禁用。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_ARD_ACC_EN, 0);

/**
 * LOCP ARD 持续高角速率检测使能开关
 *
 * 启用持续高角速率检测（角速率超 LOCP_ARD_RSP/PSP 并持续 LOCP_ARD_DUR）。
 * 与角加速度检测独立开关，可单独禁用。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_ARD_RATE_EN, 0);

/**
 * LOCP Roll 角加速度最大阈值
 *
 * Roll 轴角加速度 (d²roll/dt²) 超过此值触发 ARD 检测。
 * 标定（2026-08-13，6-8 月 136 份纯净日志，已排除失控/EGO故障/地面假速度）：
 * R 轴角加速度 P99.9=35.5、单帧 Max=285 rad/s²；连续超 100 最长仅 0.042s
 * （log_351）< ARD_T 迟滞 0.1s。失控日志（卡网）峰值 472~855 rad/s²。
 * 取 100 rad/s²（裕度约 2.8 倍，配合 ARD_T 迟滞过滤单帧噪声）。
 *
 * @min 20
 * @max 500
 * @unit rad/s^2
 * @decimal 0
 * @increment 5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_R_MAX, 100.0f);

/**
 * LOCP Pitch 角加速度最大阈值
 *
 * Pitch 轴角加速度 (d²pitch/dt²) 超过此值触发 ARD 检测。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：P 轴角加速度 P99.9=31.5、
 * 单帧 Max=457.6 rad/s²；连续超 120 最长仅 0.038s（log_331）< ARD_T 0.1s。
 * 失控日志（卡网）峰值 276~855 rad/s²。
 * 取 120 rad/s²（裕度约 3.8 倍）。
 *
 * @min 20
 * @max 500
 * @unit rad/s^2
 * @decimal 0
 * @increment 5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_P_MAX, 120.0f);

/**
 * LOCP Yaw 角加速度最大阈值
 *
 * Yaw 轴角加速度超过此值触发 ARD 检测。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：Y 轴角加速度 P99.9=11.8、
 * 单帧 Max=238.6 rad/s²；连续超 60 最长仅 0.042s（log_259）< ARD_T 0.1s。
 * 取 60 rad/s²（裕度约 5.1 倍）。
 *
 * @min 20
 * @max 500
 * @unit rad/s^2
 * @decimal 0
 * @increment 5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_Y_MAX, 60.0f);

/**
 * LOCP ARD 迟滞确认时间
 *
 * 故障条件（角加速度/持续高角速率）必须持续超过该时长（秒）才确认触发 ARD，
 * 防止传感器瞬时噪声导致误判。
 * 2026-08-13 复测（6-8 月 136 份纯净日志，已排除失控/EGO故障/地面假速度）：
 * 正常飞行角加速度连续超 100 rad/s² 最长段仅 0.042s（log_351），
 * 卡网事故 log_309 0.221s、log_311 0.082s。
 * 取 0.1s：正常裕度 2.4 倍，事故余量 2.2 倍。
 * （注：早期 1-2 月日志的 0.08~0.17s 连续段来自地面翻倒事件——倾角~180°、
 * 高度~0m、|ω|>8 rad/s，属失控样本，不参与标定。）
 *
 * @min 0.01
 * @max 2.0
 * @unit s
 * @decimal 2
 * @increment 0.05
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_T, 0.1f);

/**
 * LOCP Roll 持续高角速率设定点（低阈值·温和乱飞）
 *
 * Roll 轴角速率持续超过此值并达到持续时间阈值后触发 ARD。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：R/P 角速率 P99.9≤1.13 rad/s，
 * 无持续超 2 rad/s 段；异常日志（卡网）连续超阈值 0.26~0.96s（log_309/233）。
 * 取 2 rad/s（裕度约 1.8 倍），配合 LOCP_ARD_DUR=0.2s 抓取异常、过滤正常。
 *
 * @min 1
 * @max 30
 * @unit rad/s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_RSP, 2.0f);

/**
 * LOCP Pitch 持续高角速率设定点（低阈值·温和乱飞）
 *
 * Pitch 轴角速率持续超过此值并达到持续时间阈值后触发 ARD。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：与 LOCP_ARD_RSP 同理
 * （P 轴角速率 P99.9=1.13 rad/s），取 2 rad/s（裕度约 1.8 倍）。
 *
 * @min 1
 * @max 30
 * @unit rad/s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_PSP, 2.0f);

/**
 * LOCP Yaw 持续高角速率设定点
 *
 * Yaw 轴角速率持续超过此值并达到持续时间阈值后触发 ARD。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：Yaw 角速率 P99.9=1.38 rad/s
 * （正常旋转可达 3.14 rad/s）→ 取 5 rad/s（裕度约 3.6 倍）。
 *
 * @min 3
 * @max 30
 * @unit rad/s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_YSP, 5.0f);

/**
 * LOCP 持续高角速率最小持续时间
 *
 * 角速率必须持续高于设定点的最短时长（秒），超过后才触发 ARD。
 * 与低阈值（LOCP_ARD_RSP/PSP=2 rad/s）配合实现温和乱飞检测。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：正常日志无持续超 2 rad/s 段；
 * 异常日志（卡网）连续超阈值 0.26s（log_309）/0.96s（log_233）→ 取 0.2s
 * （配合 ARD_T 共 0.25s 确认，抓取连续 0.26s+ 的异常，过滤正常短尖峰）。
 *
 * @min 0.05
 * @max 3.0
 * @unit s
 * @decimal 2
 * @increment 0.05
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_ARD_DUR, 0.2f);

// ============================================================
// VRD (Velocity Rate Detection) —— 速度变化率检测
// ============================================================

/**
 * LOCP VRD 使能开关
 *
 * 启用速度变化率异常检测。检测过大的水平加速度、垂直加速度
 * 或 jerk（加加速度），作为飞行器失控的标志。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_VRD_EN, 0);

/**
 * LOCP VRD 水平加速度检测使能开关
 *
 * 启用水平面合成加速度检测（LOCP_VRD_AH_MAX）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_VRD_AH_EN, 0);

/**
 * LOCP VRD 自由落体/急降检测使能开关
 *
 * 启用自由落体检测（垂直加速度 LOCP_VRD_AD_MAX + 垂直速度 LOCP_VRD_VZD_MAX）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_VRD_FF_EN, 0);

/**
 * LOCP VRD Jerk（加加速度）检测使能开关
 *
 * 启用水平加速度变化率（jerk）检测（LOCP_VRD_JERK）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_VRD_JK_EN, 0);

/**
 * LOCP VRD 水平速度持续检测使能开关
 *
 * 启用水平速度持续检测（LOCP_VRD_HS_MAX 持续 LOCP_VRD_HS_DUR），
 * 用于检测飞控发疯乱飞 / EKF 发散。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_VRD_HS_EN, 0);

/**
 * LOCP 水平加速度最大阈值
 *
 * 水平面合成加速度超过此值触发 VRD 检测。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：P99.9=4.95 m/s²，
 * 最大 36.5 m/s²（log_295 激进飞行）→ 取 55 m/s²（裕度约 11 倍）。
 *
 * @min 5
 * @max 100
 * @unit m/s^2
 * @decimal 0
 * @increment 5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_AH_MAX, 55.0f);

/**
 * LOCP 垂直向下加速度阈值
 *
 * 垂直向下加速度（NED 坐标系 Z 轴正向）超过此值触发 VRD 检测。
 * 用于检测自由落体或动力急降。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：az P99.9=4.63 m/s²；
 * az>6 且 vz>3 同时满足的全量交叉样本仅 1 个（复合条件保护）。
 * 取 6 m/s²（单维度裕度约 1.3 倍，由双条件复合判定兜底）。
 *
 * @min 3
 * @max 20
 * @unit m/s^2
 * @decimal 1
 * @increment 0.5
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
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_VZD_MAX, 3.0f);

/**
 * LOCP Jerk（加加速度）阈值
 *
 * 水平加速度的变化率（jerk = da/dt）超过此值触发 VRD 检测。
 * 用于检测加速度的突变（急加速或急减速）。
 * 标定（2026-08-13，6-8 月 136 份纯净日志，排除事故/翻倒/EGO故障/地面假速度）：
 * 正常飞行 jerk P99.9=51.1 m/s³；连续超 50 的最长段 297ms 出现在
 * log_192@634s 地面翻倒事件（高度≈0、pitch 7.6 rad/s、事件后 disarmed，
 * 属失控样本）；排除后正常空中最长段 198ms（log_343），均 < 迟滞 0.3s。
 * 取 100 m/s³（约 2.0×P99.9）兼顾裕度与灵敏度。
 *
 * @min 20
 * @max 400
 * @unit m/s^3
 * @decimal 0
 * @increment 10
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_JERK, 100.0f);

/**
 * LOCP VRD 迟滞确认时间
 *
 * VRD 故障条件必须持续超过该时长（秒）才确认触发。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @increment 0.05
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_T, 0.3f);

/**
 * LOCP 水平速度持续阈值（温和乱飞）
 *
 * 水平面合成速度超过此值并持续达到 LOCP_VRD_HS_DUR 时长后触发 VRD，
 * 用于检测飞控发疯乱飞 / EKF 发散导致的意外漂移。
 * 标定（2026-08-13，6-8 月 136 份纯净日志，已排除 5 份地面假速度日志
 * log_215~219）：正常飞行水平速度 P99.9=3.19 m/s；本机理论最大速度
 * 10 m/s（正常飞行可达），阈值取 15 m/s（裕度约 4.7 倍，满速也不触发）。
 * EGO 视觉故障（7-8 月 13 份同源日志）速度 47~107 m/s 持续 3s+，全部仍能被检测到。
 * 卡网场景水平速度较低（log_311 最大 3.9 m/s），由 TRD 动力响应校验兜底。
 * 注：log_215~219（7-22/23）为地面 arm 调试，高度 0.3~0.4m 未起飞，
 * EGO 假速度 18~22 m/s——若启用本通道会在这些地面日志触发，
 * 但飞机本来就在地面，停桨无害。
 *
 * @min 1
 * @max 30
 * @unit m/s
 * @decimal 1
 * @increment 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_HS_MAX, 15.0f);

/**
 * LOCP 水平速度持续时长
 *
 * 水平速度持续超过 LOCP_VRD_HS_MAX（15 m/s）的最短时长（秒），超过后触发 VRD。
 * 与低阈值配合，过滤正常飞行的短时加速。
 * EGO 视觉故障速度发散持续 9~330s，远超过 3s，可稳定捕获。
 *
 * @min 0.5
 * @max 5.0
 * @unit s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_VRD_HS_DUR, 3.0f);

// ============================================================
// PRD (Position Rate Detection) —— 位置变化率检测
// ============================================================

/**
 * LOCP PRD 使能开关
 *
 * 启用位置变化率异常检测。检测急降、高度振荡或水平漂移，
 * 作为飞行器失控的标志。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_PRD_EN, 0);

/**
 * LOCP PRD 急降检测使能开关
 *
 * 启用急降检测（垂直速度 LOCP_PRD_VZ_MAX + 高度下降量 LOCP_PRD_ADROP）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_PRD_DES_EN, 0);

/**
 * LOCP PRD 高度振荡检测使能开关
 *
 * 启用高度振荡检测（1 秒滑动窗口标准差 LOCP_PRD_ASTD）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_PRD_OSC_EN, 0);

/**
 * LOCP PRD 水平漂移检测使能开关
 *
 * 启用水平漂移检测（水平速度 LOCP_PRD_HSPD）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_PRD_DRF_EN, 0);

/**
 * LOCP 下降速度阈值
 *
 * 垂直下降速度（NED 坐标系 VZ 正向）超过此值触发 PRD 急降检测。
 *
 * @min 1
 * @max 20
 * @unit m/s
 * @decimal 1
 * @increment 0.5
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
 * @decimal 1
 * @increment 0.5
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
 * @decimal 1
 * @increment 0.5
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
 * @decimal 1
 * @increment 0.5
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
 * @increment 0.05
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_PRD_T, 0.5f);

// ============================================================
// COD (Current Overdraw Detection) —— 电流异常检测
// ============================================================

/**
 * LOCP COD 使能开关
 *
 * 启用电流异常检测。检测总电流突变、dI/dt 尖峰，
 * 作为电机/电调故障的标志。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_COD_EN, 0);

/**
 * LOCP COD 电流突增检测使能开关
 *
 * 启用总电流突增检测（滑动均值偏差 LOCP_COD_DELTA_I / 绝对阈值 LOCP_COD_MAX_I）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_COD_SRG_EN, 0);

/**
 * LOCP COD 电流变化率(dI/dt)尖峰检测使能开关
 *
 * 启用 dI/dt 尖峰检测（LOCP_COD_DI_DT）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_COD_SPK_EN, 0);

/**
 * LOCP 总电流偏离滑动均值阈值
 *
 * 当前总电流偏离 20 帧滑动窗口均值的幅度超过此值触发 COD 检测。
 * 用于检测总电流的突然增大。
 *
 * @min 5
 * @max 40
 * @unit A
 * @decimal 1
 * @increment 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_DELTA_I, 10.0f);

/**
 * LOCP 总电流绝对最大阈值
 *
 * 总电流绝对值超过此值直接触发 COD 检测，与滑动均值无关。
 *
 * @min 10
 * @max 200
 * @unit A
 * @decimal 0
 * @increment 5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_MAX_I, 100.0f);

/**
 * LOCP 电流变化率 (dI/dt) 阈值
 *
 * 电流变化率超过此值触发 COD 检测。
 * 用于检测瞬间电流尖峰（如堵转）。
 *
 * @min 50
 * @max 1000
 * @increment 10
 * @decimal 0
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_DI_DT, 300.0f);

/**
 * LOCP COD 迟滞确认时间
 *
 * COD 故障条件必须持续超过该时长（秒）才确认触发。
 *
 * @min 0.1
 * @max 2.0
 * @unit s
 * @decimal 2
 * @increment 0.05
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_T, 0.5f);

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
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_COD_ARM_DLY, 2.0f);

// ============================================================
// MTO (MAVLink Timeout) —— MAVLink 消息超时检测
// ============================================================

/**
 * LOCP MTO 使能开关
 *
 * 启用 MAVLink 消息超时检测。检测心跳丢失、指令超时以及
 * 消息速率骤降，在无遥控器的情况下尤为关键。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_MTO_EN, 0);

/**
 * LOCP MTO 心跳超时检测使能开关
 *
 * 启用 MAVLink 心跳丢失检测（LOCP_MTO_HB_T）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_MTO_HB_EN, 0);

/**
 * LOCP MTO 指令超时检测使能开关
 *
 * 启用指令超时检测（LOCP_MTO_CMD_T + 消息速率骤降 LOCP_MTO_RATE）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_MTO_CMD_EN, 0);

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
 * @increment 0.1
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
 * @increment 0.1
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
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_MTO_RATE, 3.0f);

// ============================================================
// TRD (Thrust Response Detection) —— 动力响应校验
// ============================================================

/**
 * LOCP TRD 使能开关
 *
 * 启用动力响应校验检测。检测"高油门但垂直加速度不足"
 * （卡网/动力丢失/桨损坏/控制失效）。
 * 设置为 0 可禁用整个 TRD 检测（含分级动作与接管检查）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_TRD_EN, 0);

/**
 * LOCP TRD 高油门判定阈值
 *
 * 油门指令（0~1）超过此值视为高油门输出。
 * 配合垂直加速度校验检测"高油门但无响应"（卡网/动力丢失/桨损坏）。
 *
 * @min 0.5
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @unit norm
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_THR_H, 0.85f);

/**
 * LOCP TRD 高油门无响应确认时间
 *
 * "高油门但垂直加速度不足"需持续超过该时长（秒）才确认触发，
 * 防止起飞瞬间误报。
 *
 * @min 1.0
 * @max 5.0
 * @unit s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_T, 3.0f);

/**
 * LOCP TRD 垂直加速度下限
 *
 * 垂直加速度（EKF 去重力，NED 向上为负）未达到 -该值（无向上加速）
 * 时视为"动力无响应"。正常满油门爬升 az 约 -30 m/s²，被卡时 az≈0。
 *
 * @min 1
 * @max 15
 * @unit m/s^2
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_AZ_MIN, 5.0f);

/**
 * LOCP TRD 分级动作：降落高度阈值
 *
 * 动力无响应触发时，高度 ≤ 该值（米）先尝试柔和降落（损伤更小），
 * 超过该值或已被抛飞则直接停桨。
 *
 * @min 1
 * @max 20
 * @unit m
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_LAND_H, 5.0f);

/**
 * LOCP TRD 降落尝试超时
 *
 * 低高度降落尝试超过该时长（秒）仍未落地（被网吊住）则转停桨兜底，
 * 防止电机持续过载。
 *
 * @min 2
 * @max 10
 * @unit s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_LTOUT, 4.0f);

/**
 * LOCP TRD 接管观察窗口
 *
 * TRD 触发后，若有人可接管（RC 在线 或 QGC 有指令），观察该时长（秒）：
 * 期间用户接管成功（动力恢复正常）→ 放行；
 * 观察窗口到期仍"高油门无响应" → 判定接管无效（真失控）→ 强制停桨 Disarm。
 * 无人可接管（RC失联 且 QGC 无指令）时跳过观察，立即停桨。
 *
 * @min 1
 * @max 6
 * @unit s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_WATCH, 3.0f);

/**
 * LOCP TRD 垂直速度上限（无显著升降判定）
 *
 * 卡网/动力丢失时飞机挂住不动，|vz|≈0；正常快速升降时 |vz| 超过此值，
 * 说明动力正常（有升降响应）。
 * 标定（2026-08-13）：卡网 log_309/311 段 vz 均值 0.00~0.02 m/s；
 * 正常快速下降 log_351 vz=2.14 m/s；6-8 月 136 份纯净日志 |vz| P99.9=1.04 m/s
 * （单维度裕度约 1.4 倍，TRD 为 5 条件复合判定，不影响误判率）。
 *
 * @min 0.5
 * @max 5.0
 * @unit m/s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_VZ_MAX, 1.5f);

/**
 * LOCP TRD 水平速度上限（无水平移动判定）
 *
 * 卡网时飞机挂住不动；满油门平飞/水平加速时水平速度超过此值，
 * 说明动力正常（有水平响应）。
 * 标定（2026-08-13）：卡网段水平速度均值 0.02~0.18 m/s；
 * 6-8 月 136 份纯净日志水平速度 P99=1.32、P99.9=3.19 m/s，本机巡航 5~10 m/s。
 * 取 2 m/s：卡网(≈0)与正常巡航(≥5)之间，区分度充足。
 *
 * @min 1
 * @max 10
 * @unit m/s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_HS_MAX, 2.0f);

/**
 * LOCP TRD 最低检测高度
 *
 * 高度低于此值时不做 TRD 检测，排除地面解锁/动力测试
 * （油门推满但不起飞，高度≈0）。
 * 标定（2026-08-13）：地面测试 log_259 高度 0m；卡网 log_309/311 高度 2.5m。
 * 6-8 月另有 5 份地面 arm 调试日志（log_215~219，高度 0.28~0.43m 未起飞），
 * 同样被此门槛排除，不会触发 TRD。
 *
 * @min 0.2
 * @max 3.0
 * @unit m
 * @decimal 1
 * @increment 0.1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_TRD_MIN_H, 1.0f);

// ============================================================
// LOCP 飞机自身健康检查门槛（checkVehicleHealthy）
// ============================================================
// 独立参数，不复用各检测维度阈值，避免调检测参数时间接改变健康门槛。
// 标定（2026-08-13，6-8 月 136 份纯净日志，排除失控/EGO故障/地面假速度）：
// R/P 角速率 P99.9=1.13 rad/s；Yaw P99.9=1.38（正常旋转可达 3.14 rad/s）；
// 水平速度 P99.9=3.19 m/s；|vz| P99.9=1.04 m/s。
// ============================================================

/**
 * LOCP 健康检查：Roll/Pitch 角速率上限
 *
 * 飞机自身状态健康检查的门槛（MTO/OBS 降落安全前提）。
 * 角速率超过此值判定不健康 → MTO/OBS 触发时停桨而非降落。
 * 标定：正常 R/P 角速率 P99.9=1.13 rad/s → 取 2（裕度约 1.8 倍）。
 * 独立于 ARD 检测参数。
 *
 * @min 1
 * @max 10
 * @unit rad/s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_HC_RATE_MAX, 2.0f);

/**
 * LOCP 健康检查：Yaw 角速率上限
 *
 * 正常 yaw 旋转可达 180°/s（3.14 rad/s），阈值需高于 Roll/Pitch。
 * 标定：正常 Yaw 角速率 P99.9=1.38 rad/s → 取 5（裕度约 3.6 倍）。
 * 独立于 ARD 检测参数。
 *
 * @min 1
 * @max 15
 * @unit rad/s
 * @decimal 1
 * @increment 0.5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_HC_YAW_MAX, 5.0f);

/**
 * LOCP 健康检查：水平速度上限
 *
 * 水平速度超过此值判定不健康（EGO 视觉故障等）。
 * 标定：正常飞行水平速度 P99.9=3.19 m/s（已排除地面假速度日志 log_215~219），
 * 本机理论最大速度 10 m/s，取 15（裕度约 4.7 倍）。
 * 独立于 VRD 检测参数。
 *
 * @min 3
 * @max 30
 * @unit m/s
 * @decimal 1
 * @increment 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_HC_HS_MAX, 15.0f);

/**
 * LOCP 健康检查：垂直下降速度上限
 *
 * 垂直下降速度超过此值判定不健康（急坠）。
 * 标定：正常 |vz| P99.9=1.04 m/s，正常降落 ≤2 m/s，
 * 取 10 容忍机动下降（裕度约 5 倍）。
 * 独立于 PRD 检测参数。
 *
 * @min 3
 * @max 20
 * @unit m/s
 * @decimal 1
 * @increment 1
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_HC_VZ_MAX, 10.0f);

// ============================================================
// Crash/Impact Detection —— 碰撞/撞击检测
// ============================================================

// ============================================================
// OBS (Offboard Setpoint Sanity) —— 机载计算机Setpoint异常检测
// ============================================================

/**
 * LOCP OBS 使能开关
 *
 * 启用 Offboard Setpoint 异常检测。检测 MAVROS/机载计算机发送的
 * trajectory_setpoint 是否存在数值跳变或 NaN 注入。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_OBS_EN, 0);

/**
 * LOCP OBS 数值跳变检测使能开关
 *
 * 启用 setpoint 数值跳变检测（位置/速度/Yaw 跳变，LOCP_OBS_J_POS/J_VEL/J_YAW）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_OBS_JMP_EN, 0);

/**
 * LOCP OBS NaN 注入检测使能开关
 *
 * 启用 setpoint NaN 注入检测（有效值突变为 NaN）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_OBS_NAN_EN, 0);

/**
 * LOCP OBS 位置跳变阈值
 *
 * trajectory_setpoint 中相邻两帧位置 setpoint 的最大允许变化量。
 * 任一轴（X/Y/Z）的变化量超过此值触发 OBS 跳变检测。
 *
 * @min 1
 * @max 50
 * @unit m
 * @decimal 1
 * @increment 1
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
 * @decimal 1
 * @increment 0.5
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
 * @decimal 2
 * @increment 0.1
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
 * @increment 0.05
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_OBS_T, 0.3f);

/**
 * LOCP 碰撞检测使能开关
 *
 * 启用碰撞/撞击瞬时检测（基于 IMU 加速度范数阈值）。
 * 设置为 0 可禁用碰撞检测（触发后立即上锁）。
 *
 * 默认禁用（=0），需在 QGC 手动开启。
 * @boolean
 * @min 0
 * @max 1
 * @group LOCP
 */
PARAM_DEFINE_INT32(LOCP_CRASH_EN, 0);

/**
 * LOCP 碰撞加速度阈值
 *
 * IMU 加速度范数（三轴合成加速度幅值）的瞬时阈值。
 * 超过此值立即触发碰撞检测，无需迟滞确认，直接执行上锁动作。
 * 标定（2026-08-13，6-8 月 136 份纯净日志）：正常飞行范数中位 9.8（1g）、
 * P99.9=12.3 m/s²；降落触地冲击（高度≤0.5m）P99=10.9、Max=61.8 m/s²；
 * 真实卡网碰撞（log_309/311/233）107~123 m/s² 且持续 0.05s+。
 * 取 70 m/s²（约 7g）：正常降落不触发，卡网碰撞能触发（裕度约 5.7 倍）。
 *
 * @min 30
 * @max 200
 * @unit m/s^2
 * @decimal 0
 * @increment 5
 * @group LOCP
 */
PARAM_DEFINE_FLOAT(LOCP_CRASH_THR, 70.0f);
