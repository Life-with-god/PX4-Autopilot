# LOCP (Loss-of-Control Protection) 失控保护系统 — 详细说明文档

> **版本**: v1.0
> **适用范围**: PX4-Autopilot `locp` 分支
> **文档日期**: 2026-08-03

---

## 目录

1. [概述与设计背景](#1-概述与设计背景)
2. [系统架构](#2-系统架构)
3. [检测维度详解](#3-检测维度详解)
   - [3.1 ARD — 姿态变化率检测](#31-ard--姿态变化率检测)
   - [3.2 VRD — 速度变化率检测](#32-vrd--速度变化率检测)
   - [3.3 PRD — 位置变化率检测](#33-prd--位置变化率检测)
   - [3.4 COD — 电流异常检测](#34-cod--电流异常检测)
   - [3.5 MTO — MAVLink 消息超时检测](#35-mto--mavlink-消息超时检测)
   - [3.6 OBS — Offboard Setpoint 异常检测](#36-obs--offboard-setpoint-异常检测)
   - [3.7 Crash — 碰撞/撞击检测](#37-crash--碰撞撞击检测)
4. [严重等级仲裁机制](#4-严重等级仲裁机制)
5. [Failsafe 动作集成](#5-failsafe-动作集成)
6. [全部参数说明](#6-全部参数说明)
7. [数据流与触发链路](#7-数据流与触发链路)
8. [MAVLink 遥测与地面站显示](#8-mavlink-遥测与地面站显示)
9. [构建与烧录指南](#9-构建与烧录指南)
10. [SITL 仿真测试指南](#10-sitl-仿真测试指南)
11. [实飞验证与调参建议](#11-实飞验证与调参建议)
12. [故障排查 FAQ](#12-故障排查-faq)

---

## 1. 概述与设计背景

### 1.1 问题场景

在以下场景中，传统 PX4 安全机制可能无法有效保护飞行器：

| 场景 | 传统 failsafe 覆盖情况 | LOCP 覆盖 |
|------|----------------------|----------|
| 遥控器失效（信号丢失） | ✅ RC Loss → RTL/Land | — |
| 地面站断连 | ✅ GCS Loss → RTL/Land | — |
| **无遥控器 + 地面站指令无效** | ❌ 无法检测失控 | ✅ 多维度传感器检测 |
| 传感器故障导致飞行发散 | ❌ 仅 EKF 创新测试，滞后 | ✅ 角加速度/加速度变化率实时检测 |
| 电机/电调突发故障 | ⚠️ 部分（ESC telemetry 超时） | ✅ 电流异常 + dI/dt 尖峰检测 |
| **机载计算机软件 bug 导致异常指令** | ❌ 仅心跳超时检测 | ✅ OBS setpoint 数值跳变/NaN 检测 |
| 空中碰撞/撞击 | ❌ 无检测 | ✅ IMU 加速度范数瞬时触发 |
| 飞控死机/Watchdog 复位 | ❌ 无检测 | ✅ MTO 心跳超时检测 |

### 1.2 设计哲学

LOCP 的核心设计思想是 **"宁可误判、不可漏判"** ——

- 采用 **多维度交叉验证** 降低误报率（单一传感器扰动不会触发）
- 每个检测维度都有独立的 **迟滞滤波器**（Hysteresis）防止瞬时噪声
- 严重等级从 LEVEL_1 到 LEVEL_3 逐步升级，给出故障严重程度的 **量化评估**
- 所有触发动作 **禁止用户接管**（`UserTakeoverAllowed::Never`），确保安全关键响应不被人工取消
- 所有阈值可通过 QGroundControl/QGC 或 `param set` 实时调整

### 1.3 涉及的文件清单

| 文件 | 作用 |
|------|------|
| `src/modules/commander/failure_detector/FailureDetector.hpp` | LOCP 检测类头文件，定义所有检测函数接口和内部状态变量 |
| `src/modules/commander/failure_detector/FailureDetector.cpp` | LOCP 核心检测算法实现（~300 行） |
| `src/modules/commander/failure_detector/failure_detector_params.c` | LOCP 全部参数定义（~42 个可配置参数） |
| `src/modules/commander/Commander.cpp` | 主循环中 LOCP 标志位同步入口（`Commander::run()`） |
| `src/modules/commander/failsafe/failsafe.h` | `locp_failsafe_action` 枚举 + `fromLOCPActParam()` 声明 |
| `src/modules/commander/failsafe/failsafe.cpp` | `checkStateAndMode()` 中 LOCP 三级 + 碰撞检测的 CHECK_FAILSAFE 调用 |
| `src/modules/commander/failsafe/framework.h` | FailsafeBase 抽象基类（Action 优先级、ClearCondition、UserTakeover） |
| `src/modules/commander/failsafe/framework.cpp` | FailsafeBase 状态机实现（checkFailsafe、getSelectedAction 等） |
| `msg/FailsafeFlags.msg` | failsafe_flags uORB 消息定义，包含 LOCP 全部标志位 |
| `src/modules/commander/HealthAndArmingChecks/checks/failureDetectorCheck.cpp` | 解锁前检查中集成 fd_critical_failure |

---

## 2. 系统架构

LOCP 作为 PX4 三层安全架构中 **第二层（检测层）** 的扩展，插入到 FailureDetector 中运行，与原有故障检测共用数据源和状态机框架。

```mermaid
graph TB
    subgraph "数据源传感器"
        IMU[IMU 角速度/加速度]
        EKF[EKF 位置/速度估计]
        BATT[电池电流/电压]
        TELEM[MAVLink 遥测]
        ESC[ESC 遥测]
    end

    subgraph "FailureDetector 检测层"
        FD_ORIG[原有故障检测<br/>姿态超限/ESC故障/电机故障]
        FD_LOCP[LOCP 失控检测<br/>ARD/VRD/PRD/COD/MTO<br/>OBS/Crash]
        ARBITER[严重等级仲裁<br/>evaluateLOCPSeverity]
    end

    subgraph "failsafe_flags uORB 消息"
        FLAGS[failsafe_flags_s<br/>locp_level1/2/3<br/>locp_obs_triggered<br/>crash_detected]
    end

    subgraph "Failsafe 响应层"
        FSM[Failsafe 状态机<br/>checkStateAndMode]
        ACTION[动作执行<br/>Disarm/Terminate/Land/Descend]
    end

    subgraph "飞控执行"
        OUTPUT[actuator_armed<br/>上锁 / 终止输出]
    end

    IMU --> FD_LOCP
    EKF --> FD_LOCP
    BATT --> FD_LOCP
    TELEM --> FD_LOCP
    ESC --> FD_LOCP
    FD_ORIG --> FLAGS
    FD_LOCP --> ARBITER
    ARBITER --> FLAGS
    FLAGS --> FSM
    FSM --> ACTION
    ACTION --> OUTPUT
```

**运行时机**:
- `FailureDetector::update()` 在 Commander 主循环中每 ~10ms 调用一次（取决于主循环频率）
- LOCP 检测仅在 `ARMING_STATE_ARMED`（已解锁）状态下运行
- 未解锁时所有 LOCP 标志自动复位

---

## 3. 检测维度详解

### 3.1 ARD — 姿态变化率检测

**Attitude Rate Detection** — 检测飞行器姿态角速度/角加速度的异常变化。

**检测原理**:
1. **角加速度尖峰检测**: 计算 Roll/Pitch/Yaw 三轴角加速度（对前后两帧角速度做差分），任一轴超过阈值即触发
2. **持续高角速率检测**: 任意轴角速率超过设定点且持续超过 `LOCP_ARD_DUR` 秒

```
角加速度(momentary):  |dω/dt| > LOCP_ARD_R_MAX / LOCP_ARD_P_MAX / LOCP_ARD_Y_MAX
持续高角速率:         |ω| > LOCP_ARD_RSP / LOCP_ARD_PSP 且持续时间 > LOCP_ARD_DUR
```

**相关参数**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LOCP_ARD_EN` | 1 | ARD 检测使能开关 |
| `LOCP_ARD_R_MAX` | 80 rad/s² | Roll 角加速度最大阈值 |
| `LOCP_ARD_P_MAX` | 80 rad/s² | Pitch 角加速度最大阈值 |
| `LOCP_ARD_Y_MAX` | 60 rad/s² | Yaw 角加速度最大阈值 |
| `LOCP_ARD_T` | 0.3 s | 迟滞确认时间 |
| `LOCP_ARD_RSP` | 6 rad/s | Roll 持续高角速率设定点 |
| `LOCP_ARD_PSP` | 6 rad/s | Pitch 持续高角速率设定点 |
| `LOCP_ARD_YSP` | 5 rad/s | Yaw 持续高角速率设定点 |
| `LOCP_ARD_DUR` | 0.3 s | 持续高角速率最小持续时间 |

---

### 3.2 VRD — 速度变化率检测

**Velocity Rate Detection** — 检测飞行器线速度变化率（加速度）的异常。

**检测原理**:
1. **水平加速度异常**: 水平面合成加速度 `√(ax²+ay²)` 超过阈值
2. **自由落体/动力急降**: 垂直向下加速度 `az` 和垂直下降速度 `vz` 同时超限（双条件 AND）
3. **Jerk（加加速度）检测**: 水平加速度变化率 `|Δa_horiz|/dt` 超过阈值（检测急加速/急减速）

```
水平加速度异常:  √(ax²+ay²) > LOCP_VRD_AH_MAX
自由落体:        az > LOCP_VRD_AD_MAX  AND  vz > LOCP_VRD_VZD_MAX
Jerk异常:        |Δa_horiz|/dt > LOCP_VRD_JERK
```

**相关参数**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LOCP_VRD_EN` | 1 | VRD 检测使能开关 |
| `LOCP_VRD_AH_MAX` | 8 m/s² | 水平加速度最大阈值 |
| `LOCP_VRD_AD_MAX` | 6 m/s² | 垂直向下加速度阈值（NED Z+） |
| `LOCP_VRD_VZD_MAX` | 3 m/s | 垂直下降速度阈值（配合自由落体判断） |
| `LOCP_VRD_JERK` | 50 m/s³ | 水平 Jerk 阈值 |
| `LOCP_VRD_T` | 0.3 s | 迟滞确认时间 |

---

### 3.3 PRD — 位置变化率检测

**Position Rate Detection** — 检测飞行器位置/高度变化率的异常。

**检测原理**:
1. **急降检测**: 垂直下降速度 `vz` 和相对于 Home 点的高度下降量同时超限（双条件 AND）
2. **高度振荡检测**: 维护 10 帧高度数据的环形缓冲，计算滑动窗口标准差，超过阈值判定失控振荡
3. **水平漂移检测**: 水平面合成速度 `√(vx²+vy²)` 超过阈值

```
急降:          vz > LOCP_PRD_VZ_MAX  AND  (z - home.z) > LOCP_PRD_ADROP
高度振荡:      σ_10帧(高度) > LOCP_PRD_ASTD (需要至少5帧数据)
水平漂移:      √(vx²+vy²) > LOCP_PRD_HSPD
```

**相关参数**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LOCP_PRD_EN` | 1 | PRD 检测使能开关 |
| `LOCP_PRD_VZ_MAX` | 5 m/s | 垂直下降速度阈值 |
| `LOCP_PRD_ADROP` | 3 m | 高度下降量阈值（相对 Home 点） |
| `LOCP_PRD_ASTD` | 2 m | 高度振荡标准差阈值（10帧滑动窗口） |
| `LOCP_PRD_HSPD` | 5 m/s | 水平漂移速度阈值 |
| `LOCP_PRD_T` | 0.5 s | 迟滞确认时间 |

---

### 3.4 COD — 电流异常检测

**Current Overdraw Detection** — 检测动力系统电流的异常变化。

**检测原理**:
1. **总电流突增**: 维护 20 帧电流数据的环形缓冲，实时比较当前值与滑动均值的偏差；或绝对电流超限
2. **dI/dt 尖峰**: 前后两帧电流差分计算电流变化率，检测瞬间电流尖峰（如电机堵转）
3. **单路 ESC 过流**: 检测任意单路 ESC 电流是否异常偏高（超过各 ESC 均值的 2 倍且超过绝对阈值）

```
总电流突增:       I_now - I_avg_20f > LOCP_COD_DELTA_I  OR  I_now > LOCP_COD_MAX_I
dI/dt尖峰:        |dI/dt| > LOCP_COD_DI_DT
单路ESC过流:      I_esc[i] > avg(I_esc[all]) * 2  AND  I_esc[i] > LOCP_COD_ESC_MAX
```

**相关参数**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LOCP_COD_EN` | 1 | COD 检测使能开关 |
| `LOCP_COD_DELTA_I` | 10 A | 总电流偏离滑动均值阈值 |
| `LOCP_COD_MAX_I` | 45 A | 总电流绝对最大阈值 |
| `LOCP_COD_DI_DT` | 30 A/s | 电流变化率 dI/dt 阈值 |
| `LOCP_COD_ESC_MAX` | 10 A | 单路 ESC 电流绝对最大阈值 |
| `LOCP_COD_T` | 0.3 s | 迟滞确认时间 |

---

### 3.5 MTO — MAVLink 消息超时检测

**MAVLink Timeout** — 检测 MAVLink 通信链路的中断。

**检测原理**:
1. **心跳超时**: 距最后一次收到 MAVLink 心跳超过 `LOCP_MTO_HB_T` 秒
2. **指令超时**: 距最后一次收到 vehicle_command 超过 `LOCP_MTO_CMD_T` 秒
3. **消息速率骤降**: 1 秒滑动窗口内消息速率低于 `LOCP_MTO_RATE` Hz

**触发条件（逻辑与）**:
- **条件A**: `心跳丢失` —— 单一即触发
- **条件B**: `指令超时 AND 消息速率骤降` —— 需要同时满足

最终 MTO 触发条件 = 条件A OR 条件B。 这样设计的意图是：仅心跳丢失可能只是心跳通道故障，而指令通道和数据通道仍然正常；但若心跳丢失，系统仍然保守地将其视为通信中断。

```
触发MTO:  心跳丢失 OR (指令超时 AND 速率骤降)
```

**相关参数**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LOCP_MTO_EN` | 1 | MTO 检测使能开关 |
| `LOCP_MTO_HB_T` | 1.5 s | 心跳超时阈值 |
| `LOCP_MTO_CMD_T` | 2.0 s | 指令超时阈值 |
| `LOCP_MTO_RATE` | 3 Hz | 消息最低速率阈值 |

---

### 3.6 OBS — Offboard Setpoint 异常检测

**Offboard Setpoint Sanity** — 检测 MAVROS/机载计算机发送的 trajectory_setpoint 是否存在数值异常。

**背景**: 在 Offboard 模式下，机载计算机通过 MAVLink `SET_POSITION_TARGET_LOCAL_NED` 消息持续向飞控发送 setpoint 指令。 如果机载计算机软件出现 bug（如数值溢出、野指针、ROS 节点间通信异常），可能发送：
- **数值跳变 (Spike)**: setpoint 在相邻帧间剧烈跳动（如位置突然偏移 50m）
- **NaN 注入**: 受控轴的 setpoint 突然变为 NaN（无效浮点数）

现有的 `offboard_control_signal_lost` 只能检测心跳超时，无法检测这种 "心跳正常但数值异常" 的情况。 OBS 填补了这个空白。

**检测原理**:

1. **NaN 注入检测**: 订阅 `trajectory_setpoint` 话题，记录上一帧各轴的有效数值。 如果某轴上一帧有有效数值（非 NaN），而当前帧变为 NaN → 触发异常。

2. **数值跳变检测**: 对于连续两帧都有有效数值的轴，计算变化量并比较阈值：
   - 位置跳变: `|pos_new - pos_old| > LOCP_OBS_JUMP_POS`
   - 速度跳变: `|vel_new - vel_old| > LOCP_OBS_JUMP_VEL`
   - Yaw 跳变: `|yaw_new - yaw_old| > LOCP_OBS_JUMP_YAW`（自动处理 ±PI 环绕）

```
NaN注入:    PX4_ISFINITE(prev) AND !PX4_ISFINITE(curr)  → 异常
位置跳变:  |pos_curr - pos_prev| > LOCP_OBS_JUMP_POS   → 异常
速度跳变:  |vel_curr - vel_prev| > LOCP_OBS_JUMP_VEL   → 异常
Yaw跳变:   |yaw_diff| > LOCP_OBS_JUMP_YAW               → 异常
```

**OBS 触发的动作**:
OBS 是 LOCP 体系中的一个独立维度，**同时**参与两个层面的触发：
1. **作为维度参与严重等级仲裁**: 增加 `evaluateLOCPSeverity()` 中的触发计数
2. **作为独立 failsafe 通道**: 直接触发 `CHECK_FAILSAFE(locp_obs_triggered)`，按独立的 `LOCP_OBS_ACT` 执行（默认 Land，不允许用户接管）

这样的双重机制确保：即使其他维度都正常（count=0），只要 OBS 自身触发，就会立即执行保护动作。 同时 OBS 动作由 `LOCP_OBS_ACT` 独立配置，避免单次 setpoint 异常触发过于激进的 Disarm。

**OBS 不检测的场景**:
- ❌ 心跳正常但不再发送 setpoint（由 `offboard_control_signal_lost` 检测）
- ❌ 机载计算机过热降频导致控制周期变长（频率变化不在检测范围）
- ❌ MAVROS 崩溃后自动重启（心跳丢失由现有机制检测）
- ✅ **只检测 setpoint 数值本身的合法性**

**数据源**:
OBS 直接订阅 `trajectory_setpoint` uORB 话题。 这个 topic 由 `mavlink_receiver` 在收到 `SET_POSITION_TARGET_LOCAL_NED` MAVLink 消息后发布。

**相关参数**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LOCP_OBS_EN` | 1 | OBS 检测使能开关 |
| `LOCP_OBS_JUMP_POS` | 10 m | 位置跳变阈值 |
| `LOCP_OBS_JUMP_VEL` | 5 m/s | 速度跳变阈值 |
| `LOCP_OBS_JUMP_YAW` | 1.57 rad (~90°) | Yaw 跳变阈值 |
| `LOCP_OBS_T` | 0.3 s | 迟滞确认时间 |
| `LOCP_OBS_ACT` | 3 (Land) | OBS 触发动作（独立配置） |

---

### 3.7 Crash — 碰撞/撞击检测

**Crash/Impact Detection** — 检测飞行器是否受到外部碰撞或撞击。

**检测原理**:
- 计算 IMU 三轴加速度范数 `|a| = √(ax²+ay²+az²)`
- 瞬时超过阈值即触发，**无迟滞滤波**（碰撞是瞬时事件，不能等待 confirm time）
- 触发后锁存 2 秒，给予飞控足够的响应时间，然后自动清除

```
碰撞触发:  √(ax²+ay²+az²) > LOCP_CRASH_THR
锁存时间:  触发后持续保持 2 秒，然后自动清除
```

**为什么碰撞检测是独立通道而不是 LOCP 维度之一？**

碰撞是瞬时事件，不受迟滞滤波；且它的响应动作是 Disarm（上锁），不可延迟、不可接管。 因此它直接通过 failsafe_flags 中的 `crash_detected` 字段触发，不经过 `evaluateLOCPSeverity()` 的等级仲裁。

**相关参数**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LOCP_CRASH_THR` | 50 m/s² (~5.1g) | 加速度范数碰撞阈值 |

---

## 4. 严重等级仲裁机制

`evaluateLOCPSeverity()` 根据各维度的触发情况，按以下规则输出综合严重等级：

| 条件下触发的维度数 | 致命组合 | 通信状态 | 输出等级 | 默认动作 |
|-------------------|---------|----------|---------|----------|
| 0 | — | — | **NONE (0)** | 无动作 |
| 1 | — | 通信正常 | **LEVEL_1 (1)** | 降落 (Land) |
| 2 | — | — | **LEVEL_2 (2)** | 降落 (Land) |
| ≥3 | — | — | **LEVEL_3 (3)** | 上锁 (Disarm) |
| 任意 | COD + ARD 同时触发 | — | **LEVEL_3 (3)** | 上锁 (Disarm) |
| ≥1 (非 MTO) | — | MTO 触发 | **LEVEL_2 (2)** | 降落 (Land) |
| 任意 | OBS 独立触发 | — | **OBS_ACT** | 降落 (Land) |

**致命组合（Fatal Combo）说明**:
- `COD && ARD` — 电流异常 + 姿态异常同时触发
- 这两个维度来自独立数据源（电池电流 + IMU 角速度），同时触发是真实动力系统故障证据
- 直接跳级到 LEVEL_3，不按维度数量计算，也不做折扣

**数据源相关性折扣（Correlation Discount）**:
- VRD 和 PRD 共用同一个 EKF 估计数据源（vehicle_local_position）
- 如果 EKF 因 GPS/光流异常而发散，VRD 和 PRD 会**同时**触发——但这不是双重故障证据，而是单一 EKF 源问题的连锁反应
- 因此当 VRD 和 PRD 同时触发时，**按 1 个维度计算**，防止 EKF 源问题被误判为 LEVEL_2/3

**OBS 独立触发说明**:
- OBS 除了参与维度计数，还有一条独立的 failsafe 通道
- 独立通道按 `LOCP_OBS_ACT` 执行（默认 Land），避免单次 setpoint 异常触发激进的 Disarm

**等级判定伪代码**:
```cpp
int count = ARD + VRD + PRD + COD + MTO + OBS;

// 数据源相关性折扣：VRD 和 PRD 共用 EKF，同时触发按 1 维计算
if (VRD && PRD) count -= 1;

bool fatal = COD && ARD;                    // 独立数据源，不做折扣
int count_excl_mto = count - (MTO ? 1 : 0); // 排除 MTO 自身
bool comm_dead = MTO && (count_excl_mto >= 1); // 需要至少 1 个其他异常

if (count >= 3 || fatal)     return 3; // LEVEL_3: 终止飞行
if (count >= 2 || comm_dead) return 2; // LEVEL_2: 紧急动作
if (count >= 1)              return 1; // LEVEL_1: 常规动作
return 0;                              // NONE: 正常飞行
```

---

## 5. Failsafe 动作集成

LOCP 检测结果通过以下链路最终转化为飞控执行动作：

### 5.1 标志位同步（Commander::run()）

```cpp
// Commander.cpp - 主循环中 (~1837行附近)
failsafe_flags_s &locp_flags = _health_and_arming_checks.failsafeFlags();

// 各维度触发标志
locp_flags.locp_ard_triggered = _failure_detector.getLOCP_ARD();
locp_flags.locp_vrd_triggered = _failure_detector.getLOCP_VRD();
locp_flags.locp_prd_triggered = _failure_detector.getLOCP_PRD();
locp_flags.locp_cod_triggered = _failure_detector.getLOCP_COD();
locp_flags.locp_mto_triggered = _failure_detector.getLOCP_MTO();
locp_flags.crash_detected    = _failure_detector.getCrashDetected();

// 综合等级
uint8_t sev = _failure_detector.getLOCPSeverity();
locp_flags.locp_severity = sev;
locp_flags.locp_level1   = (sev >= 1);
locp_flags.locp_level2   = (sev >= 2);
locp_flags.locp_level3   = (sev >= 3);
```

### 5.2 状态机评估（Failsafe::checkStateAndMode()）

```cpp
// failsafe.cpp - checkStateAndMode() 中 (~690行附近)
// LEVEL_1
CHECK_FAILSAFE(status_flags, locp_level1,
    fromLOCPActParam(_param_locp_l1_act.get()));
// LEVEL_2
CHECK_FAILSAFE(status_flags, locp_level2,
    fromLOCPActParam(_param_locp_l2_act.get()));
// LEVEL_3
CHECK_FAILSAFE(status_flags, locp_level3,
    fromLOCPActParam(_param_locp_l3_act.get()));
// Crash
CHECK_FAILSAFE(status_flags, crash_detected,
    ActionOptions(Action::Disarm).cannotBeDeferred()
    .allowUserTakeover(UserTakeoverAllowed::Never));
```

### 5.3 动作参数映射（fromLOCPActParam）

| 参数值 | 枚举 | Failsafe Action | 清除条件 | 用户接管 |
|--------|------|----------------|----------|---------|
| 0 | None | 无动作 | — | — |
| 1 | Warning | Warn（仅警告） | WhenConditionClears | Never |
| 2 | Hold | Hold（悬停） | OnModeChangeOrDisarm | Never |
| 3 | Land | Land（降落） | OnDisarm | Never |
| 4 | Descend | Descend（急降） | OnDisarm | Never |
| 5 | RTL | RTL（返航） | OnDisarm | Never |
| 6 | Terminate | Terminate（终止） | Never | Never |
| 7 | Disarm | Disarm（上锁） | Never | Never |

> **重要**: 所有 LOCP 触发的动作**禁止用户接管**（`UserTakeoverAllowed::Never`）。 这意味着一旦 LOCP 触发，遥控器操纵杆移动或模式切换无法取消保护动作——这是安全关键设计。

### 5.4 动作执行（handleModeIntentionAndFailsafe）

```cpp
// Commander.cpp - handleModeIntentionAndFailsafe() 中
switch (_failsafe.selectedAction()) {
case FailsafeBase::Action::Disarm:
    disarm(arm_disarm_reason_t::failsafe, true);  // 强制上锁
    break;
case FailsafeBase::Action::Terminate:
    _vehicle_status.nav_state = NAVIGATION_STATE_TERMINATION;
    // 输出切换到 failsafe 值
    break;
}
```

---

## 6. 全部参数说明

### 6.1 ARD — 姿态变化率检测参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `LOCP_ARD_EN` | INT32 | 1 | 0/1 | — | ARD 检测使能 |
| `LOCP_ARD_R_MAX` | FLOAT | 80 | 20~500 | rad/s² | Roll 角加速度阈值 |
| `LOCP_ARD_P_MAX` | FLOAT | 80 | 20~500 | rad/s² | Pitch 角加速度阈值 |
| `LOCP_ARD_Y_MAX` | FLOAT | 60 | 20~500 | rad/s² | Yaw 角加速度阈值 |
| `LOCP_ARD_T` | FLOAT | 0.3 | 0.1~2.0 | s | 迟滞确认时间 |
| `LOCP_ARD_RSP` | FLOAT | 6.0 | 3~30 | rad/s | Roll 持续高角速率设定点 |
| `LOCP_ARD_PSP` | FLOAT | 6.0 | 3~30 | rad/s | Pitch 持续高角速率设定点 |
| `LOCP_ARD_YSP` | FLOAT | 5.0 | 3~30 | rad/s | Yaw 持续高角速率设定点 |
| `LOCP_ARD_DUR` | FLOAT | 0.3 | 0.1~2.0 | s | 持续高角速率最小持续时间 |

### 6.2 VRD — 速度变化率检测参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `LOCP_VRD_EN` | INT32 | 1 | 0/1 | — | VRD 检测使能 |
| `LOCP_VRD_AH_MAX` | FLOAT | 8.0 | 5~50 | m/s² | 水平加速度阈值 |
| `LOCP_VRD_AD_MAX` | FLOAT | 6.0 | 3~20 | m/s² | 垂直向下加速度阈值 |
| `LOCP_VRD_VZD_MAX` | FLOAT | 3.0 | 1~15 | m/s | 垂直下降速度阈值 |
| `LOCP_VRD_JERK` | FLOAT | 50.0 | 10~200 | m/s³ | Jerk 阈值 |
| `LOCP_VRD_T` | FLOAT | 0.3 | 0.1~2.0 | s | 迟滞确认时间 |

### 6.3 PRD — 位置变化率检测参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `LOCP_PRD_EN` | INT32 | 1 | 0/1 | — | PRD 检测使能 |
| `LOCP_PRD_VZ_MAX` | FLOAT | 5.0 | 1~20 | m/s | 垂直下降速度阈值 |
| `LOCP_PRD_ADROP` | FLOAT | 3.0 | 1~50 | m | 高度下降量阈值 |
| `LOCP_PRD_ASTD` | FLOAT | 2.0 | 0.5~10 | m | 高度振荡标准差阈值 |
| `LOCP_PRD_HSPD` | FLOAT | 5.0 | 1~30 | m/s | 水平漂移速度阈值 |
| `LOCP_PRD_T` | FLOAT | 0.5 | 0.1~2.0 | s | 迟滞确认时间 |

### 6.4 COD — 电流异常检测参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `LOCP_COD_EN` | INT32 | 1 | 0/1 | — | COD 检测使能 |
| `LOCP_COD_DELTA_I` | FLOAT | 10.0 | 5~40 | A | 偏离滑动均值阈值 |
| `LOCP_COD_MAX_I` | FLOAT | 45.0 | 10~150 | A | 总电流绝对阈值 |
| `LOCP_COD_DI_DT` | FLOAT | 30.0 | 10~200 | A/s | dI/dt 阈值 |
| `LOCP_COD_ESC_MAX` | FLOAT | 10.0 | 5~50 | A | 单路 ESC 电流阈值 |
| `LOCP_COD_T` | FLOAT | 0.3 | 0.1~2.0 | s | 迟滞确认时间 |

### 6.5 MTO — MAVLink 超时检测参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `LOCP_MTO_EN` | INT32 | 1 | 0/1 | — | MTO 检测使能 |
| `LOCP_MTO_HB_T` | FLOAT | 1.5 | 0.5~10 | s | 心跳超时阈值 |
| `LOCP_MTO_CMD_T` | FLOAT | 2.0 | 0.5~10 | s | 指令超时阈值 |
| `LOCP_MTO_RATE` | FLOAT | 3.0 | 0.5~50 | Hz | 消息最低速率阈值 |

### 6.6 OBS — Offboard Setpoint 异常检测参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `LOCP_OBS_EN` | INT32 | 1 | 0/1 | — | OBS 检测使能 |
| `LOCP_OBS_JUMP_POS` | FLOAT | 10.0 | 1~50 | m | 位置跳变阈值 |
| `LOCP_OBS_JUMP_VEL` | FLOAT | 5.0 | 1~20 | m/s | 速度跳变阈值 |
| `LOCP_OBS_JUMP_YAW` | FLOAT | 1.57 | 0.5~6.28 | rad | Yaw 跳变阈值 |
| `LOCP_OBS_T` | FLOAT | 0.3 | 0.1~2.0 | s | 迟滞确认时间 |
| `LOCP_OBS_ACT` | INT32 | 3 | 0~7 | — | OBS 触发动作（默认 Land） |

### 6.7 碰撞检测参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `LOCP_CRASH_THR` | FLOAT | 50.0 | 30~200 | m/s² | 加速度范数碰撞阈值 |

### 6.8 等级动作配置参数

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `LOCP_L1_ACT` | INT32 | 3 (Land) | 0~7 | LEVEL_1 触发动作 |
| `LOCP_L2_ACT` | INT32 | 3 (Land) | 0~7 | LEVEL_2 触发动作 |
| `LOCP_L3_ACT` | INT32 | 7 (Disarm) | 0~7 | LEVEL_3 触发动作 |

**动作值映射**:

| 值 | 动作 | 说明 |
|----|------|------|
| 0 | None | 不做任何响应 |
| 1 | Warning | 仅发出警告（地面站 + 日志） |
| 2 | Hold | 悬停保持当前位置 |
| 3 | Land | 降落到地面 |
| 4 | Descend | 快速下降（Emergency Descent） |
| 5 | RTL | 返航（Return to Launch） |
| 6 | Terminate | 飞行终止（输出切换到 failsafe 值） |
| 7 | Disarm | 立即上锁/停止电机 |

---

## 7. 数据流与触发链路

### 7.1 完整触发链路（从传感器到动作）

```
时间线（每个 Commander 主循环周期 ~10ms）:

T+0ms    传感器数据更新:
         vehicle_angular_velocity   → ang_vel (100Hz)
         vehicle_local_position     → loc     (50Hz)
         battery_status             → bat     (10Hz)
         esc_status                 → esc     (10Hz)
         telemetry_status           → telemetry
         vehicle_acceleration       → accel   (100Hz)

T+1ms    FailureDetector::update()
         ├─ updateAttitudeStatus()         [原有]
         ├─ updateEscsStatus()             [原有]
         ├─ updateMotorStatus()            [原有]
         ├─ updateImbalancedPropStatus()   [原有]
         └─ updateLOCP()                   [LOCP] ★
             ├─ checkAttitudeRateAnomaly(ang_vel)   → _locp_ard_triggered
             ├─ checkVelocityRateAnomaly(loc)       → _locp_vrd_triggered
             ├─ checkPositionRateAnomaly(loc, home) → _locp_prd_triggered
             ├─ checkCurrentAnomaly(bat, esc)       → _locp_cod_triggered
             ├─ checkMavlinkTimeout()               → _locp_mto_triggered
             ├─ checkOffboardSetpointSanity()       → _locp_obs_triggered
             ├─ checkCrashImpact()                  → _crash_detected
             └─ evaluateLOCPSeverity()              → _locp_severity

T+2ms    Commander::run() — LOCP 标志位同步:
         failsafe_flags.locp_ard_triggered = getLOCP_ARD()
         failsafe_flags.locp_vrd_triggered = getLOCP_VRD()
         failsafe_flags.locp_prd_triggered = getLOCP_PRD()
         failsafe_flags.locp_cod_triggered = getLOCP_COD()
         failsafe_flags.locp_mto_triggered = getLOCP_MTO()
         failsafe_flags.locp_obs_triggered = getLOCP_OBS()
         failsafe_flags.crash_detected     = getCrashDetected()
         failsafe_flags.locp_severity      = getLOCPSeverity()
         failsafe_flags.locp_level1/2/3    = sev >= 1/2/3

T+3ms    handleModeIntentionAndFailsafe()
         └─ Failsafe::update()
             └─ checkStateAndMode()
                 ├─ CHECK_FAILSAFE(locp_level1) → action L1
                 ├─ CHECK_FAILSAFE(locp_level2) → action L2
                 ├─ CHECK_FAILSAFE(locp_level3) → action L3
                 ├─ CHECK_FAILSAFE(crash_detected) → Disarm
                 └─ CHECK_FAILSAFE(locp_obs_triggered) → action L2 (独立通道)

T+4ms    getSelectedAction()
         → 选择最高优先级 Action (可能产生 Hold 延迟)

T+5ms    handleModeIntentionAndFailsafe() — 动作执行
         ├─ Action::Disarm    → disarm(failsafe, forced=true)
         └─ Action::Terminate → nav_state = NAVIGATION_STATE_TERMINATION

T+6ms    actuator_armed 发布
         → PWM 输出切换到 failsafe 值 或 上锁
```

### 7.2 所属参数组

所有 LOCP 参数属于 `LOCP` 参数组。在 QGroundControl 的参数页面中，可以按组筛选 "LOCP" 来查看和修改所有 LOCP 相关参数。

---

## 8. MAVLink 遥测与地面站显示

### 8.1 failsafe_flags 消息中的 LOCP 字段

LOCP 检测结果通过 `failsafe_flags` uORB 消息对外暴露，可通过 MAVLink 遥测链路查看。 以下是新增的字段：

```c
// msg/FailsafeFlags.msg — LOCP 相关字段

// --- 各检测维度触发标志 ---
bool locp_ard_triggered       // ARD 姿态变化率异常触发
bool locp_vrd_triggered       // VRD 速度变化率异常触发
bool locp_prd_triggered       // PRD 位置变化率异常触发
bool locp_cod_triggered       // COD 电流异常触发
bool locp_mto_triggered       // MTO MAVLink超时触发

// --- 综合严重等级 ---
uint8 locp_severity           // 0=NONE, 1=LEVEL_1, 2=LEVEL_2, 3=LEVEL_3

// --- 派生等级标志 ---
bool locp_level1              // 等级 >= 1
bool locp_level2              // 等级 >= 2
bool locp_level3              // 等级 >= 3

// --- 碰撞/撞击检测 ---
bool crash_detected           // 碰撞检测触发
```

### 8.2 在地面站中查看

1. **QGroundControl (QGC)**:
   - 在 `Analyze Tools` → `MAVLink Inspector` 中选择 `FAILSAFE_FLAGS` 消息
   - 或使用 `Widgets` → `Custom Command` 发送 MAVLink 请求

2. **MAVLink Console** (通过 `mavlink_shell.py`):
   ```bash
   # 查看 LOCP 参数
   listener failsafe_flags
   # 查看 LOCP 相关参数
   param show LOCP
   ```

3. **日志分析** (通过 `plotjuggler` 或 `flight_review`):
   - 搜索 `failsafe_flags` topic
   - 绘制 `locp_severity`、`locp_level1/2/3`、`crash_detected` 等字段

---

## 9. 构建与烧录指南

### 9.1 前置环境要求

在开始之前，请确保已按照 PX4 官方文档完成开发环境搭建：

- **Ubuntu 20.04/22.04** (推荐)
- **PX4 工具链**: [PX4 Ubuntu 开发环境指南](https://docs.px4.io/main/zh/dev_setup/dev_env_linux_ubuntu.html)

安装必要工具：

```bash
# 克隆 PX4-Autopilot（如果尚未克隆）
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 切换到 locp 分支（包含 LOCP 安全机制的分支）
git checkout locp

# 更新子模块
git submodule update --init --recursive

# 运行 PX4 环境配置脚本（首次使用）
bash ./Tools/setup/ubuntu.sh
```

### 9.2 支持的飞控硬件

LOCP 安全机制已集成到 Commander 模块中，支持 **所有** PX4 兼容的飞控硬件。 以下为经过测试的硬件平台：

| 飞控 | Build Target | 架构 |
|------|-------------|------|
| CUAV 7-Nano | `make cuav_7-nano_default` | Cortex-M7 (STM32H7) |
| Pixhawk FMUv5 | `make px4_fmu-v5_default` | Cortex-M7 (STM32F7) |
| Pixhawk FMUv6 | `make px4_fmu-v6_default` | Cortex-M7 (STM32H7) |
| CUAV X7 | `make cuav_x7_default` | Cortex-M7 (STM32H7) |
| Holybro Pix32v5 | `make holybro_pix32v5_default` | Cortex-M7 (STM32F7) |
| Cube Orange | `make cubepilot_cubeorange_default` | Cortex-M7 (STM32H7) |
| SITL (仿真) | `make px4_sitl_default` | x86_64 |

### 9.3 构建固件

#### 方式一：命令行构建（推荐）

```bash
cd ~/PX4-Autopilot

# 以 CUAV 7-Nano 为例
make cuav_7-nano_default

# 构建完成后，固件位于:
# build/cuav_7-nano_default/cuav_7-nano_default.px4
```

#### 方式二：使用 VS Code 构建

1. 在 VS Code 中打开 `~/PX4-Autopilot` 文件夹
2. 按 `Ctrl+Shift+P`，选择 `CMake: Select Configure Preset`
3. 选择对应的飞控目标（如 `cuav_7-nano_default`）
4. 按 `Ctrl+Shift+P`，选择 `CMake: Build`

### 9.4 进入 Bootloader 模式（How to Enter Bootloader）

烧录固件前，飞控必须进入 **Bootloader 模式**。 不同飞控硬件的进入方式略有不同，以下是各主流飞控的详细操作方法：

---

#### 方法一：安全开关/BOOT 按钮法（最常用）

> 适用于：**Pixhawk 4/5/6 系列、CUAV 系列、Cube 系列、Holybro 系列** 等绝大多数 STM32 飞控

**操作步骤**：

1. 确认飞控与电脑之间的 USB 线**未连接**（先拔掉 USB）
2. 找到飞控上的 **安全开关 (Safety Button)** 或 **BOOT 按钮**：

| 飞控型号 | 按钮位置 | 说明 |
|----------|----------|------|
| Pixhawk 4 (FMUv5) | 侧边 **SAFETY** 按钮 | 较长的矩形按钮，旁边有 LED |
| Pixhawk 5X/6X (FMUv5X/v6) | 侧边 **BOOT** 按钮 | 小型圆形按钮，靠近 USB 口 |
| CUAV 7-Nano / X7 | 顶部 **BOOT** 按钮 | 标记为 "BOOT" 的微动开关 |
| Cube Orange | 顶部 **SAFETY** 按钮 | 主接口旁边的安全开关 |
| Holybro Pix32v5/v6 | 侧边 **BOOT** 按钮 | 标记为 "BOOT" 或 "SAFETY" |

3. **按住不放**该按钮
4. **保持按住**的同时，将 USB 线插入飞控和电脑
5. 等待 **2~3 秒**，松开按钮
6. 此时飞控 LED 应快速闪烁或保持常亮（不同飞控表现不同），表示已进入 Bootloader 模式

> :warning: **关键提示**: 一定是**先按住按钮 :arrow_right: 再插 USB**，顺序不能反！

---

#### 方法二：QGroundControl 软件重启法

> 适用于：飞控**当前固件正常运行**、可以通过 USB 连接 QGC 的情况

**操作步骤**：

1. 用 USB 线连接飞控到电脑（正常连接，不需要按按钮）
2. 打开 QGroundControl，确认飞控已连接
3. 进入 `设置` (齿轮图标) :arrow_right: `固件` (Firmware) 页面
4. QGC 会自动检测飞控并弹出固件更新对话框
5. 在对话框中选择 **"高级设置"** :arrow_right: 勾选 **"自定义固件文件"**
6. 选择您编译好的 `.px4` 文件
7. QGC 会自动让飞控重启进入 Bootloader 并完成烧录

> :warning: **注意**: 如果飞控当前固件已损坏无法启动，请用**方法一**或**方法三**。

---

#### 方法三：NSH 命令行重启法

> 适用于：可以通过 **MAVLink Shell** 或 **USB 串口终端** 连接到飞控 NSH 的情况

```bash
# 方式 A: 通过 MAVLink Shell（如果飞控已连接 QGC）
cd ~/PX4-Autopilot
./Tools/mavlink_shell.py

# 在 NSH 终端中执行：
nsh> reboot -b
# 或
nsh> bootloader

# 飞控会立即重启进入 Bootloader 模式
```

```bash
# 方式 B: 通过 USB 串口直接连接
# 先找到飞控的串口设备
ls /dev/ttyACM*

# 使用 screen 或 miniterm 连接
screen /dev/ttyACM0 57600

# 按 Enter 键 3 次，出现 nsh> 提示符后：
nsh> reboot -b
```

> :warning: **注意**: 执行 `reboot -b` 后飞控立即进入 Bootloader，NSH 连接会断开。 此时执行 `make upload` 即可烧录。

---

#### 方法四：MAVLink 命令重启法

> 适用于：可以通过 MAVLink 链路（USB/数传/WiFi）与飞控通信的情况

**使用 pymavlink 发送命令**:

```python
#!/usr/bin/env python3
import pymavlink.mavutil as mavutil

# 连接到飞控
conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
conn.wait_heartbeat()

# 发送 reboot to bootloader 命令 (param1=3)
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    0,  # confirmation
    3,  # param1: 3 = reboot to bootloader
    0, 0, 0, 0, 0, 0  # param2-7: unused
)
```

---

#### 方法五：硬件短接法（仅限特定飞控，作为最后手段）

> :warning: **危险操作，仅限有经验的开发者！短路可能导致飞控永久损坏！**

某些飞控在 Bootloader 损坏或按钮失效时，可以通过短接 PCB 上的 BOOT0 引脚强制进入系统 Bootloader：

| 飞控型号 | 短接方法 |
|----------|----------|
| Pixhawk 1 (FMUv2) | 短接 PCB 上的 **BOOT0** 和 **3.3V** 焊盘 |
| Pixracer (FMUv4) | 短接 **BOOT** 和 **RST** 焊盘 |
| CUAV V5+ | 短接 **BOOT0** 排针 |

**操作步骤（通用）**:
1. 彻底断开飞控电源（拔出 USB 和电池）
2. 用镊子或跳线帽短接 BOOT0 引脚
3. 保持短接，插上 USB
4. 等待 3 秒，移除短接
5. 此时飞控进入 STM32 系统 Bootloader（DFU 模式）
6. 使用 STM32CubeProgrammer 或 `dfu-util` 烧录

---

#### 如何确认飞控已进入 Bootloader？

```bash
# 在 Linux 上检查 USB 设备
lsusb

# 如果看到以下任一设备，说明已进入 Bootloader：
# - "STMicroelectronics STM32 BOOTLOADER"  （DFU 模式）
# - "PX4 BL" 或 "3D Robotics PX4"           （PX4 Bootloader）

# 也可以用 dmesg 查看内核日志
dmesg | tail -20

# 正常启动时通常显示为：
#   cdc_acm: USB ACM device
# Bootloader 模式下显示为：
#   usb 1-1: New USB device found, idVendor=0483, idProduct=df11
```

---

### 9.5 烧录固件到飞控

#### 方式一：命令行一键烧录（推荐）

```bash
cd ~/PX4-Autopilot

# 构建 + 烧录一键完成
make cuav_7-nano_default upload

# 或者分步执行：
# 步骤1: 先构建固件
make cuav_7-nano_default

# 步骤2: 飞控进入 Bootloader（参见 9.4 节）

# 步骤3: 上传固件
make cuav_7-nano_default upload
```

**完整操作流程**：

1. 执行 `make <target>` 编译固件
2. 按 [9.4 节](#94-进入-bootloader-模式how-to-enter-bootloader) 的方法让飞控进入 Bootloader 模式
3. 执行 `make <target> upload` 上传固件
4. 终端显示 `Upload finished` 后，飞控自动重启
5. 等待飞控启动完成（LED 变为正常闪烁模式）

#### 方式二：通过 QGroundControl 烧录

1. 打开 QGroundControl
2. 连接飞控 USB（如飞控已正常运行，QGC 会自动检测并弹出固件更新页）
3. 如 QGC 未自动弹出，进入 `设置` (齿轮图标) :arrow_right: `固件` (Firmware)
4. 在固件页面底部的 **"高级设置"** 中：
   - 勾选 **"自定义固件文件"**
   - 点击浏览，选择 `build/<target>/<target>.px4` 文件
5. 点击 **"确定"** 开始烧录
6. 等待进度条完成，飞控会自动重启

> :bulb: **QGC 方式的优点**: 不需要手动进入 Bootloader，QGC 会通过 MAVLink 命令自动让飞控重启到 Bootloader。

#### 方式三：通过 SD 卡烧录（适用于无法 USB 连接的场景）

```bash
# 1. 将 SD 卡插入电脑读卡器
# 2. 将固件复制到 SD 卡根目录并重命名为 firmware.px4
cp build/cuav_7-nano_default/cuav_7-nano_default.px4 /media/$USER/<SD_CARD_NAME>/firmware.px4

# 3. 将 SD 卡插入飞控
# 4. 上电，飞控会自动检测并烧录
```

> :warning: **注意**: 部分飞控需要在 SD 卡上创建 `/etc/extras.txt` 文件并写入 `set +update` 来启用自动固件更新。

#### 烧录失败的常见原因与解决方案

| 现象 | 原因 | 解决方案 |
|------|------|----------|
| `make upload` 报 `No device found` | 飞控未进入 Bootloader | 检查 USB 连接，重新按方法进入 Bootloader |
| 烧录到一半中断 | USB 线松动或供电不足 | 更换短而粗的 USB 数据线，避免用 USB Hub |
| QGC 找不到飞控 | 飞控固件损坏无法启动 | 使用**方法一**（按钮法）强制进入 Bootloader |
| `Permission denied` | Linux 没有 USB 设备权限 | `sudo usermod -a -G dialout $USER` 后重新登录 |
| 固件上传成功但飞控不启动 | 固件与硬件不匹配 | 确认 build target 正确，检查 `default.px4board` 配置 |

### 9.6 验证固件版本

烧录完成后，通过 MAVLink Console 验证 LOCP 功能是否在固件中：

```bash
# 连接到飞控 MAVLink Shell
./Tools/mavlink_shell.py

# 查看 LOCP 参数
param show LOCP_ARD_EN
param show LOCP_VRD_EN
param show LOCP_PRD_EN
param show LOCP_COD_EN
param show LOCP_MTO_EN
param show LOCP_OBS_EN
param show LOCP_L1_ACT
param show LOCP_L2_ACT
param show LOCP_L3_ACT
param show LOCP_CRASH_THR

# 如果以上参数都存在，说明 LOCP 已正确编译到固件中
```

在 QGroundControl 中:
- 进入 `参数` 页面
- 在搜索框中输入 `LOCP`
- 应看到所有 LOCP 参数（约 42 个，含 OBS）

---

## 10. SITL 仿真测试指南

在实际飞行前，强烈建议先在 SITL 仿真环境中测试 LOCP 功能。

### 10.1 启动 SITL 仿真

```bash
cd ~/PX4-Autopilot

# 启动 jMAVSim 四旋翼仿真
make px4_sitl_default jmavsim

# 或使用 Gazebo-Classic
make px4_sitl_default sitl_gazebo-classic
```

### 10.2 通过故障注入测试 LOCP

LOCP 的故障注入通过 MAVLink 命令 `MAV_CMD_INJECT_FAILURE` 实现：

```bash
# 在新终端中打开 MAVLink Shell
cd ~/PX4-Autopilot
./Tools/mavlink_shell.py
```

**测试 ARD（姿态变化率）**:

由于 SITL 中加速度计数据是仿真生成的，姿态变化率检测的测试可以通过以下方式：

```bash
# 在 QGC 中，使用 Analyze → MAVLink Console 发送命令
# 模拟电机故障以触发 ARD（电机不同步会导致姿态晃动）

# 方法1: 注入电机故障
failure inject --unit=motor --type=off --instance=1

# 方法2: 直接在 MAVLink Shell 中设置参数降低阈值
param set LOCP_ARD_T 0.1          # 缩短迟滞时间
param set LOCP_ARD_R_MAX 5.0      # 降低角加速度阈值
param set LOCP_ARD_RSP 1.0        # 降低持续角速率阈值
param set LOCP_ARD_DUR 0.1        # 缩短持续时间阈值
```

然后通过遥控器（或 QGC 虚拟摇杆）进行剧烈的姿态操作来触发。

**测试 Crash 碰撞检测**:

```bash
# 降低碰撞阈值以方便测试
param set LOCP_CRASH_THR 5.0   # 降低到 5 m/s² (~0.5g)

# 执行着陆或撞击地面
commander land
```

**查看触发状态**:

```bash
# 在 MAVLink Shell 中监听 failsafe_flags
listener failsafe_flags
```

### 10.3 验证响应行为

| 测试场景 | 预期行为 |
|----------|----------|
| 仅降低单个维度阈值 | LOCP LEVEL_1 触发 → 执行 `LOCP_L1_ACT` (默认 Land) |
| 降低两个维度阈值 | LOCP LEVEL_2 触发 → 执行 `LOCP_L2_ACT` (默认 Disarm) |
| 降低三个维度阈值 | LOCP LEVEL_3 触发 → 执行 `LOCP_L3_ACT` (默认 Disarm) |
| 碰撞阈值设为很低 + 着陆 | `crash_detected` 触发 → 立即 Disarm |
| 关闭 MAVLink 心跳 | MTO 心跳丢失触发 → LEVEL_2 (MTO+其他) |
| 恢复正常 | 动作根据 ClearCondition 清除 |

### 10.4 SITL 中的传感器模拟限制

> ⚠️ **SITL 的限制**: SITL 中的传感器数据是理想化的模拟数据，不能完全反映真实硬件的噪声和漂移特性。LOCP 的阈值在 SITL 中可能过于敏感或过于迟钝。**实飞前务必根据真实飞行日志重新调参**。

---

## 11. 实飞验证与调参建议

### 11.1 首飞建议配置

首次飞行时，建议采用 **保守配置** —— 仅启用警告，不启用自动动作。 这样可以先验证检测准确性，再逐步放开自动响应。

```bash
# === 保守配置 (仅警告) ===
param set LOCP_L1_ACT 1    # LEVEL_1: 仅警告
param set LOCP_L2_ACT 1    # LEVEL_2: 仅警告
param set LOCP_L3_ACT 1    # LEVEL_3: 仅警告
# 碰撞检测保持上锁（安全底线）
```

### 11.2 飞行后日志分析

每次飞行后分析 LOCP 相关日志：

1. 下载飞行日志（通过 QGC 或 SD 卡）
2. 使用 [Flight Review](https://logs.px4.io/) 或 PlotJuggler 打开日志
3. 搜索以下字段并绘图:
   - `failsafe_flags.locp_severity`
   - `failsafe_flags.locp_ard_triggered`
   - `failsafe_flags.locp_vrd_triggered`
   - `failsafe_flags.locp_prd_triggered`
   - `failsafe_flags.locp_cod_triggered`
   - `failsafe_flags.locp_mto_triggered`
   - `failsafe_flags.crash_detected`

4. 与飞行视频/时间线对比，验证是否有 **误报** (false positive) 或 **漏报** (false negative)

### 11.3 调参策略

| 问题 | 症状 | 解决方案 |
|------|------|----------|
| **误报 (false positive)** | 正常飞行中 LOCP 触发 | 增大迟滞时间 `LOCP_*_T` 或增大对应阈值 |
| **漏报 (false negative)** | 实际失控但未触发 | 减小阈值或缩短迟滞时间 |
| **特定维度频繁误报** | 如 ARD 在高速机动时频繁触发 | 增大 `LOCP_ARD_R_MAX` 或将 `LOCP_L1_ACT` 设为 Warn |
| **碰撞检测误报** | 着陆时误触发 crash_detected | 增大 `LOCP_CRASH_THR`（默认 50 → 70 m/s²） |
| **MTO 通信波动误报** | 在信号边缘区 MTO 频繁触发 | 增大 `LOCP_MTO_HB_T`（默认 1.5 → 3 s） |

### 11.4 机型差异化调参

不同机型的正常飞行参数差异很大，需要针对机型调整默认值：

| 机类型 | 建议调整 |
|--------|----------|
| **小型竞速四旋翼** | 增大 ARD 阈值（角速度可达 20+ rad/s），增大 VRD 阈值 |
| **大型航拍六旋翼** | 默认值基本适用，可适当降低 PRD 阈值 |
| **固定翼** | 适当降低 ARD Yaw 阈值，增大 PRD 水平漂移阈值 |
| **VTOL** | 过渡期间自动禁用姿态检测，过渡后不调整也不需要额外配置 |
| **直升机** | 需要重新校准所有阈值（振动特性不同） |

---

## 12. 故障排查 FAQ

### Q1: 烧录后看不到 LOCP 参数？

**A**: 确认以下步骤：
1. 确保烧录的固件是从 `locp` 分支构建的
2. 执行 `param show LOCP_ARD_EN` ——如果返回 `parameter not found`，则固件未正确编译 LOCP
3. 检查 `boards/<target>/default.px4board` 中 `CONFIG_COMMANDER_MODULE=y` 是否存在（Commander 模块为必需）

### Q2: SITL 中 LOCP 没有触发？

**A**:
1. 确保飞机处于**已解锁**状态（`commander takeoff` 或 `commander arm`）
2. 降低对应维度的阈值到合理范围
3. 使用 `listener failsafe_flags` 确认标志位是否变化

### Q3: 如何临时禁用 LOCP？

**A**:
```bash
# 方法1: 将动作设为 None
param set LOCP_L1_ACT 0
param set LOCP_L2_ACT 0
param set LOCP_L3_ACT 0

# 方法2: 关闭各维度检测
param set LOCP_ARD_EN 0
param set LOCP_VRD_EN 0
param set LOCP_PRD_EN 0
param set LOCP_COD_EN 0
param set LOCP_MTO_EN 0
```

### Q4: 碰撞检测阈值应该设多少？

**A**: 默认 50 m/s²（~5g）适用于大多数四旋翼。建议：
- **小型竞速机**：70~80 m/s²（7~8g）—— 高速机动中加速度较大
- **大型航拍机**：40~50 m/s²（4~5g）—— 正常飞行加速度较小
- **固定翼着陆**：可适当提高阈值，避免着陆时误触发

### Q6: OBS 会在哪些情况下误触发？

**A**: 以下场景可能导致 OBS 误触发：
- **参数阈值设置过小**: `LOCP_OBS_JUMP_POS` 设为 1m，如果机载计算机正常发送的 setpoint 轨迹本身就包含大幅机动
- **首次收到 setpoint**: 第一帧 setpoint 的 `_sp_was_valid=false`，OBS 不做跳变检测（冷启动保护）
- **MAVROS 在位置和速度控制之间切换**: 如先发送位置+速度 setpoint，再改为仅发送速度 setpoint。 NaN 注入检测会根据 `PX4_ISFINITE` 做判断

### Q7: OBS 和现有的 `offboard_control_signal_lost` 有什么区别？

**A**:
| 方面 | `offboard_control_signal_lost` | OBS (locp_obs_triggered) |
|------|-------------------------------|--------------------------|
| 检测内容 | MAVLink 心跳超时 | trajectory_setpoint 数值异常 |
| 检测场景 | 机载计算机死机/断连/MAVROS崩溃 | 软件 bug 导致的坏值/跳变/NaN |
| 响应方式 | 模式回退 (Fallback/Land)| 直接 Disarm (不可接管) |
| 触发条件 | 500ms 无心跳 | 数值跳变 > 阈值 或 NaN 注入 |

### Q5: LOCP 触发后如何恢复？

**A**: 取决于触发的动作：
- `Action::None`: 无需恢复
- `Action::Warn`: 自动恢复（条件清除时）
- `Action::Hold`: 模式切换或上锁后清除
- `Action::Land/Descend/RTL`: 上锁后清除
- `Action::Disarm`: 上锁后清除（**不可用户接管**）
- `Action::Terminate`: 永不自动清除，需重启飞控

---

## 附录 A: 检测算法伪代码

### ARD 检测伪代码

```
function checkAttitudeRateAnomaly(ang_vel):
    dt = 0.01  // 假设100Hz更新频率

    // 角加速度 = 角速度差分
    d_roll  = |ang_vel.xyz[0] - rollspeed_prev| / dt
    d_pitch = |ang_vel.xyz[1] - pitchspeed_prev| / dt
    d_yaw   = |ang_vel.xyz[2] - yawspeed_prev| / dt

    rollspeed_prev  = ang_vel.xyz[0]
    pitchspeed_prev = ang_vel.xyz[1]
    yawspeed_prev   = ang_vel.xyz[2]

    // 角加速度超限
    accel_fault = d_roll > R_MAX OR d_pitch > P_MAX OR d_yaw > Y_MAX

    // 持续高角速率计时
    if |ang_vel.xyz[0]| > RSP OR |ang_vel.xyz[1]| > PSP OR |ang_vel.xyz[2]| > YSP:
        if att_rate_high_start == 0: att_rate_high_start = now
    else:
        att_rate_high_start = 0

    // 持续高角速率超限
    sustained_high = (|ang_vel.xyz[0]| > RSP OR |ang_vel.xyz[1]| > PSP)
                     AND (now - att_rate_high_start) > ARD_DUR

    // 任意条件满足即认为异常
    triggered = accel_fault OR sustained_high

    // 迟滞滤波
    ard_hysteresis.update(triggered, ARD_T, now)
    return ard_hysteresis.get_state()
```

### 严重等级仲裁伪代码

```
function evaluateLOCPSeverity():
    count = ARD + VRD + PRD + COD + MTO

    fatal_combo = COD AND ARD     // 致命组合：电流+姿态
    comm_dead   = MTO AND count >= 1  // 通信中断 + 其他异常

    if count >= 3 OR fatal_combo:
        return 3    // LEVEL_3: 终止飞行
    if count >= 2 OR comm_dead:
        return 2    // LEVEL_2: 紧急降落/上锁
    if count >= 1:
        return 1    // LEVEL_1: 降落
    return 0        // NONE: 正常
```

---

## 附录 B: 参数速查表

```
┌─────────────────────┬─────────┬──────────┬──────────────────────────────┐
│ 参数名               │ 默认值   │ 单位      │ 说明                         │
├─────────────────────┼─────────┼──────────┼──────────────────────────────┤
│ LOCP_ARD_EN         │ 1       │ —        │ ARD 使能                      │
│ LOCP_ARD_R_MAX      │ 80      │ rad/s²   │ Roll 角加速度阈值              │
│ LOCP_ARD_P_MAX      │ 80      │ rad/s²   │ Pitch 角加速度阈值             │
│ LOCP_ARD_Y_MAX      │ 60      │ rad/s²   │ Yaw 角加速度阈值               │
│ LOCP_ARD_T          │ 0.3     │ s        │ ARD 迟滞时间                   │
│ LOCP_ARD_RSP        │ 6.0     │ rad/s    │ Roll 持续角速率设定点           │
│ LOCP_ARD_PSP        │ 6.0     │ rad/s    │ Pitch 持续角速率设定点          │
│ LOCP_ARD_YSP        │ 5.0     │ rad/s    │ Yaw 持续角速率设定点            │
│ LOCP_ARD_DUR        │ 0.3     │ s        │ 持续角速率最小持续时间          │
│ LOCP_VRD_EN         │ 1       │ —        │ VRD 使能                      │
│ LOCP_VRD_AH_MAX     │ 8.0     │ m/s²     │ 水平加速度阈值                 │
│ LOCP_VRD_AD_MAX     │ 6.0     │ m/s²     │ 垂直向下加速度阈值              │
│ LOCP_VRD_VZD_MAX    │ 3.0     │ m/s      │ 垂直下降速度阈值               │
│ LOCP_VRD_JERK       │ 50.0    │ m/s³     │ Jerk 阈值                     │
│ LOCP_VRD_T          │ 0.3     │ s        │ VRD 迟滞时间                   │
│ LOCP_PRD_EN         │ 1       │ —        │ PRD 使能                      │
│ LOCP_PRD_VZ_MAX     │ 5.0     │ m/s      │ 急降垂直速度阈值               │
│ LOCP_PRD_ADROP      │ 3.0     │ m        │ 急降高度下降量阈值              │
│ LOCP_PRD_ASTD       │ 2.0     │ m        │ 高度振荡标准差阈值              │
│ LOCP_PRD_HSPD       │ 5.0     │ m/s      │ 水平漂移速度阈值               │
│ LOCP_PRD_T          │ 0.5     │ s        │ PRD 迟滞时间                   │
│ LOCP_COD_EN         │ 1       │ —        │ COD 使能                      │
│ LOCP_COD_DELTA_I    │ 10.0    │ A        │ 电流偏离均值阈值               │
│ LOCP_COD_MAX_I      │ 45.0    │ A        │ 总电流绝对阈值                 │
│ LOCP_COD_DI_DT      │ 30.0    │ A/s      │ dI/dt 阈值                    │
│ LOCP_COD_ESC_MAX    │ 10.0    │ A        │ 单路 ESC 电流阈值              │
│ LOCP_COD_T          │ 0.3     │ s        │ COD 迟滞时间                   │
│ LOCP_MTO_EN         │ 1       │ —        │ MTO 使能                      │
│ LOCP_MTO_HB_T       │ 1.5     │ s        │ 心跳超时阈值                   │
│ LOCP_MTO_CMD_T      │ 2.0     │ s        │ 指令超时阈值                   │
│ LOCP_MTO_RATE       │ 3.0     │ Hz       │ 消息最低速率阈值               │
│ LOCP_OBS_EN         │ 1       │ —        │ OBS 使能                      │
│ LOCP_OBS_JUMP_POS   │ 10.0    │ m        │ 位置跳变阈值                   │
│ LOCP_OBS_JUMP_VEL   │ 5.0     │ m/s      │ 速度跳变阈值                   │
│ LOCP_OBS_JUMP_YAW   │ 1.57    │ rad      │ Yaw 跳变阈值                   │
│ LOCP_OBS_T          │ 0.3     │ s        │ OBS 迟滞时间                   │
│ LOCP_OBS_ACT        │ 3       │ —        │ OBS 动作 (3=Land)             │
│ LOCP_CRASH_THR      │ 50.0    │ m/s²     │ 碰撞加速度阈值                 │
│ LOCP_L1_ACT         │ 3       │ —        │ LEVEL_1 动作 (3=Land)         │
│ LOCP_L2_ACT         │ 3       │ —        │ LEVEL_2 动作 (3=Land)         │
│ LOCP_L3_ACT         │ 7       │ —        │ LEVEL_3 动作 (7=Disarm)       │
└─────────────────────┴─────────┴──────────┴──────────────────────────────┘
```

---

> **文档维护者**: PX4 LOCP 开发团队
> **更新日期**: 2026-08-03
> **对应分支**: `locp`
> **问题反馈**: 请通过项目 Issue Tracker 提交 Bug 报告或功能建议
