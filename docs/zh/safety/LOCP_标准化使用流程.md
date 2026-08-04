

 第一部分：参数说明

1.1 检测维度与严重等级

LOCP 通过 6 个独立维度检测失控：
───────────────────────────────────────────────────────────────────────────────
 维度     含义                                              数据源
───────────────────────────────────────────────────────────────────────────────
 ARD      姿态变化率异常（角加速度/持续高角速率）           IMU 角速度
 VRD      速度变化率异常（加速度/jerk/自由落体）            EKF local_position
 PRD      位置变化率异常（急降/高度振荡/漂移）              EKF local_position
 COD      电流异常（总电流突增/dI/dt/ESC过流）              battery_status + esc_status
 MTO      MAVLink 消息超时（心跳/指令/速率）                telemetry_status
 OBS      Offboard Setpoint 异常（跳变/NaN注入）            trajectory_setpoint
 Crash    碰撞/撞击检测（IMU加速度尖峰）                   vehicle_acceleration
───────────────────────────────────────────────────────────────────────────────

严重等级= 触发维度数 + 数据源折扣：
───────────────────────────────────────────────────────────────────────────────────
 等级      条件                                            默认动作       配置参数
───────────────────────────────────────────────────────────────────────────────────
 1         1 维触发（VRD+PRD 同时触发折扣为 1 维）          Land 降落      LOCP_L1_ACT=3
 2         2 维触发 / MTO+任一其他异常                      Land 降落      LOCP_L2_ACT=3
 3         3+ 维触发 / COD+ARD 致命组合                     Disarm 上锁    LOCP_L3_ACT=7
 OBS       Setpoint 跳变/NaN注入                            Land 降落      LOCP_OBS_ACT=3
 Crash     加速度范数 > 阈值                                Disarm 上锁    硬编码
───────────────────────────────────────────────────────────────────────────────────
1.2 参数表
───────────────────────────────────────────────────────────────────────────────────────────────────
 维度    参数                      含义                                默认值   2.7kg推荐   范围
───────────────────────────────────────────────────────────────────────────────────────────────────
 ARD    LOCP_ARD_EN               使能开关                               1         1       0-1
 ARD    LOCP_ARD_R_MAX            Roll 角加速度上限                     80        80       20-500 rad/s²
 ARD    LOCP_ARD_P_MAX            Pitch 角加速度上限                    80        80       20-500 rad/s²
 ARD    LOCP_ARD_Y_MAX            Yaw 角加速度上限                      60        60       20-500 rad/s²
 ARD    LOCP_ARD_RSP              Roll 持续高角速率设定点                6         6       3-30 rad/s
 ARD    LOCP_ARD_PSP              Pitch 持续高角速率设定点               6         6       3-30 rad/s
 ARD    LOCP_ARD_YSP              Yaw 持续高角速率设定点                 5         5       3-30 rad/s
 ARD    LOCP_ARD_DUR              持续高角速率最短时间                  0.3       0.3      0.1-2.0 s
 ARD    LOCP_ARD_T                迟滞确认时间                          0.3       0.3      0.1-2.0 s

 VRD    LOCP_VRD_EN               使能开关                               1         1       0-1
 VRD    LOCP_VRD_AH_MAX           水平加速度上限                         8         8       5-50 m/s²
 VRD    LOCP_VRD_AD_MAX           垂直向下加速度阈值                     6         6       3-20 m/s²
 VRD    LOCP_VRD_VZD_MAX          垂直下降速度阈值                       3         3       1-15 m/s
 VRD    LOCP_VRD_JERK             水平 Jerk 阈值                        50        50       10-200 m/s³
 VRD    LOCP_VRD_T                迟滞确认时间                          0.3       0.3      0.1-2.0 s

 PRD    LOCP_PRD_EN               使能开关                               1         1       0-1
 PRD    LOCP_PRD_VZ_MAX           下降速度上限                           5         5       1-20 m/s
 PRD    LOCP_PRD_ADROP            高度下降量上限                         3         3       1-50 m
 PRD    LOCP_PRD_ASTD             高度标准差上限                         2         2       0.5-10 m
 PRD    LOCP_PRD_HSPD             水平漂移速度上限                       5         5       1-30 m/s
 PRD    LOCP_PRD_T                迟滞确认时间                          0.5       0.5      0.1-2.0 s

 COD    LOCP_COD_EN               使能开关                               1         1       0-1
        ⚠ 无电流传感器须设为 0
 COD    LOCP_COD_MAX_I            总电流绝对上限                        45       **40**    10-150 A
 COD    LOCP_COD_DELTA_I          电流偏离滑动均值阈值                  10       **8**     5-40 A
 COD    LOCP_COD_DI_DT            电流变化率 dI/dt 上限                 30        30       10-200 A/s
 COD    LOCP_COD_ESC_MAX          单路 ESC 电流上限                     10       **12**    5-50 A
 COD    LOCP_COD_T                迟滞确认时间                          0.3       0.3      0.1-2.0 s

 MTO    LOCP_MTO_EN               使能开关                               1         1       0-1
 MTO    LOCP_MTO_HB_T             心跳超时阈值                          1.5       1.5      0.5-10 s
 MTO    LOCP_MTO_CMD_T            指令超时阈值                          2.0       2.0      0.5-10 s
 MTO    LOCP_MTO_RATE             消息最低速率                           3         3       0.5-50 Hz

 OBS    LOCP_OBS_EN               使能开关                               1         1       0-1
 OBS    LOCP_OBS_JUMP_POS         位置跳变阈值                          10        10       1-50 m
 OBS    LOCP_OBS_JUMP_VEL         速度跳变阈值                           5         5       1-20 m/s
 OBS    LOCP_OBS_JUMP_YAW         Yaw 跳变阈值                         1.57      1.57      0.5-6.28 rad
 OBS    LOCP_OBS_T                迟滞确认时间                          0.3       0.3      0.1-2.0 s

 Crash  LOCP_CRASH_THR            碰撞加速度范数阈值                    50        50       30-200 m/s²

 动作   LOCP_L1_ACT               LEVEL_1 动作                          3         3       0-7
 动作   LOCP_L2_ACT               LEVEL_2 动作                          3         3       0-7
 动作   LOCP_L3_ACT               LEVEL_3 动作                          7         7       0-7
 动作   LOCP_OBS_ACT              OBS 触发动作                          3         3       0-7
───────────────────────────────────────────────────────────────────────────────────────────────────



第二部分：使用方法
2.1 编译固件

cd ~/PX4-Autopilot

# 确认在 locp 分支
git branch
# 应显示: * locp

# 编译目标: CUAV 7-Nano
make cuav_7-nano_default

编译产物位于 `build/cuav_7-nano_default/cuav_7-nano_default.px4`。

### 2.2 烧录固件

#### 什么是 Bootloader 模式

飞控正常运行时运行的是 PX4 固件。烧录新固件时需要让飞控进入 **Bootloader（引导加载程序）模式**——此时飞控停止运行固件，只等待接收新的固件文件。Bootloader 是出厂预烧在芯片中的一小段程序，不会被用户固件覆盖。

#### CUAV 7-Nano 进入 Bootloader 的方法

CUAV 7-Nano 使用 STM32H7 芯片，内置 DFU（Device Firmware Upgrade）Bootloader。

**方法一：QGC 自动进入（推荐）**

1. USB Type-C 连接飞控到电脑
2. 打开 QGroundControl
3. 点击顶部 **齿轮图标 → Firmware**
4. QGC 会自动让飞控重启进入 Bootloader 模式
5. 选择"高级设置" → "自定义固件文件" → 选择 `.px4` 文件
6. 等待烧录完成，飞控自动重启

**方法二：按住 BOOT 键上电**

1. 断开飞控 USB 电源
2. 按住飞控上的 **BOOT 按键**（位于电路板上，标有 BOOT0）
3. 保持按住 BOOT 键，插入 USB 连接电脑
4. 等待 2-3 秒后松开 BOOT 键
5. 此时飞控进入 DFU 模式（LED 不亮或保持常亮，与正常启动不同）
6. 执行烧录命令：

```bash
# 确认设备已识别:
ls /dev/ttyACM*      # 通常显示 /dev/ttyACM0

# 烧录:
python3 Tools/upload.py --port /dev/ttyACM0 build/cuav_7-nano_default/cuav_7-nano_default.px4
```

#### 烧录完成后的验证

烧录成功后飞控自动重启。在 MAVLink Shell 中验证：

```bash
ver
# 应显示分支信息包含 "locp"

param show LOCP_ARD_EN
# 确认 LOCP 参数存在
```

### 2.3 在 QGC 中修改参数

1. 飞控上电 → QGC 连接
2. 顶部菜单 **齿轮图标 → Parameters**（参数）
3. 搜索框输入 `LOCP` → 显示全部 LOCP 参数
4. 点击参数值修改 → 自动保存到飞控 RAM
5. 修改完毕后，点击 **Tools → 全部保存到闪存**（或执行 `param save`）

```bash
# 也可在 MAVLink Shell 中逐条设置:
param set LOCP_COD_MAX_I 40
param set LOCP_COD_ESC_MAX 12
param set LOCP_COD_DELTA_I 8
param save
```

### 2.4 日常使用

#### 解锁前检查

```bash
# MAVLink Shell 中确认 LOCP 状态:
listener failsafe_flags
# 全部 locp_xxx_triggered=0, locp_severity=0
```

#### 飞行中查看

QGC → MAVLink Inspector → 搜索 `failsafe_flags` → 观察 `locp_severity` 是否为 0。

#### 需要调参的情况

| 现象 | 调整 |
|------|------|
| 正常飞行中 LOCP 误触发 | 提高对应维度的阈值 |
| 正常着陆触发 Crash | 提高 `LOCP_CRASH_THR` |
| 推油门电流尖峰触发 COD | 提高 `LOCP_COD_DI_DT` |
| 数传偶发丢包触发 MTO | 提高 `LOCP_MTO_HB_T` |

#### 临时禁用单个维度

```bash
param set LOCP_xxx_EN 0    # 将该维度关闭
```

#### 查看飞行后触发情况

导出 SD 卡 `.ulg` 日志 → https://review.px4.io/ → 搜索 `failsafe_flags` 话题。

### 2.5 一键参数脚本

将以下内容保存为 `locp_setup.txt`，在 MAVLink Shell 中粘贴执行：

```bash
# === ARD ===
param set LOCP_ARD_EN 1 ; param set LOCP_ARD_R_MAX 80 ; param set LOCP_ARD_P_MAX 80
param set LOCP_ARD_Y_MAX 60 ; param set LOCP_ARD_RSP 6 ; param set LOCP_ARD_PSP 6
param set LOCP_ARD_YSP 5 ; param set LOCP_ARD_DUR 0.3 ; param set LOCP_ARD_T 0.3

# === VRD ===
param set LOCP_VRD_EN 1 ; param set LOCP_VRD_AH_MAX 8 ; param set LOCP_VRD_AD_MAX 6
param set LOCP_VRD_VZD_MAX 3 ; param set LOCP_VRD_JERK 50 ; param set LOCP_VRD_T 0.3

# === PRD ===
param set LOCP_PRD_EN 1 ; param set LOCP_PRD_VZ_MAX 5 ; param set LOCP_PRD_ADROP 3
param set LOCP_PRD_ASTD 2 ; param set LOCP_PRD_HSPD 5 ; param set LOCP_PRD_T 0.5

# === COD ===
param set LOCP_COD_EN 1 ; param set LOCP_COD_MAX_I 40 ; param set LOCP_COD_DELTA_I 8
param set LOCP_COD_DI_DT 30 ; param set LOCP_COD_ESC_MAX 12 ; param set LOCP_COD_T 0.3

# === MTO ===
param set LOCP_MTO_EN 1 ; param set LOCP_MTO_HB_T 1.5 ; param set LOCP_MTO_CMD_T 2.0
param set LOCP_MTO_RATE 3

# === OBS ===
param set LOCP_OBS_EN 1 ; param set LOCP_OBS_JUMP_POS 10 ; param set LOCP_OBS_JUMP_VEL 5
param set LOCP_OBS_JUMP_YAW 1.57 ; param set LOCP_OBS_T 0.3

# === Crash & 动作 ===
param set LOCP_CRASH_THR 50 ; param set LOCP_L1_ACT 3 ; param set LOCP_L2_ACT 3
param set LOCP_L3_ACT 7 ; param set LOCP_OBS_ACT 3

param save
```
