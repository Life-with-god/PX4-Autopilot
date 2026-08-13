#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LOCP Threshold Calibration Tool —— LOCP 参数阈值标定工具
=========================================================

原理：先实际飞行（悬停 + 慢速机动 + 爬升下降），收集 .ulg 日志，
本工具统计各检测维度在【正常飞行】中的数据分布，据此设置 LOCP_* 阈值。

阈值设置三原则：
  1. 先飞后设：正常飞行一圈，统计真实数据（本工具）
  2. 留足裕度：阈值必须远离正常数据分布（不误报），又能覆盖故障（不漏报）
  3. 故障验证：用 FailureInjector 注入故障 / 模拟测试，确认能可靠触发

用法：
    python3 locp_threshold_calibration.py flight1.ulg [flight2.ulg ...]

依赖：
    pip install pyulog numpy

输出：Markdown 格式的统计报告，含每个 LOCP 参数的实测统计值与建议阈值。
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    print("[错误] 缺少依赖 pyulog，请先安装: pip install pyulog numpy")
    sys.exit(1)

# PX4 枚举
ARMING_STATE_ARMED = 2
NUM_MOTORS = 8          # esc_status 最多 8 路


# ---------------------------------------------------------------
# 通用工具
# ---------------------------------------------------------------
def get_dataset(ulog: ULog, name: str, instance: int = 0) -> Optional[ULog.Data]:
    """安全获取话题数据集，不存在则返回 None。"""
    try:
        return ulog.get_dataset(name, instance)
    except (KeyError, IndexError):
        return None


def percentile(arr: np.ndarray, p: float) -> float:
    """有限值百分位，空/全 NaN 返回 nan。"""
    a = arr[np.isfinite(arr)]
    if a.size == 0:
        return float('nan')
    return float(np.percentile(a, p))


def max_finite(arr: np.ndarray) -> float:
    a = arr[np.isfinite(arr)]
    return float(a.max()) if a.size else float('nan')


def min_finite(arr: np.ndarray) -> float:
    a = arr[np.isfinite(arr)]
    return float(a.min()) if a.size else float('nan')


def rate_limit(arr: np.ndarray, dt: np.ndarray, max_dt: float = 1.0, min_dt: float = 0.001) -> np.ndarray:
    """差分求变化率，过滤异常 dt。"""
    d = np.abs(np.diff(arr)) / dt
    d[~((dt > min_dt) & (dt < max_dt))] = np.nan
    return d


def flight_mask(ulog: ULog, t_ref: np.ndarray) -> np.ndarray:
    """将 vehicle_status 的解锁时段插值到参考时间戳，返回布尔掩码。"""
    vs = get_dataset(ulog, 'vehicle_status')
    if vs is None or 'arming_state' not in vs.data:
        return np.ones(t_ref.size, dtype=bool)
    t_vs = vs.data['timestamp'] * 1e-6
    armed = vs.data['arming_state'] == ARMING_STATE_ARMED
    return np.interp(t_ref, t_vs, armed.astype(float)) > 0.5


# ---------------------------------------------------------------
# 各检测维度分析
# ---------------------------------------------------------------
def analyze_cod(ulog: ULog, t_ref: np.ndarray) -> Optional[dict]:
    """COD 电流异常：MAX_I / DELTA_I / DI_DT 标定参考"""
    bat = get_dataset(ulog, 'battery_status')
    if bat is None or 'current_a' not in bat.data:
        return None
    t_bat = bat.data['timestamp'] * 1e-6
    cur = bat.data['current_a']
    m = flight_mask(ulog, t_bat)
    cur_m = cur[m]
    if cur_m.size == 0:
        return None
    dt = np.diff(t_bat[m])
    di_dt = rate_limit(cur[m], dt)
    return {
        'i_mean': float(np.mean(cur_m)), 'i_p95': percentile(cur_m, 95),
        'i_max': max_finite(cur_m), 'di_dt_max': max_finite(di_dt),
    }


def analyze_ard(ulog: ULog, t_ref: np.ndarray) -> Optional[dict]:
    """ARD 姿态变化率：R_MAX/P_MAX/Y_MAX / RSP/PSP/YSP 标定参考"""
    ang = get_dataset(ulog, 'vehicle_angular_velocity')
    if ang is None or 'xyz[0]' not in ang.data:
        return None
    t_a = ang.data['timestamp'] * 1e-6
    m = flight_mask(ulog, t_a)
    rate_max = [max_finite(np.abs(ang.data[f'xyz[{i}]'])[m]) for i in range(3)]
    dt = np.diff(t_a[m])
    accel_max = []
    for i in range(3):
        d = rate_limit(ang.data[f'xyz[{i}]'][m], dt)
        accel_max.append(max_finite(d))
    return {'rate_max': rate_max, 'accel_max': accel_max}


def analyze_vrd_prd(ulog: ULog, t_ref: np.ndarray) -> Optional[dict]:
    """VRD/PRD 速度位置变化率：AH_MAX/VZD_MAX / VZ_MAX/HSPD 标定参考"""
    loc = get_dataset(ulog, 'vehicle_local_position')
    if loc is None:
        return None
    t_l = loc.data['timestamp'] * 1e-6
    m = flight_mask(ulog, t_l)
    if m.sum() == 0:
        return None
    ah = np.hypot(loc.data['ax'][m], loc.data['ay'][m])
    az = loc.data['az'][m]
    vz = loc.data['vz'][m]
    vh = np.hypot(loc.data['vx'][m], loc.data['vy'][m])
    return {
        'ah_max': max_finite(ah), 'az_max': max_finite(az), 'az_min': min_finite(az),
        'vz_max': max_finite(vz), 'vz_min': min_finite(vz), 'vh_max': max_finite(vh),
    }


def analyze_mto(ulog: ULog, t_ref: np.ndarray) -> Optional[dict]:
    """MTO MAVLink 超时：HB_T 标定参考"""
    tel = get_dataset(ulog, 'telemetry_status')
    if tel is None:
        return None
    t = tel.data['timestamp'] * 1e-6
    if t.size < 2:
        return None
    interval = np.diff(t)
    return {
        'hb_interval_mean': float(np.mean(interval)),
        'hb_interval_max': max_finite(interval),
        'hb_rate_min': float(1.0 / np.max(interval)) if np.max(interval) > 0 else float('nan'),
    }


# ---------------------------------------------------------------
# 报告输出
# ---------------------------------------------------------------
def fmt(v: float, suffix: str = '') -> str:
    return f"{v:.1f}{suffix}" if np.isfinite(v) else "—(无数据)"


def print_report(ulog: ULog, path: str) -> None:
    print(f"\n{'=' * 72}")
    print(f" LOCP 阈值标定报告 —— {path}")
    print(f"{'=' * 72}")

    # 日志基本信息
    t_start = ulog.start_timestamp * 1e-6
    t_end = ulog.last_timestamp * 1e-6
    print(f" 日志时长: {t_end - t_start:.1f} s")

    # ---------------- COD ----------------
    r = analyze_cod(ulog, None)
    if r:
        print(f"\n【COD 电流异常】数据源: battery_status.current_a")
        print(f"  电流: 均值={fmt(r['i_mean'], 'A')}  P95={fmt(r['i_p95'], 'A')}  最大={fmt(r['i_max'], 'A')}")
        print(f"  → 建议 LOCP_COD_MAX_I ≈ {1.2 * r['i_max']:.1f}~{1.3 * r['i_max']:.1f}"
              f"  (高于正常最大电流 20~30%)")
        print(f"  最大 dI/dt: {fmt(r['di_dt_max'], 'A/s')}")
        if np.isfinite(r['di_dt_max']):
            print(f"  → 建议 LOCP_COD_DI_DT ≈ {2.0 * r['di_dt_max']:.0f}~{3.0 * r['di_dt_max']:.0f}"
                  f"  (高于正常尖峰 2~3 倍)")
    else:
        print(f"\n【COD】无 battery_status 数据")

    # ---------------- ARD ----------------
    r = analyze_ard(ulog, None)
    if r:
        print(f"\n【ARD 姿态变化率】数据源: vehicle_angular_velocity")
        axes = ['Roll', 'Pitch', 'Yaw']
        for i in range(3):
            print(f"  {axes[i]}: 最大角速率={fmt(r['rate_max'][i], 'rad/s')}  "
                  f"最大角加速度={fmt(r['accel_max'][i], 'rad/s²')}")
            if np.isfinite(r['rate_max'][i]):
                print(f"    → 建议 {['LOCP_ARD_RSP', 'LOCP_ARD_PSP', 'LOCP_ARD_YSP'][i]} ≈ "
                      f"{1.5 * r['rate_max'][i]:.1f}~{2.0 * r['rate_max'][i]:.1f}  (1.5~2 倍)")
            if np.isfinite(r['accel_max'][i]):
                print(f"    → 建议 {['LOCP_ARD_R_MAX', 'LOCP_ARD_P_MAX', 'LOCP_ARD_Y_MAX'][i]} ≈ "
                      f"{1.5 * r['accel_max'][i]:.1f}~{2.0 * r['accel_max'][i]:.1f}  (1.5~2 倍)")
    else:
        print(f"\n【ARD】无 vehicle_angular_velocity 数据")

    # ---------------- VRD / PRD ----------------
    r = analyze_vrd_prd(ulog, None)
    if r:
        print(f"\n【VRD 速度变化率】数据源: vehicle_local_position")
        print(f"  水平加速度最大: {fmt(r['ah_max'], 'm/s²')}")
        print(f"  垂直加速度最大: {fmt(r['az_max'], 'm/s²')}  最小: {fmt(r['az_min'], 'm/s²')}")
        print(f"  → 建议 LOCP_VRD_AH_MAX ≈ {1.5 * r['ah_max']:.1f}~{2.0 * r['ah_max']:.1f}  (1.5~2 倍)")
        print(f"\n【PRD 位置变化率】数据源: vehicle_local_position")
        print(f"  垂直速度最大: {fmt(r['vz_max'], 'm/s')}  最小: {fmt(r['vz_min'], 'm/s')}")
        print(f"  → 建议 LOCP_PRD_VZ_MAX / LOCP_VRD_VZD_MAX ≈ {1.5 * r['vz_max']:.1f}~{2.0 * r['vz_max']:.1f}")
        print(f"  水平速度最大: {fmt(r['vh_max'], 'm/s')}")
        print(f"  → 建议 LOCP_PRD_HSPD ≈ {1.5 * r['vh_max']:.1f}~{2.0 * r['vh_max']:.1f}  (1.5~2 倍)")
    else:
        print(f"\n【VRD/PRD】无 vehicle_local_position 数据")

    # ---------------- MTO ----------------
    r = analyze_mto(ulog, None)
    if r:
        print(f"\n【MTO MAVLink 超时】数据源: telemetry_status")
        print(f"  心跳间隔: 均值={fmt(r['hb_interval_mean'], 's')}  最大={fmt(r['hb_interval_max'], 's')}")
        print(f"  → 建议 LOCP_MTO_HB_T ≈ {2.0 * r['hb_interval_mean']:.1f}~{3.0 * r['hb_interval_mean']:.1f}"
              f"  (正常心跳间隔 2~3 倍，且 > 最大间隔)")
    else:
        print(f"\n【MTO】无 telemetry_status 数据")

    print(f"\n{'=' * 72}")
    print(" 说明: 建议值仅供参考，请结合机型动力学、ESC 规格、安全要求综合判断。")
    print("       设定后务必用故障注入/模拟测试验证不漏报，再实飞确认不误报。")
    print(f"{'=' * 72}\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description='LOCP 参数阈值标定工具：统计正常飞行日志，输出建议阈值。')
    parser.add_argument('logs', nargs='+', help='.ulg 飞行日志文件路径（可多个，会合并统计）')
    args = parser.parse_args()

    for path in args.logs:
        try:
            ulog = ULog(path)
        except Exception as e:  # noqa: BLE001
            print(f"[错误] 无法解析 {path}: {e}", file=sys.stderr)
            continue
        print_report(ulog, path)


if __name__ == '__main__':
    main()
