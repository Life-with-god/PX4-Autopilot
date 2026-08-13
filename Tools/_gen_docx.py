# -*- coding: utf-8 -*-
"""
生成 LOCP 参数说明 Word 文档
解析 src/modules/commander/failure_detector/failure_detector_params.c
提取所有 LOCP_ 参数，按检测维度分组，生成 .docx
"""
import os
import re
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                   'src', 'modules', 'commander', 'failure_detector',
                   'failure_detector_params.c')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                   'firmware', 'LOCP参数说明_v3.docx')

# 维度分组（按参数名前缀识别）
GROUPS = [
    ('总开关',            ['LOCP_EN']),
    ('ARD 姿态变化率',     ['LOCP_ARD_']),
    ('VRD 速度变化率',     ['LOCP_VRD_']),
    ('PRD 位置变化率',     ['LOCP_PRD_']),
    ('COD 电流异常',       ['LOCP_COD_']),
    ('MTO 通信超时',       ['LOCP_MTO_']),
    ('TRD 动力响应',       ['LOCP_TRD_']),
    ('OBS Offboard 异常',  ['LOCP_OBS_']),
    ('Crash 碰撞',         ['LOCP_CRASH_']),
]

def parse_params(path):
    """解析 params.c，返回 [(name, type, default, min, max, unit, desc), ...]"""
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()

    params = []
    # 按 PARAM_DEFINE_XXX(NAME, default); 分割，每个参数块前面是注释
    pattern = re.compile(
        r'/\*\*(.*?)\*/\s*PARAM_DEFINE_(FLOAT|INT32|BOOL)\(([A-Z0-9_]+),\s*([^)]+)\);',
        re.S)
    for m in pattern.finditer(text):
        comment, ptype, name, default = m.groups()
        # 只提取 LOCP 参数
        if not name.startswith('LOCP_'):
            continue

        # 从注释提取 @min/@max/@unit 和描述
        min_v = max_v = unit = ''
        desc_lines = []
        for line in comment.splitlines():
            line = line.strip().lstrip('*').strip()
            mm = re.match(r'@min\s+([\d.]+)', line)
            if mm:
                min_v = mm.group(1)
                continue
            mm = re.match(r'@max\s+([\d.]+)', line)
            if mm:
                max_v = mm.group(1)
                continue
            mm = re.match(r'@unit\s+(.+)', line)
            if mm:
                unit = mm.group(1).strip()
                continue
            if line:
                desc_lines.append(line)

        if ptype == 'FLOAT':
            type_str = 'FLOAT'
            default_str = default.strip().rstrip('f')
        elif ptype == 'BOOL':
            type_str = 'BOOL'
            default_str = default.strip()
        else:
            type_str = 'INT32'
            default_str = default.strip()

        params.append({
            'name': name,
            'type': type_str,
            'default': default_str,
            'min': min_v,
            'max': max_v,
            'unit': unit,
            'desc': ' '.join(desc_lines),
        })
    return params


def group_params(params):
    """按前缀分组，返回 [(group_name, [params...]), ...]"""
    result = []
    used = set()
    for gname, prefixes in GROUPS:
        items = [p for p in params if p['name'] not in used
                 and any(p['name'].startswith(pre) or p['name'] == pre for pre in prefixes)]
        used.update(p['name'] for p in items)
        result.append((gname, items))
    # 未分组的参数
    rest = [p for p in params if p['name'] not in used]
    if rest:
        result.append(('其他', rest))
    return result


def set_cell(cell, text, bold=False, size=9):
    cell.text = ''
    p = cell.paragraphs[0]
    run = p.add_run(str(text))
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.name = '微软雅黑'
    run._element.rPr.rFonts.set(
        '{%s}eastAsia' % 'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
        '微软雅黑')


def main():
    params = parse_params(SRC)
    print(f"共解析 {len(params)} 个 LOCP 参数")

    doc = Document()
    # 标题
    title = doc.add_heading('PX4 LOCP 失控保护系统参数说明', 0)
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    sub = doc.add_paragraph()
    sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = sub.add_run('cuav_7-nano 定制固件 | 2026-08-13 标定 | 共 %d 个参数' % len(params))
    r.font.size = Pt(10)
    r.font.color.rgb = RGBColor(0x66, 0x66, 0x66)

    doc.add_paragraph(
        '本系统（LOCP, Loss-of-Control Protection）在无遥控器/地面站失效时，'
        '通过多维度传感器融合自动检测失控并触发保护动作。所有使能开关默认 = 0（禁用），'
        '需按需开启。动作分配（固定，无等级参数）：\n'
        '  · 停桨 Disarm：ARD / VRD / PRD / COD / Crash（飞机自身已失控，控制不可信）\n'
        '  · 降落 Land：MTO / OBS，且须通过飞机自身健康检查（checkVehicleHealthy）\n'
        '    —— 健康→降落；不健康→停桨（飞机自身正常才可安全受控落地）\n'
        '  · TRD 分级：低高度降落（含接管窗口），高高度/失联停桨')

    # ============ 一、检测维度概览 ============
    doc.add_heading('一、检测维度概览', level=1)
    groups = group_params(params)
    table = doc.add_table(rows=1, cols=4)
    table.style = 'Light Grid Accent 1'
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr = table.rows[0].cells
    for i, t in enumerate(['检测维度', '检测内容', '使能开关', '默认']):
        set_cell(hdr[i], t, bold=True)

    overview = [
        ('总开关', 'LOCP 整体使能', 'LOCP_EN', '0 (禁用)'),
        ('ARD 姿态变化率', '角加速度尖峰 / 持续高角速率（乱飞）', 'LOCP_ARD_EN', '0'),
        ('VRD 速度变化率', '水平加速度 / 自由落体 / jerk（EGO故障）', 'LOCP_VRD_EN', '0'),
        ('PRD 位置变化率', '急降 / 高度振荡 / 水平漂移', 'LOCP_PRD_EN', '0'),
        ('COD 电流异常', '总电流突增 / dI·dt 尖峰', 'LOCP_COD_EN', '0'),
        ('MTO 通信超时', '心跳 / 指令超时 / 消息速率骤降', 'LOCP_MTO_EN', '0'),
        ('TRD 动力响应', '高油门但垂直加速度不足（卡网/动力丢失）', 'LOCP_TRD_EN', '0'),
        ('OBS Offboard', 'Setpoint 跳变 / NaN 注入', 'LOCP_OBS_EN', '0'),
        ('Crash 碰撞', 'IMU 加速度范数尖峰（卡网碰撞）', 'LOCP_CRASH_EN', '0'),
    ]
    for row in overview:
        cells = table.add_row().cells
        for i, t in enumerate(row):
            set_cell(cells[i], t)

    # ============ 二、参数详细说明 ============
    doc.add_heading('二、参数详细说明', level=1)
    for gname, items in groups:
        if not items:
            continue
        doc.add_heading(gname, level=2)
        t = doc.add_table(rows=1, cols=6)
        t.style = 'Light Grid Accent 1'
        hdr = t.rows[0].cells
        for i, h in enumerate(['参数名', '类型', '默认值', '范围', '单位', '说明']):
            set_cell(hdr[i], h, bold=True)
        for p in items:
            cells = t.add_row().cells
            set_cell(cells[0], p['name'], bold=True)
            set_cell(cells[1], p['type'])
            set_cell(cells[2], p['default'])
            set_cell(cells[3], (p['min'] + ' ~ ' + p['max']) if p['min'] and p['max'] else '—')
            set_cell(cells[4], p['unit'] or '—')
            set_cell(cells[5], p['desc'])

    # 注：不提供"推荐开启配置"——所有开关默认禁用，由使用者按需自行决定

    doc.save(OUT)
    print(f"Word 文档已生成: {OUT}")


if __name__ == '__main__':
    main()
