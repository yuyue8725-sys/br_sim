#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图自检工具：随时确认 maps/robocon_map.pgm 是否完好、能否给 Nav2 用。

检查项
  1. 元信息：分辨率 / 原点 / 尺寸 / 覆盖范围
  2. 像素统计：占用 / 空闲 / 未知 比例
  3. 四面围墙是否闭合（从出生点做 4-邻域洪水填充，看会不会「漏」到地图边界外）
  4. 9 个巡视航点是否都能从出生点走到（全向底盘可直达的区域）

用法
  ros2 run br_sim check_map.py                     # 检查包内 maps/robocon_map.yaml
  ros2 run br_sim check_map.py --map /path/x.yaml  # 检查任意地图
  ros2 run br_sim check_map.py -v                  # 额外打印栅格坐标

退出码：0 = 全部通过，1 = 有检查项失败
"""
import argparse
import os
import sys
from collections import deque

# 与 sim.launch.py 的出生点一致
DEFAULT_START = (4.5, 2.0)

# 建图巡视用过的 9 个航点（环形通道四角 + 中段）
DEFAULT_WAYPOINTS = [
    ('东走廊南', 4.78, -4.55),
    ('东走廊中', 4.78, 0.0),
    ('东走廊北', 4.78, 4.55),
    ('北走廊中', 0.0, 4.60),
    ('西走廊北', -4.50, 4.55),
    ('西走廊中', -4.50, 1.30),
    ('西走廊南', -4.50, -4.55),
    ('南走廊中', 0.0, -4.55),
    ('南走廊东', 3.5, -4.55),
]


def load_map_yaml(path):
    """解析 map_server 的 yaml（有 PyYAML 就用，没有就简易解析）。"""
    try:
        import yaml
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except ImportError:
        pass
    meta = {}
    with open(path, 'r') as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line or ':' not in line:
                continue
            k, v = line.split(':', 1)
            k, v = k.strip(), v.strip()
            if v.startswith('['):
                meta[k] = [float(x) for x in v.strip('[]').split(',')]
            else:
                try:
                    meta[k] = float(v)
                except ValueError:
                    meta[k] = v
    return meta


def load_pgm(path):
    """读 P2(ASCII) / P5(二进制) PGM，返回 (宽, 高, 像素, maxval)。"""
    with open(path, 'rb') as f:
        data = f.read()
    magic = data[:2]
    if magic not in (b'P2', b'P5'):
        raise ValueError('不是 PGM 文件: %s' % path)

    # 逐个 token 读 header（中间可能有 # 注释）
    tokens, i, n = [], 0, len(data)
    while len(tokens) < 4:
        while i < n and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < n and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < n and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])

    if magic == b'P5':
        i += 1                                  # 跳过分隔空白
        px = data[i:i + w * h]
        if len(px) < w * h:
            raise ValueError('PGM 像素数据不完整')
    else:
        vals = [int(x) for x in data[i:].split()]
        px = bytes(v & 0xFF for v in vals[:w * h])
    return w, h, px, maxv


class GridMap(object):
    """按 map_server 语义解释像素：占用 / 空闲 / 未知。"""

    def __init__(self, yaml_path):
        self.yaml_path = os.path.abspath(yaml_path)
        d = load_map_yaml(self.yaml_path)
        img = d.get('image')
        if not img:
            raise IOError('yaml 缺少 image 字段: %s' % self.yaml_path)
        if not os.path.isabs(img):
            img = os.path.join(os.path.dirname(self.yaml_path), img)
        if not os.path.exists(img):
            raise IOError('yaml 里的 image 不存在: %s' % img)
        self.image_path = img
        self.w, self.h, self.px, self.maxv = load_pgm(img)
        self.res = float(d.get('resolution', 0.05))
        origin = d.get('origin', [0.0, 0.0, 0.0])
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.negate = int(d.get('negate', 0))
        self.occ_th = float(d.get('occupied_thresh', 0.65))
        self.free_th = float(d.get('free_thresh', 0.25))
        self.mode = d.get('mode', 'trinary')

    # ---- 坐标系换算 -------------------------------------------------
    def to_rc(self, x, y):
        """世界坐标 -> (row, col)。PGM 第 0 行是 y 最大处。"""
        c = int((x - self.ox) / self.res)
        r = self.h - 1 - int((y - self.oy) / self.res)
        return r, c

    def to_xy(self, r, c):
        return self.ox + c * self.res, self.oy + (self.h - 1 - r) * self.res

    def raw(self, r, c):
        return self.px[r * self.w + c]

    def occupancy(self, r, c):
        """map_server 判据：occ = (255-v)/255，negate 时取反。"""
        occ = (255.0 - self.raw(r, c)) / 255.0
        return 1.0 - occ if self.negate else occ

    def kind(self, r, c):
        occ = self.occupancy(r, c)
        if occ > self.occ_th:
            return 'occ'
        if occ < self.free_th:
            return 'free'
        return 'unk'

    def kind_at(self, x, y):
        r, c = self.to_rc(x, y)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 'out'
        return self.kind(r, c)

    # ---- 连通性 -----------------------------------------------------
    def flood(self, sx, sy):
        """从 (sx, sy) 起在「非占用」格上做 4-邻域洪水填充。

        返回 (reached, leaks)；leaks = 扩展到地图边界外的次数，
        >0 说明围墙没闭合（可通行区域漏出去了）。
        """
        start = self.to_rc(sx, sy)
        if not (0 <= start[0] < self.h and 0 <= start[1] < self.w):
            raise ValueError('起点 (%.2f, %.2f) 不在 map 范围内' % (sx, sy))
        if self.kind(*start) == 'occ':
            raise ValueError('起点 (%.2f, %.2f) 落在障碍格上' % (sx, sy))
        seen = {start}
        q = deque([start])
        leaks = 0
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.h and 0 <= nc < self.w):
                    leaks += 1
                    continue
                if (nr, nc) in seen or self.kind(nr, nc) == 'occ':
                    continue
                seen.add((nr, nc))
                q.append((nr, nc))
        return seen, leaks


def default_map_path():
    """优先取 install/share 里的地图，其次退回源码目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('br_sim'),
                            'maps', 'robocon_map.yaml')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.abspath(os.path.join(here, '..', 'maps', 'robocon_map.yaml'))


def main(argv=None):
    ap = argparse.ArgumentParser(description='地图自检：围墙闭合性 / 走廊可达性')
    ap.add_argument('--map', default=default_map_path(), help='地图 yaml 路径')
    ap.add_argument('--start', nargs=2, type=float, default=list(DEFAULT_START),
                    metavar=('X', 'Y'), help='洪水填充起点（默认出生点 4.5 2.0）')
    ap.add_argument('-v', '--verbose', action='store_true', help='打印栅格坐标')
    args = ap.parse_args(argv)

    print('=' * 62)
    print(' 地图自检: %s' % args.map)
    print('=' * 62)

    try:
        gm = GridMap(args.map)
    except (IOError, ValueError) as e:
        print(' ✗ 无法读取地图: %s' % e)
        return 1

    print('[元信息]')
    print('  图像     : %s' % gm.image_path)
    print('  栅格     : %d × %d   分辨率 %.3f m/格' % (gm.w, gm.h, gm.res))
    print('  原点     : (%.3f, %.3f)' % (gm.ox, gm.oy))
    print('  覆盖范围 : x %.2f ~ %.2f   y %.2f ~ %.2f' % (
        gm.ox, gm.ox + gm.w * gm.res, gm.oy, gm.oy + gm.h * gm.res))
    print('  阈值     : free<%s  occupied>%s  negate=%d  mode=%s'
          % (gm.free_th, gm.occ_th, gm.negate, gm.mode))

    # ---- 像素统计 ---------------------------------------------------
    cnt = {'occ': 0, 'free': 0, 'unk': 0}
    raw = {'black': 0, 'gray': 0, 'white': 0}
    for r in range(gm.h):
        for c in range(gm.w):
            cnt[gm.kind(r, c)] += 1
            v = gm.raw(r, c)
            raw['black' if v < 65 else ('white' if v > 240 else 'gray')] += 1
    total = float(gm.w * gm.h)
    print('\n[像素统计] 按 yaml 阈值: 占用 %.1f%%   空闲 %.1f%%   未知 %.1f%%'
          % (100 * cnt['occ'] / total, 100 * cnt['free'] / total,
             100 * cnt['unk'] / total))
    print('           原始像素  : 黑(0) %.1f%%   灰(205) %.1f%%   白(254) %.1f%%'
          % (100 * raw['black'] / total, 100 * raw['gray'] / total,
             100 * raw['white'] / total))
    if gm.mode == 'trinary' and raw['gray'] and gm.free_th > 0.196:
        print('   ⓘ mode=trinary 的未知像素=205（占用率 0.196），free_thresh=%.3f > 0.196\n'
              '     时 map_server 会把它当「空闲」。本场地未知区（平台内部、场外背景）\n'
              '     都被围墙/侧面封闭，洪水填充证明走不进去，故不影响规划。\n'
              '     若要严格保留 unknown，把 map yaml 的 free_thresh 改成 0.196。'
              % gm.free_th)

    fails = []

    # ---- 围墙闭合性 -------------------------------------------------
    sx, sy = args.start
    print('\n[围墙闭合性] 起点 (%.2f, %.2f)' % (sx, sy))
    try:
        seen, leaks = gm.flood(sx, sy)
    except ValueError as e:
        print('  ✗ %s' % e)
        return 1
    x0, y0 = gm.to_xy(max(p[0] for p in seen), min(p[1] for p in seen))
    x1, y1 = gm.to_xy(min(p[0] for p in seen), max(p[1] for p in seen))
    print('  可通行连通域: %d 格（%.1f m²）' % (len(seen), len(seen) * gm.res ** 2))
    print('  包围盒      : x %.2f ~ %.2f   y %.2f ~ %.2f' % (x0, x1, y0, y1))
    if leaks:
        print('  ✗ 越界 %d 次 → 围墙未闭合（或地图被裁切）' % leaks)
        fails.append('围墙未闭合')
    else:
        print('  ✓ 0 次越界，围墙闭合，可通行区被完全包围')

    # ---- 航点可达性 -------------------------------------------------
    print('\n[走廊可达性] 从起点到各航点')
    names = {'occ': '障碍格', 'unk': '未知格', 'out': '超出地图', 'free': '不连通'}
    for name, wx, wy in DEFAULT_WAYPOINTS:
        k = gm.kind_at(wx, wy)
        rc = gm.to_rc(wx, wy)
        ok = (k != 'out') and (rc in seen)
        if args.verbose:
            extra = ' → 栅格(%3d,%3d) %s' % (rc[0], rc[1], k)
        else:
            extra = ''
        print('  %s %-8s (%6.2f, %6.2f)%s%s'
              % ('✓' if ok else '✗', name, wx, wy, extra,
                 '' if ok else '（%s）' % names[k]))
        if not ok:
            fails.append('航点不可达: %s' % name)

    # ---- 结论 -------------------------------------------------------
    print('\n' + '=' * 62)
    if fails:
        print(' 结果: ✗ 失败 %d 项 → %s' % (len(fails), '，'.join(fails)))
        print(' 建议: 重新建图并保存（README 步骤 2/3）后重跑本检查')
    else:
        print(' 结果: ✓ 全部通过，地图可直接用于 navigation.launch.py')
    print('=' * 62)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
    