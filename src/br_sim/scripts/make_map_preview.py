#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 PGM 地图渲染成 PNG 预览图（README 展示用）。

用法
  ros2 run br_sim make_map_preview.py                  # 生成 maps/robocon_map_preview.png
  ros2 run br_sim make_map_preview.py --scale 4        # 放大 4 倍
  ros2 run br_sim make_map_preview.py --map x.yaml -o y.png

依赖：Pillow（sudo apt install python3-pil）
颜色：墙=黑  可通行=白  未探索=灰  机器人出生点=红十字
"""
import argparse
import os
import sys

from check_map import DEFAULT_START, GridMap, default_map_path

COLOR = {'occ': (0, 0, 0), 'free': (255, 255, 255), 'unk': (176, 176, 176)}
MARGIN = 28          # 底部留白，用于画 1 m 比例尺


def color_of(gm, r, c):
    """预览配色：墙=黑，未探索=浅灰，可通行=白。

    注意：map_saver 的 trinary 模式把未知写成 205，而 `free_thresh: 0.25` 时
    map_server 会把它当「空闲」。这里按**原始像素**把 205 画成灰色，
    是为了让预览图能看出「哪些区域还没探索」，与 map_server 的判定有意不同。
    """
    if gm.kind(r, c) == 'occ':
        return COLOR['occ']
    if gm.raw(r, c) < 250:       # 205 = trinary 未知像素
        return COLOR['unk']
    return COLOR['free']


def main(argv=None):
    ap = argparse.ArgumentParser(description='渲染地图 PNG 预览')
    ap.add_argument('--map', default=default_map_path(), help='地图 yaml 路径')
    ap.add_argument('-o', '--out', default=None, help='输出 png（默认与地图同目录）')
    ap.add_argument('--scale', type=int, default=3, help='每格像素数（默认 3）')
    ap.add_argument('--start', nargs=2, type=float, default=list(DEFAULT_START),
                    metavar=('X', 'Y'), help='机器人出生点标记')
    args = ap.parse_args(argv)

    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        print('需要 Pillow: sudo apt install -y python3-pil')
        return 1

    gm = GridMap(args.map)
    # 用 realpath：install/share 里是软链，PNG 要落在源码 maps/ 目录
    src_dir = os.path.dirname(os.path.realpath(gm.yaml_path))
    out = args.out or os.path.join(src_dir, 'robocon_map_preview.png')
    S = max(1, args.scale)

    img = Image.new('RGB', (gm.w * S, gm.h * S + MARGIN), (255, 255, 255))
    px = img.load()
    for r in range(gm.h):
        k = [color_of(gm, r, c) for c in range(gm.w)]
        for c in range(gm.w):
            col = k[c]
            for dy in range(S):
                for dx in range(S):
                    px[c * S + dx, r * S + dy] = col

    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default()
    except Exception:
        font = None

    # 1 m 比例尺
    bar = int(round(1.0 / gm.res)) * S
    y0 = gm.h * S + 8
    d.rectangle([6, y0, 6 + bar, y0 + 4], fill=(0, 0, 0))
    if font:
        d.text((6 + bar + 6, y0 - 3), '1 m', fill=(0, 0, 0), font=font)

    # 出生点十字
    sx, sy = args.start
    r0, c0 = gm.to_rc(sx, sy)
    cx, cy = int(c0 * S + S / 2), int(r0 * S + S / 2)
    d.line([cx - 7, cy, cx + 7, cy], fill=(220, 0, 0), width=2)
    d.line([cx, cy - 7, cx, cy + 7], fill=(220, 0, 0), width=2)

    img.save(out)
    print('已生成: %s  (%d × %d px, %d px/格)' % (out, img.size[0], img.size[1], S))
    return 0


if __name__ == '__main__':
    sys.exit(main())
