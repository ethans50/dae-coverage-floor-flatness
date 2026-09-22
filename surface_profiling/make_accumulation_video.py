#!/usr/bin/env python3
# surface_profiling/make_accumulation_video.py
"""
저장된 프레임 기록(.npz)으로 누적 영상(.mp4)을 다시 만드는 스크립트. 재주행 없이
z 창/표시 범위/해상도 등을 바꿔 가며 확인할 때 씀. 옵션을 주지 않으면 params.yaml의
surface_profiling 값을 씀.

사용 예:
  python3 make_accumulation_video.py ~/dae_floor_maps/analytics/pointclouds/frames/frames_<ts>.npz
  python3 make_accumulation_video.py <npz> --z-min -0.025 --z-max 0.03 --z-margin 0.2 --out /tmp/a.mp4
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.config_paths import load_config, resolve_map_yaml_path, resolve_visualization_dir  # noqa: E402
from utils.frame_video import render_accumulation_video  # noqa: E402


def main():
    workspace_root, cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--out', default=None, help='기본: visualization_dir/accumulation_<npz 이름의 타임스탬프>.mp4')
    ap.add_argument('--map', default=resolve_map_yaml_path(workspace_root, cfg))
    ap.add_argument('--z-min', type=float, default=cfg.get('z_min', -0.005))
    ap.add_argument('--z-max', type=float, default=cfg.get('z_max', 0.035))
    ap.add_argument('--z-margin', type=float, default=cfg.get('video_z_margin', 0.12))
    ap.add_argument('--fps', type=int, default=cfg.get('video_fps', 30))
    ap.add_argument('--max-frames', type=int, default=cfg.get('video_max_frames', 1500))
    ap.add_argument('--max-dim', type=int, default=cfg.get('video_max_dim_px', 1600))
    args = ap.parse_args()

    out = args.out
    if out is None:
        name = os.path.basename(args.npz).replace('frames_', 'accumulation_').replace('.npz', '.mp4')
        out = os.path.join(resolve_visualization_dir(workspace_root, cfg), name)

    result = render_accumulation_video(
        args.npz, args.map, out, z_min=args.z_min, z_max=args.z_max, z_margin=args.z_margin,
        fps=args.fps, max_video_frames=args.max_frames, max_dim_px=args.max_dim)
    print(f"[+] Saved: {result}" if result else "[!] 영상 생성 실패")


if __name__ == '__main__':
    main()
