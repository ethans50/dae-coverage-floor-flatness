# surface_profiling/utils/frame_video.py
"""
프레임 기록(.npz, frame_recorder.py)으로 top-down 누적 영상을 만듦.

도면의 벽과 로봇(velodyne_link 위치)은 검은색, 결과에 반영된 바닥 점은 z 높이별
색으로 뿌려 로봇이 움직이면서 점이 쌓이는 과정을 보여줌.

색 규칙(z는 map 기준, floor_extractor의 z-window [z_min, z_max]와 같은 값, 히트맵과 같은 jet 컬러맵):
  - 셀 색 = 그 시점까지 누적된 z-window 안 점의 **평균 z**(히트맵의 셀 평균과 같은 집계).
    마지막 한 점이 아니라 평균이므로 최종 프레임은 히트맵과 같은 값으로 수렴함.
  - window 안에 점이 하나도 없고 밖에만 있는 셀: window 아래는 회색, 위는 자홍
    -> "점이 없는 것"과 "점은 있는데 z가 떠서 window에 잘리는 것"을 구분하기 위함.
  - 표시 밴드는 window를 z_margin만큼 넓힌 범위이고, 그 밖(벽/천장 등)은 그리지 않음.

기각된 프레임(저역통과)과 이동 중 프레임의 점은 그리지 않음(최종 결과에 반영되지
않는 점이므로). 화면 상단 텍스트에 누적 기각 프레임 수를 표시함.
"""

import os

import cv2
import numpy as np

from .frame_recorder import FrameRecorder, load_frame_log, used_frame_mask
from .heatmap_generator import _load_occupancy_map

_BELOW_COLOR = (128, 128, 128)  # BGR gray (jet 컬러맵과 겹치지 않는 색)
_ABOVE_COLOR = (255, 0, 255)    # BGR magenta
_LUT_SIZE = 256


def _jet_lut():
    """히트맵(matplotlib 'jet')과 같은 색을 쓰기 위해 matplotlib에서 직접 LUT를 뽑음(BGR)."""
    import matplotlib
    rgb = (matplotlib.colormaps['jet'](np.linspace(0.0, 1.0, _LUT_SIZE))[:, :3] * 255).astype(np.uint8)
    return rgb[:, ::-1].copy()


def _wall_mask(map_img):
    """점유(어두운) 픽셀 마스크. ROS map_server 기본(negate=0)은 점유=검정임."""
    img = map_img.astype(np.float64)
    if img.ndim == 3:
        img = img[..., :3].mean(axis=2)
    if img.max() > 1.0:
        img = img / 255.0
    return img < 0.25


def _view_bounds(walls, res, ox, oy, poses, margin_m):
    """벽 bbox와 로봇 궤적 bbox의 합집합(+margin)을 world 좌표로 반환함."""
    xs, ys = [poses[:, 0].min(), poses[:, 0].max()], [poses[:, 1].min(), poses[:, 1].max()]
    rows, cols = np.nonzero(walls)
    if rows.size:
        xs += [ox + cols.min() * res, ox + (cols.max() + 1) * res]
        ys += [oy + rows.min() * res, oy + (rows.max() + 1) * res]
    return (min(xs) - margin_m, max(xs) + margin_m, min(ys) - margin_m, max(ys) + margin_m)


def render_accumulation_video(npz_path, map_yaml_path, out_path, z_min, z_max,
                              z_margin=0.12, fps=30, max_video_frames=1500,
                              max_dim_px=1600, max_px_per_m=100.0, robot_radius_m=0.11):
    """누적 영상을 out_path(.mp4)에 저장하고 실제 저장 경로를 반환함(실패 시 None).

    max_video_frames: 영상 프레임 수 상한. 기록 프레임이 더 많으면 균등 간격으로
      솎아서 그리되, 점 누적은 모든 프레임에 대해 수행함.
    max_px_per_m: 확대 상한. 1 px = 1cm(100 px/m)면 점 간격이 조밀한 바닥에서도 충분함.
    """
    log = load_frame_log(npz_path)
    n_frames = len(log['stamps'])
    if n_frames == 0:
        return None

    poses, offsets, points = log['poses'], log['offsets'], log['points']
    flags, stamps = log['flags'], log['stamps']
    used = used_frame_mask(log)

    # --- 도면 / 뷰 영역 ---
    map_img, res, ox, oy = _load_occupancy_map(map_yaml_path)
    walls = _wall_mask(map_img)
    x0, x1, y0, y1 = _view_bounds(walls, res, ox, oy, poses, margin_m=1.0)
    px_per_m = min(max_px_per_m, max_dim_px / max(x1 - x0, y1 - y0))
    width = int((x1 - x0) * px_per_m) // 2 * 2
    height = int((y1 - y0) * px_per_m) // 2 * 2

    def to_px(x, y):
        # world -> 이미지 픽셀. 이미지 y는 아래로 증가하므로 뒤집음.
        return (np.rint((x - x0) * px_per_m).astype(np.int32),
                np.rint((y1 - y) * px_per_m).astype(np.int32))

    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    wr, wc = np.nonzero(walls)
    wx, wy = to_px(ox + (wc + 0.5) * res, oy + (wr + 0.5) * res)
    # 도면 픽셀(res)이 영상 픽셀보다 클 수 있어 벽 셀 크기만큼 사각형으로 채움.
    half = max(int(np.ceil(res * px_per_m / 2)), 0)
    for cx, cy in zip(wx, wy):
        cv2.rectangle(canvas, (cx - half, cy - half), (cx + half, cy + half), (0, 0, 0), -1)

    # --- 점 색상: 셀 평균 z를 히트맵과 같은 방식(clip 후 정규화, jet)으로 칠함 ---
    lut = _jet_lut()
    z_lo, z_hi = z_min - z_margin, z_max + z_margin
    n_px = width * height
    sum_in = np.zeros(n_px, dtype=np.float64)
    cnt_in = np.zeros(n_px, dtype=np.int32)
    sum_out = np.zeros(n_px, dtype=np.float64)
    cnt_out = np.zeros(n_px, dtype=np.int32)
    canvas_flat = canvas.reshape(-1, 3)

    def cell_colors(idx):
        """idx(고유 픽셀 인덱스)의 누적 평균으로 색을 정함."""
        colors = np.empty((len(idx), 3), dtype=np.uint8)
        has_in = cnt_in[idx] > 0
        mean_in = sum_in[idx[has_in]] / cnt_in[idx[has_in]]
        lut_idx = np.clip(((mean_in - z_min) / (z_max - z_min) * (_LUT_SIZE - 1)).astype(np.int32), 0, _LUT_SIZE - 1)
        colors[has_in] = lut[lut_idx]
        out = ~has_in
        mean_out = sum_out[idx[out]] / np.maximum(cnt_out[idx[out]], 1)
        colors[out] = np.where((mean_out < z_min)[:, None], np.array(_BELOW_COLOR, np.uint8), np.array(_ABOVE_COLOR, np.uint8))
        return colors

    # --- 영상 ---
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    stride = max(1, int(np.ceil(n_frames / max_video_frames)))
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    if not writer.isOpened():
        print(f"[!] VideoWriter를 열 수 없음: {out_path}")
        return None

    n_drawn = 0
    n_rejected = 0
    r_px = max(int(round(robot_radius_m * px_per_m)), 3)
    t0 = stamps[0]
    try:
        for i in range(n_frames):
            if flags[i] & FrameRecorder.FLAG_REJECTED:
                n_rejected += 1
            if used[i] and offsets[i + 1] > offsets[i]:
                pts = points[offsets[i]:offsets[i + 1]]
                pts = pts[(pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)]
                if len(pts):
                    px, py = to_px(pts[:, 0], pts[:, 1])
                    ok = (px >= 0) & (px < width) & (py >= 0) & (py < height)
                    idx = (py[ok] * width + px[ok]).astype(np.int64)
                    zz = pts[ok, 2].astype(np.float64)
                    inside = (zz >= z_min) & (zz <= z_max)
                    np.add.at(sum_in, idx[inside], zz[inside])
                    np.add.at(cnt_in, idx[inside], 1)
                    np.add.at(sum_out, idx[~inside], zz[~inside])
                    np.add.at(cnt_out, idx[~inside], 1)
                    touched = np.unique(idx)
                    canvas_flat[touched] = cell_colors(touched)
                    n_drawn += int(inside.sum())

            if i % stride != 0 and i != n_frames - 1:
                continue

            img = canvas.copy()
            rx, ry = to_px(np.array([poses[i, 0]]), np.array([poses[i, 1]]))
            rx, ry = int(rx[0]), int(ry[0])
            cv2.circle(img, (rx, ry), r_px, (0, 0, 0), -1)
            tip = (int(rx + 2 * r_px * np.cos(poses[i, 3])), int(ry - 2 * r_px * np.sin(poses[i, 3])))
            cv2.line(img, (rx, ry), tip, (0, 0, 0), max(r_px // 3, 1))

            state = "CAPTURE" if flags[i] & FrameRecorder.FLAG_CAPTURE else "transit"
            hud = (f"t={stamps[i] - t0:7.1f}s frame {i + 1}/{n_frames} [{state}] "
                   f"pts={n_drawn} rejected_frames={n_rejected}")
            cv2.putText(img, hud, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(img, f"cell mean z, jet [{z_min*100:.1f},{z_max*100:.1f}]cm | gray below | magenta above",
                        (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            writer.write(img)
    finally:
        writer.release()

    return out_path
