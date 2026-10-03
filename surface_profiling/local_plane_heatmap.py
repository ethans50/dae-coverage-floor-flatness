#!/usr/bin/env python3
# surface_profiling/local_plane_heatmap.py
"""
프레임 기록(.npz)에서 "국소 평면 대비 편차" 히트맵을 만들고, 처리 방식별로 비교 지표를 출력함.

처리 순서:
  1) 센서 거리 r 이 [r_min, r_max] 인 점만 사용함. r_max 는 --max-ring 번째 하향 빔(바닥 ring)과
     그 다음 ring 반경의 중간값임. 먼 ring 일수록 자세 오차가 거리에 비례해 커지므로 안쪽 ring 만 씀.
  2) 벽 근처 점을 제외함. --outside-nodes exclude(기본)면 토폴로지 노드 마스크 밖의 점(벽 바로 옆
     띠와 문턱 구간)을, --wall-margin M 을 주면 지도 벽에서 M[m] 이내의 점을 제외함. 이 구간의 점은
     같은 셀의 안쪽 관측보다 일관되게 높고 z 분포가 z 창 상단까지 길게 퍼져, 바닥이 아니라 벽 면의
     점이 z 창에 섞인 것으로 보임(확정은 아님). 제외가 정확도 개선을 입증하는 것은 아님.
  3) 모드별로 잔차를 계산함.
       raw    전체 중앙값만 뺌(국소 평면 없음. 비교 기준)
       frame  프레임 하나의 점으로 맞춘 평면을 뺌
       window 여러 프레임을 합쳐 점마다 주변 --window 지름 원 안의 점으로 맞춘 평면을 뺌

정답(GT)이 없으므로 시각화만으로 오차 제거와 왜곡을 구분할 수 없음. 그래서 지표를 함께 냄.
  - 교차 일관성: 같은 셀을 서로 반대 진행 방향 프레임들이 각각 본 평균의 차이 표준편차(작을수록
    측정 오차가 적음. 실제 바닥은 같으므로 차이는 오차뿐임)
  - 돔 보존율: 알려진 높이의 raised-cosine 돔(z = h/2*(1+cos(pi*r/R)), r<R)을 점에 더해 같은 처리를
    한 뒤, 돌려받은 신호를 돔 모양에 최소제곱으로 맞춘 이득(1에 가까울수록 실제 결함이 보존됨.
    돔 크기별로 평면이 얼마나 먹는지 보여 줌)
  - 링별 평균 잔차: ring 간 편향이 남았는지
  - 셀 평균의 공간 표준편차(남은 무늬의 크기. 단독으로는 왜곡과 구분되지 않으므로 위 지표와 함께 볼 것)

출력은 옵션 조합별로 파일 이름이 달라짐(~/dae_floor_maps/visualization/surface_profiling/):
  local_plane_<ts>[_keep][_wm<M>].png          모드별 히트맵(색 범위 +-vrange cm)
  local_plane_<ts>[_keep][_wm<M>]_metrics.png   돔 보존율과 링별 편향 그래프
  local_plane_<ts>[_keep][_wm<M>]_excluded.png  제외된 점만 그린 지도(--show-excluded 일 때)

사용 예:
  # 기본: ring 4 이내, 노드 밖 점 제외, 세 모드 비교 (히트맵 + 지표 출력, 약 2~3분)
  python3 local_plane_heatmap.py frames_<ts>.npz
  # 노드 밖 점을 포함한 결과(파일 이름에 _keep 이 붙음)와 비교
  python3 local_plane_heatmap.py frames_<ts>.npz --outside-nodes keep
  # 벽에서 0.2 m 이내도 제외하고, 제외된 점이 어디에 있는지도 그림
  python3 local_plane_heatmap.py frames_<ts>.npz --wall-margin 0.2 --show-excluded
  # frame 모드만, 합성 돔을 방 안쪽 (x, y)에 넣어 보존율 확인, 색 범위 +-1.5 cm
  python3 local_plane_heatmap.py frames_<ts>.npz --modes frame --inject-at -6.7 1.0 --vrange 1.5
  # 빠르게 히트맵만 (보존율 계산 생략)
  python3 local_plane_heatmap.py frames_<ts>.npz --no-inject
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.frame_recorder import describe_capture_conditions, load_frame_log, used_frame_mask  # noqa: E402
from utils.local_plane import frame_plane_residual, window_plane_residual  # noqa: E402
from utils.ring_coverage import vlp16_ring_radii_m  # noqa: E402

HOME = os.path.expanduser('~/dae_floor_maps')
INJECT_RADII_M = (0.25, 0.5, 0.75, 1.0, 1.25)
INJECT_HEIGHT_M = 0.02


def resolve_npz(path):
    if os.path.exists(path):
        return path
    for sub in ('frames', 'waypoints'):
        cand = os.path.join(HOME, 'analytics', 'pointclouds', sub, os.path.basename(path))
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError(path)


def ring_limit(mount_height_m, max_ring):
    radii = vlp16_ring_radii_m(mount_height_m)
    if max_ring >= len(radii):
        return float(radii[-1]) + 0.3
    return float((radii[max_ring - 1] + radii[max_ring]) / 2.0)


def load_node_lookup(topology_path):
    t = np.load(topology_path)
    nodes = (t['nodes'] > 0)[:, ::-1, :]          # 지도 이미지 행 순서(위에서 아래)를 world y 오름차순으로 뒤집음
    res, org = float(t['resolution']), t['origin']
    inside = nodes.any(axis=0)

    def lookup(x, y):
        ix, iy = ((x - org[0]) / res).astype(int), ((y - org[1]) / res).astype(int)
        ok = (ix >= 0) & (iy >= 0) & (ix < inside.shape[1]) & (iy < inside.shape[0])
        out = np.zeros(x.shape, dtype=bool)
        out[ok] = inside[iy[ok], ix[ok]]
        return out
    return lookup


def load_points(log, r_min, r_max, z_band, keep_fn):
    off, poses = log['offsets'], log['poses']
    used = used_frame_mask(log)
    cols = {k: [] for k in ('x', 'y', 'z', 'r', 'fid', 'cx', 'cy', 'grp')}
    dropped_outside = 0
    for i in np.nonzero(used & (off[1:] > off[:-1]))[0]:
        p = log['points'][off[i]:off[i + 1]]
        r = np.hypot(p[:, 0] - poses[i, 0], p[:, 1] - poses[i, 1])
        k = (r >= r_min) & (r <= r_max) & (np.abs(p[:, 2]) <= z_band)
        if keep_fn is not None:
            ins = keep_fn(p[:, 0], p[:, 1])
            dropped_outside += int((k & ~ins).sum())
            k &= ins
        n = int(k.sum())
        if n == 0:
            continue
        cols['x'].append(p[k, 0]); cols['y'].append(p[k, 1]); cols['z'].append(p[k, 2]); cols['r'].append(r[k])
        cols['fid'].append(np.full(n, i)); cols['cx'].append(np.full(n, poses[i, 0])); cols['cy'].append(np.full(n, poses[i, 1]))
        cols['grp'].append(np.full(n, np.sin(poses[i, 3]) >= 0))   # 진행 방향 두 그룹(교차 일관성용)
    d = {k: np.concatenate(v) for k, v in cols.items()}
    d['x'], d['y'], d['z'] = d['x'].astype(np.float64), d['y'].astype(np.float64), d['z'].astype(np.float64)
    return d, dropped_outside, float(poses[used, 2].mean())


def load_wall_distance(map_yaml):
    """지도 벽까지의 거리[m]를 돌려주는 함수를 만듦."""
    import cv2
    from utils.frame_video import _wall_mask
    from utils.heatmap_generator import _load_occupancy_map
    img, res, ox, oy = _load_occupancy_map(map_yaml)
    dist = cv2.distanceTransform((~_wall_mask(img)).astype(np.uint8), cv2.DIST_L2, 5) * res

    def lookup(x, y):
        ix = np.clip(((x - ox) / res).astype(int), 0, dist.shape[1] - 1)
        iy = np.clip(((y - oy) / res).astype(int), 0, dist.shape[0] - 1)
        return dist[iy, ix]
    return lookup


def residual(mode, d, z, window_m):
    if mode == 'raw':
        return z - np.median(z)
    if mode == 'frame':
        out = np.empty(z.size)
        cuts = np.concatenate([[0], np.flatnonzero(np.diff(d['fid'])) + 1, [z.size]])
        for a, b in zip(cuts[:-1], cuts[1:]):
            out[a:b] = frame_plane_residual(d['x'][a:b], d['y'][a:b], z[a:b], d['cx'][a], d['cy'][a])
        return out
    if mode == 'window':
        return window_plane_residual(d['x'], d['y'], z, window_m=window_m)
    raise ValueError(mode)


def cell_index(x, y, g, origin):
    return ((y - origin[1]) / g).astype(np.int64) * 100000 + ((x - origin[0]) / g).astype(np.int64)


def spatial_std(d, res, g=0.05):
    ok = ~np.isnan(res)
    key = cell_index(d['x'][ok], d['y'][ok], g, (d['x'].min(), d['y'].min()))
    u, inv = np.unique(key, return_inverse=True)
    cnt, s = np.bincount(inv), np.bincount(inv, res[ok])
    m = (s / cnt)[cnt >= 5]
    return float(m.std() * 100.0) if m.size else float('nan')


def crossover_std(d, res, g=0.05, n_min=8):
    ok = ~np.isnan(res)
    key = cell_index(d['x'][ok], d['y'][ok], g, (d['x'].min(), d['y'].min()))
    u, inv = np.unique(key, return_inverse=True)
    stats = []
    for grp in (True, False):
        w = (d['grp'][ok] == grp)
        c = np.bincount(inv, weights=w, minlength=u.size)
        s = np.bincount(inv, weights=w * res[ok], minlength=u.size)
        stats.append((c, s))
    both = (stats[0][0] >= n_min) & (stats[1][0] >= n_min)
    diff = stats[0][1][both] / stats[0][0][both] - stats[1][1][both] / stats[1][0][both]
    return (float(diff.std() * 100.0), int(both.sum())) if diff.size else (float('nan'), 0)


def ring_means(d, res, radii):
    edges = [0.0] + list((radii[:-1] + radii[1:]) / 2.0) + [1e9]
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (d['r'] >= lo) & (d['r'] < hi) & ~np.isnan(res)
        out.append(float(res[m].mean() * 100.0) if m.sum() > 1000 else float('nan'))
    return out


def dome(d, cx, cy, radius, height):
    r = np.hypot(d['x'] - cx, d['y'] - cy)
    return np.where(r < radius, height / 2.0 * (1.0 + np.cos(np.pi * r / radius)), 0.0)


def retention(res, res0, prof):
    ok = (prof > 0) & ~np.isnan(res) & ~np.isnan(res0)
    if ok.sum() < 500:
        return float('nan')
    return float(((res - res0)[ok] * prof[ok]).sum() / (prof[ok] ** 2).sum())


def render_map(ax, d, res, vrange_cm, map_yaml, title, g=0.02):
    ok = ~np.isnan(res)
    x, y = d['x'][ok], d['y'][ok]
    xb, yb = np.arange(x.min(), x.max() + g, g), np.arange(y.min(), y.max() + g, g)
    s, _, _ = np.histogram2d(x, y, bins=[xb, yb], weights=res[ok])
    c, _, _ = np.histogram2d(x, y, bins=[xb, yb])
    m = np.where(c > 0, s / np.maximum(c, 1), np.nan) * 100.0
    if map_yaml and os.path.exists(map_yaml):
        from utils.heatmap_generator import _load_occupancy_map
        img, resn, ox, oy = _load_occupancy_map(map_yaml)
        ax.imshow(img, cmap='gray', origin='lower', extent=[ox, ox + img.shape[1] * resn, oy, oy + img.shape[0] * resn], zorder=0)
    cmap = plt.get_cmap('jet').copy()
    cmap.set_bad((1, 1, 1, 0))
    im = ax.imshow(m.T, origin='lower', extent=[x.min(), x.max(), y.min(), y.max()], cmap=cmap,
                   vmin=-vrange_cm, vmax=vrange_cm, interpolation='nearest', zorder=1)
    ax.set_xlim(x.min() - 0.3, x.max() + 0.3); ax.set_ylim(y.min() - 0.3, y.max() + 0.3)
    ax.set_aspect('equal'); ax.set_title(title, fontsize=10)
    return im


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--modes', default='raw,frame,window', help='쉼표로 구분: raw, frame, window')
    ap.add_argument('--max-ring', type=int, default=4, help='사용할 가장 바깥 바닥 ring 번호(1이 가장 안쪽)')
    ap.add_argument('--r-min', type=float, default=1.1)
    ap.add_argument('--z-band', type=float, default=0.06, help='|z|가 이 값[m] 이하인 점만 사용')
    ap.add_argument('--window', type=float, default=3.0, help='window 모드 평면 창 지름[m]')
    ap.add_argument('--outside-nodes', choices=['exclude', 'keep'], default='exclude')
    ap.add_argument('--wall-margin', type=float, default=0.0, help='지도 벽에서 이 거리[m] 이내 점 제외(0이면 사용 안 함)')
    ap.add_argument('--topology', default=os.path.join(HOME, 'maps', 'topology', 'final_topological_map.npz'))
    ap.add_argument('--map', default=os.path.join(HOME, 'maps', 'grid', 'map_from_dae.yaml'))
    ap.add_argument('--show-excluded', action='store_true', help='제외된 점(노드 밖/벽 근처)만 그린 지도를 추가로 저장')
    ap.add_argument('--vrange', type=float, default=2.0, help='색 범위 +-[cm]')
    ap.add_argument('--inject-at', nargs=2, type=float, metavar=('X', 'Y'), help='합성 돔 중심(기본: 점이 가장 많은 1m 블록)')
    ap.add_argument('--no-inject', action='store_true', help='돔 보존율 계산 생략(시간 절약)')
    ap.add_argument('--out-dir', default=os.path.join(HOME, 'visualization', 'surface_profiling'))
    args = ap.parse_args()

    path = resolve_npz(args.npz)
    log = load_frame_log(path)
    lines, warns = describe_capture_conditions(log)
    for ln in lines:
        print(f"[*] 측정 조건: {ln}")
    for w in warns:
        print(f"[!] {w}")

    inside_fn = None
    if args.outside_nodes == 'exclude':
        if os.path.exists(args.topology):
            inside_fn = load_node_lookup(args.topology)
        else:
            print(f"[!] 토폴로지 마스크가 없어 노드 밖 점 제외를 건너뜀: {args.topology}")
    keep_fn = inside_fn
    if args.wall_margin > 0:
        wall_dist = load_wall_distance(args.map)
        keep_fn = (lambda x, y: wall_dist(x, y) > args.wall_margin) if inside_fn is None else \
            (lambda x, y: inside_fn(x, y) & (wall_dist(x, y) > args.wall_margin))
    mount_h = float(np.nanmean(log['poses'][used_frame_mask(log), 2]))
    r_max = ring_limit(mount_h, args.max_ring)
    d, dropped, _ = load_points(log, args.r_min, r_max, args.z_band, keep_fn)
    print(f"[*] 센서 높이 {mount_h:.3f} m, 사용 거리 {args.r_min:.2f}-{r_max:.2f} m(ring 1~{args.max_ring}), "
          f"점 {d['z'].size}개 (벽 근처/노드 밖 제외 {dropped}개)")

    radii = vlp16_ring_radii_m(mount_h)[:args.max_ring]
    modes = [m for m in args.modes.split(',') if m]
    if args.inject_at:
        cx, cy = args.inject_at
    else:
        hh, xe, ye = np.histogram2d(d['x'], d['y'], bins=[np.arange(d['x'].min(), d['x'].max() + 1, 1.0), np.arange(d['y'].min(), d['y'].max() + 1, 1.0)])
        i, j = np.unravel_index(hh.argmax(), hh.shape)
        cx, cy = xe[i] + 0.5, ye[j] + 0.5

    results = {}
    for mode in modes:
        res = residual(mode, d, d['z'], args.window)
        sstd = spatial_std(d, res)
        cstd, ncell = crossover_std(d, res)
        rings = ring_means(d, res, radii)
        ret = []
        if not args.no_inject:
            for R in INJECT_RADII_M:
                prof = dome(d, cx, cy, R, INJECT_HEIGHT_M)
                ret.append(retention(residual(mode, d, d['z'] + prof, args.window), res, prof))
        results[mode] = dict(res=res, spatial=sstd, cross=cstd, ncell=ncell, rings=rings, ret=ret)
        print(f"[{mode}] 셀 평균 공간 std {sstd:.3f} cm | 교차 일관성 std {cstd:.3f} cm (셀 {ncell}) | "
              f"링별 평균 잔차(cm) {', '.join('%+.2f' % v for v in rings)}")
        if ret:
            print(f"        돔 보존율(합성 높이 {INJECT_HEIGHT_M*100:.0f} cm, 위치 ({cx:.1f},{cy:.1f}), 반경 "
                  + ' / '.join(f'{R:g}m' for R in INJECT_RADII_M) + "): " + ' / '.join('%.2f' % v for v in ret))

    tag = os.path.splitext(os.path.basename(path))[0].replace('frames_', '')
    tag += ('_keep' if args.outside_nodes == 'keep' else '') + (f'_wm{args.wall_margin:g}' if args.wall_margin > 0 else '')
    os.makedirs(args.out_dir, exist_ok=True)
    fig, axes = plt.subplots(1, len(modes), figsize=(9 * len(modes), 8), squeeze=False)
    for ax, mode in zip(axes[0], modes):
        r = results[mode]
        im = render_map(ax, d, r['res'], args.vrange, args.map,
                        f"{mode}   cell-mean std {r['spatial']:.2f} cm   crossover std {r['cross']:.2f} cm")
    fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.7, label=f'residual [cm] (+-{args.vrange})')
    out1 = os.path.join(args.out_dir, f'local_plane_{tag}.png')
    fig.savefig(out1, dpi=70, bbox_inches='tight'); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    for mode in modes:
        if results[mode]['ret']:
            ax[0].plot(INJECT_RADII_M, results[mode]['ret'], marker='o', label=mode)
        ax[1].plot(range(1, len(radii) + 1), results[mode]['rings'], marker='o', label=mode)
    ax[0].set_xlabel('synthetic dome radius R [m]'); ax[0].set_ylabel('retention'); ax[0].set_ylim(0, 1.1)
    ax[0].axhline(1, color='gray', lw=0.5); ax[0].set_title('dome retention (1 = preserved)'); ax[0].legend()
    ax[1].set_xlabel('ring (1 = innermost)'); ax[1].set_ylabel('mean residual [cm]'); ax[1].set_title('ring bias'); ax[1].legend()
    out2 = os.path.join(args.out_dir, f'local_plane_{tag}_metrics.png')
    fig.savefig(out2, dpi=80, bbox_inches='tight'); plt.close(fig)
    print(f"[+] saved: {out1}\n[+] saved: {out2}")

    if args.show_excluded and keep_fn is not None:
        dall, _, _ = load_points(log, args.r_min, r_max, args.z_band, None)
        sub = {k: v[~keep_fn(dall['x'], dall['y'])] for k, v in dall.items()}
        fig, ax = plt.subplots(figsize=(9, 8))
        im = render_map(ax, sub, sub['z'] - np.median(d['z']), args.vrange, args.map,
                        f"excluded points only ({sub['z'].size} pts, raw z - median)")
        fig.colorbar(im, ax=ax, shrink=0.7, label=f'z - median [cm] (+-{args.vrange})')
        out3 = os.path.join(args.out_dir, f'local_plane_{tag}_excluded.png')
        fig.savefig(out3, dpi=70, bbox_inches='tight'); plt.close(fig)
        print(f"[+] saved: {out3}")


if __name__ == '__main__':
    main()
