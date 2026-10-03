#!/usr/bin/env python3
# surface_profiling/analyze_min_detectable_height.py
"""
프레임 기록(.npz)에 raised-cosine 돔을 합성해, 돔 지름별 최소 검출 높이(MDH)를 추정 방식별로 구함.
시뮬레이션이나 노이즈 모델 없이 기록된 실측 점군(실측 오차와 바닥 기복 포함)만 씀.

추정 방식(셀 격자 위 높이장 e):
  raw     전체 중앙값만 뺀 셀 평균(프레임 평면 없음. 비교 기준)
  frame   프레임 하나의 점으로 맞춘 평면을 뺀 셀 평균 잔차(기준 길이 = 프레임)
  joint   바닥 높이장과 프레임별 평면을 함께 추정한 셀 높이(utils/floor_adjustment.py)

검출 통계: 중심 p, 지름 D 인 돔 모양에 대해 반경 1.25*R 창 안의 셀 값을 e = A*돔 + B(상수)로 최소제곱 적합한 진폭 A(p)[m].
상수 B 는 국소 오프셋을 흡수함. 돔이 창에 충분히 덮일 때만(관측 셀 50% 이상, 돔 면적 60% 이상) 유효하고, 돔 전체가 방 안에 들어가도록
중심이 지도 벽에서 R 이상 떨어진 위치만 후보로 씀(오경보 임계와 주입 위치 모두 같은 후보 집합).

최소 검출 높이: 돔 없는 기록에서 모든 유효 위치의 A 의 최댓값(이 기록에서 오경보가 0 이 되는 임계 T)을 넘기려면 필요한 높이.
  MDH(p0) = (T - A0(p0)) / g(p0, D),  g = 추정기가 돌려주는 진폭 이득(= 돔 높이 h 를 넣었을 때 A 의 증가 / h)
추정기가 높이에 선형이라고 보고 h_inj 한 번의 주입으로 g 를 구하며, 선형성은 다른 높이로 확인함. 주입 위치 p0 는 유효 위치에서
무작위(고정 시드)로 최소 간격을 두고 뽑아 위치별 분포(중앙값, 90분위)를 보고함. T 는 이 기록 하나의 최댓값이라 일반화된 오경보율이
아니며, 99.9분위 임계(T_q)도 함께 냄. 기록에 이미 있는 실제 바닥 기복은 배경으로 포함됨.

사용 예:
  python3 analyze_min_detectable_height.py frames_<ts>.npz
  python3 analyze_min_detectable_height.py frames_<ts>.npz --diameters 100,200,300 --n-locations 4 --no-joint
"""

import argparse
import csv
import os
import sys

import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import local_plane_heatmap as L  # noqa: E402
from utils.floor_adjustment import FloorAdjustment  # noqa: E402
from utils.frame_recorder import used_frame_mask  # noqa: E402
from utils.local_plane import frame_plane_residual  # noqa: E402

WINDOW_FACTOR = 1.25
MIN_WINDOW_COVER, MIN_DOME_COVER = 0.5, 0.6


def kernels(radius, cell):
    k = int(np.ceil(WINDOW_FACTOR * radius / cell))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    dist = np.hypot(xx, yy) * cell
    window = (dist <= WINDOW_FACTOR * radius).astype(np.float64)
    dome = np.where(dist < radius, 0.5 * (1.0 + np.cos(np.pi * np.minimum(dist, radius) / radius)), 0.0)
    return window, dome


def amplitude_map(e, window, dome):
    """모든 중심에서 e = A*돔 + B 적합의 A. 관측이 부족한 중심은 NaN(두 번째 반환값 = 유효 마스크)."""
    m = (~np.isnan(e)).astype(np.float64)
    ee = np.where(m > 0, e, 0.0)
    cor = lambda a, k: ndimage.correlate(a, k, mode='constant')
    n, se, sp, spp, sep = cor(m, window), cor(ee, window), cor(m, dome), cor(m, dome ** 2), cor(ee, dome)
    den = spp - sp ** 2 / np.maximum(n, 1.0)
    valid = (n >= MIN_WINDOW_COVER * window.sum()) & (sp >= MIN_DOME_COVER * dome.sum()) & (den > 1e-9)
    amp = np.where(valid, (sep - se * sp / np.maximum(n, 1.0)) / np.where(valid, den, 1.0), np.nan)
    return amp, valid


class Fields:
    """기록과 격자, 추정 방식별 높이장 계산(돔 주입 포함)."""

    def __init__(self, d, cell_m, use_joint, smooth):
        self.d, self.C = d, cell_m
        x, y = d['x'], d['y']
        self.x0, self.y0 = x.min(), y.min()
        ix, iy = ((x - self.x0) / cell_m).astype(np.int64), ((y - self.y0) / cell_m).astype(np.int64)
        self.nx, self.ny = ix.max() + 1, iy.max() + 1
        self.H = self.nx * self.ny
        self.cell = iy * self.nx + ix
        gx, gy = np.meshgrid(self.x0 + (np.arange(self.nx) + .5) * cell_m, self.y0 + (np.arange(self.ny) + .5) * cell_m)
        self.cell_xy = np.c_[gx.ravel(), gy.ravel()]
        self.count = np.bincount(self.cell, minlength=self.H)
        self.z0 = d['z']
        self.med = float(np.median(self.z0))
        cuts = np.concatenate([[0], np.flatnonzero(np.diff(d['fid'])) + 1, [x.size]])
        self.seg = list(zip(cuts[:-1], cuts[1:]))
        self.frame_xy = np.array([(d['cx'][a], d['cy'][a]) for a, _ in self.seg])
        self.res0 = self._frame_residual(self.z0, range(len(self.seg)), np.full(x.size, np.nan))
        self.S0, self.N0 = self._sums(self.res0)
        self.joint = None
        if use_joint:
            fidx = np.repeat(np.arange(len(self.seg)), [b - a for a, b in self.seg])
            self.joint = FloorAdjustment(x, y, fidx, self.cell, (self.ny, self.nx), self.cell_xy, self.frame_xy, smooth=smooth)
            self.joint.solve(self.z0)

    def _frame_residual(self, z, frames, out):
        for f in frames:
            a, b = self.seg[f]
            out[a:b] = frame_plane_residual(self.d['x'][a:b], self.d['y'][a:b], z[a:b], *self.frame_xy[f])
        return out

    def _sums(self, vals):
        ok = ~np.isnan(vals)
        return (np.bincount(self.cell[ok], vals[ok], minlength=self.H), np.bincount(self.cell[ok], minlength=self.H))

    def _grid(self, v):
        return np.where(self.count >= 5, v, np.nan).reshape(self.ny, self.nx)

    def fields(self, zz=None, px=None, py=None, reach=None):
        """추정 방식별 높이장(ny, nx). zz 가 None 이면 주입 없는 기록. 주입이면 (px, py) 근처 프레임만 프레임 모드를 다시 계산함."""
        z = self.z0 if zz is None else zz
        raw = np.bincount(self.cell, z, minlength=self.H) / np.maximum(self.count, 1) - self.med
        if zz is None:
            S, N = self.S0, self.N0
        else:
            aff = np.nonzero(np.hypot(self.frame_xy[:, 0] - px, self.frame_xy[:, 1] - py) <= reach)[0]
            idx = np.concatenate([np.arange(*self.seg[f]) for f in aff])
            new = self._frame_residual(z, aff, np.full(z.size, np.nan))[idx]
            old = self.res0[idx]
            S, N = self.S0.copy(), self.N0.copy()
            for vals, sign in ((old, -1), (new, 1)):
                ok = ~np.isnan(vals)
                S += sign * np.bincount(self.cell[idx][ok], vals[ok], minlength=self.H)
                N += sign * np.bincount(self.cell[idx][ok], minlength=self.H)
        out = {'raw': self._grid(raw), 'frame': self._grid(S / np.maximum(N, 1))}
        if self.joint is not None:
            out['joint'] = self._grid(self.joint.solve(z))
        return out


def pick_locations(valid, cell_xy_grid, radius, n, rng):
    ys, xs = np.nonzero(valid)
    order = rng.permutation(ys.size)
    chosen = []
    for k in order:
        p = cell_xy_grid[ys[k], xs[k]]
        if all(np.hypot(*(p - q)) >= max(radius, 0.75) for _, _, q in chosen):
            chosen.append((ys[k], xs[k], p))
        if len(chosen) == n:
            break
    return chosen


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--diameters', default='50,100,150,200,250,300,400', help='돔 지름[cm], 쉼표로 구분')
    ap.add_argument('--n-locations', type=int, default=8, help='지름마다 돔을 주입할 위치 수')
    ap.add_argument('--height', type=float, default=0.02, help='이득을 구하는 주입 돔 높이[m]')
    ap.add_argument('--cell', type=float, default=0.125, help='격자 크기[m]')
    ap.add_argument('--smooth', type=float, default=0.2, help='joint 평활 벌점 가중치')
    ap.add_argument('--no-joint', action='store_true', help='joint 추정을 생략(빠름)')
    ap.add_argument('--max-ring', type=int, default=4)
    ap.add_argument('--r-min', type=float, default=1.1)
    ap.add_argument('--wall-margin', type=float, default=0.2, help='지도 벽에서 이 거리[m] 이내 점 제외')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--topology', default=os.path.join(L.HOME, 'maps', 'topology', 'final_topological_map.npz'))
    ap.add_argument('--map', default=os.path.join(L.HOME, 'maps', 'grid', 'map_from_dae.yaml'))
    ap.add_argument('--out-dir', default=os.path.join(L.HOME, 'visualization', 'surface_profiling'))
    args = ap.parse_args()

    path = L.resolve_npz(args.npz)
    log = L.load_frame_log(path)
    inside = L.load_node_lookup(args.topology)
    wall = L.load_wall_distance(args.map)
    mount_h = float(np.nanmean(log['poses'][used_frame_mask(log), 2]))
    r_max = L.ring_limit(mount_h, args.max_ring)
    d, _, _ = L.load_points(log, args.r_min, r_max, 0.06, lambda x, y: inside(x, y) & (wall(x, y) > args.wall_margin))
    print(f"[*] 점 {d['z'].size}개, 사용 거리 {args.r_min:.2f}-{r_max:.2f} m, 벽 {args.wall_margin} m 이내 제외")
    F = Fields(d, args.cell, not args.no_joint, args.smooth)
    base = F.fields()
    names = list(base)
    cell_grid = F.cell_xy.reshape(F.ny, F.nx, 2)
    wall_grid = wall(F.cell_xy[:, 0], F.cell_xy[:, 1]).reshape(F.ny, F.nx)
    lin_cases = []
    rng = np.random.default_rng(args.seed)

    rows, summary = [], {}
    for dcm in [float(v) for v in args.diameters.split(',')]:
        R = dcm / 200.0
        window, dome = kernels(R, args.cell)
        amp0, valid0 = {}, {}
        for nm in names:
            amp0[nm], valid0[nm] = amplitude_map(base[nm], window, dome)
        fits = wall_grid >= R
        for nm in names:
            valid0[nm] = valid0[nm] & fits
        common = np.logical_and.reduce([valid0[nm] for nm in names])
        locs = pick_locations(common, cell_grid, R, args.n_locations, rng)
        if not locs:
            print(f"D={dcm:.0f} cm: 유효한 위치 없음(돔이 관측 영역보다 큼)")
            continue
        if dcm >= 150:   # 선형성 확인용: 처음 유효한 지름과 가장 큰 지름의 첫 위치
            lin_cases = lin_cases[:1] + [(R, window, dome, locs[0], amp0)]
        gain = {nm: [] for nm in names}
        for (iy, ix, p) in locs:
            prof = L.dome(d, p[0], p[1], R, args.height)
            e1 = F.fields(d['z'] + prof, p[0], p[1], R + r_max + 0.05)
            for nm in names:
                a1, _ = amplitude_map(e1[nm], window, dome)
                gain[nm].append((a1[iy, ix] - amp0[nm][iy, ix]) / args.height)
        for nm in names:
            a0 = amp0[nm][valid0[nm]]
            t_max, t_q, sigma = a0.max(), np.quantile(a0, 0.999), a0.std()
            g = np.array(gain[nm])
            base_at = np.array([amp0[nm][iy, ix] for iy, ix, _ in locs])
            mdh = np.where(g > 0.05, np.maximum(t_max - base_at, 0.0) / np.maximum(g, 1e-9), np.inf)
            mdhq = np.where(g > 0.05, np.maximum(t_q - base_at, 0.0) / np.maximum(g, 1e-9), np.inf)
            row = dict(D_cm=dcm, estimator=nm, n_loc=len(locs), n_valid=int(valid0[nm].sum()), gain_med=float(np.median(g)),
                       gain_min=float(g.min()), gain_max=float(g.max()), sigma_mm=sigma * 1e3, t_max_mm=t_max * 1e3,
                       t_q_mm=t_q * 1e3, mdh_med_mm=float(np.median(mdh)) * 1e3, mdh_p90_mm=float(np.quantile(mdh, 0.9)) * 1e3,
                       mdhq_med_mm=float(np.median(mdhq)) * 1e3)
            rows.append(row)
            print(f"D={dcm:4.0f} cm {nm:6s} 위치 {len(locs)} 유효중심 {row['n_valid']:5d} | 이득 {row['gain_med']:.2f} "
                  f"[{row['gain_min']:.2f},{row['gain_max']:.2f}] | 배경 σ {row['sigma_mm']:.2f} T {row['t_max_mm']:.2f} mm | "
                  f"MDH 중앙 {row['mdh_med_mm']:.1f} p90 {row['mdh_p90_mm']:.1f} mm (T_q 기준 중앙 {row['mdhq_med_mm']:.1f})")

    for R, window, dome, (iy, ix, p), amp0 in lin_cases:
        for h in (0.01, 0.04):
            e1 = F.fields(d['z'] + L.dome(d, p[0], p[1], R, h), p[0], p[1], R + r_max + 0.05)
            gs = {nm: (amplitude_map(e1[nm], window, dome)[0][iy, ix] - amp0[nm][iy, ix]) / h for nm in names}
            print(f"[선형성] D={2 * R * 100:.0f} cm, h={h * 1e3:.0f} mm 주입 이득: " + ', '.join(f'{nm} {g:.2f}' for nm, g in gs.items()))

    tag = os.path.splitext(os.path.basename(path))[0].replace('frames_', '') + f'_h{args.height * 1e3:g}'
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, f'mdh_curve_{tag}.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    fig, ax = plt.subplots(1, 3, figsize=(17, 4.8))
    for nm in names:
        r = [q for q in rows if q['estimator'] == nm]
        D = [q['D_cm'] for q in r]
        ax[0].plot(D, [q['gain_med'] for q in r], marker='o', label=nm)
        ax[0].fill_between(D, [q['gain_min'] for q in r], [q['gain_max'] for q in r], alpha=0.15)
        ax[1].plot(D, [q['t_max_mm'] for q in r], marker='o', label=nm)
        ax[2].plot(D, [q['mdh_med_mm'] for q in r], marker='o', label=nm)
        ax[2].fill_between(D, [q['mdh_med_mm'] for q in r], [q['mdh_p90_mm'] for q in r], alpha=0.15)
    ax[2].axhline(15, color='gray', ls='--', lw=0.8)
    ax[0].set_title('amplitude gain (1 = fully kept)'); ax[1].set_title('zero-false-alarm threshold T [mm]')
    ax[2].set_title('minimum detectable height [mm] (median, band to p90)')
    for a in ax:
        a.set_xlabel('dome diameter [cm]'); a.legend(); a.grid(alpha=0.3)
    ax[2].set_yscale('log')
    png_path = os.path.join(args.out_dir, f'mdh_curve_{tag}.png')
    fig.savefig(png_path, dpi=80, bbox_inches='tight'); plt.close(fig)
    print(f"[+] saved: {png_path}\n[+] saved: {csv_path}")


if __name__ == '__main__':
    main()
