# surface_profiling/utils/frame_recorder.py
"""
PointCloud2 프레임 단위 기록 계층.

`combined_*.pcd`는 voxel 다운샘플을 거친 결과라 "어느 시각에 어느 셀에 점이 몇 개
찍혔는지"를 알 수 없음. 이 모듈은 다운샘플 이전의 map 좌표 점을 프레임별로
남겨서, 주행 후에 (1) 누적 과정 영상 생성(`frame_video.py`), (2) 셀별 점 수/통과
횟수 분석(`analyze_frame_log.py`)에 쓸 수 있게 함.

주행 중에는 메모리에 append만 하고, 저장(.npz)은 수집이 끝난 뒤 한 번에 함.
바닥 분석에 쓸모없는 벽/천장 점으로 메모리가 커지지 않도록 z를 넓은 밴드
[z_min, z_max]로만 남기되, 바닥 z-window(floor_extractor)보다 충분히 넓게 잡아
"점은 있는데 z가 떠서 z-window에 잘리는 경우"도 나중에 구분할 수 있게 함.

.npz 구성 (F = 프레임 수, P = 저장된 점 총수):
  stamps      (F,)   float64  msg.header.stamp [s]
  poses       (F,4)  float64  velodyne_link의 map 기준 (x, y, z, yaw)
  flags       (F,)   uint8    비트: FLAG_CAPTURE, FLAG_REJECTED
  n_raw       (F,)   int32    필터링 전 프레임의 유효 점 수(점을 저장하지 않은 프레임은 0)
  offsets     (F+1,) int64    points[offsets[i]:offsets[i+1]]이 i번째 프레임의 점
  points      (P,3)  float32  map 좌표 점(z 밴드로만 잘림)
  meta_*      스칼라  only_capture_at_waypoints 등 기록 조건
"""

import math

import numpy as np


class FrameRecorder:
    FLAG_CAPTURE = 1    # 프레임이 들어올 때 capture_active였음
    FLAG_REJECTED = 2   # TF 저역통과 필터가 프레임을 기각함

    def __init__(self, z_min=-0.5, z_max=0.5, store_transit_points=False,
                 only_capture_at_waypoints=True):
        self.z_min = z_min
        self.z_max = z_max
        # False면 최종 결과에 반영될 수 있는 프레임(캡처 구간이거나 연속 수집 모드)의
        # 점만 저장하고, 이동(transit) 프레임은 pose만 남김.
        self.store_transit_points = store_transit_points
        self.only_capture_at_waypoints = only_capture_at_waypoints

        self._stamps = []
        self._poses = []
        self._flags = []
        self._n_raw = []
        self._counts = []
        self._points = []

    @staticmethod
    def pose_from_transform(trans):
        """TransformStamped -> (x, y, z, yaw). yaw는 쿼터니언에서 ZYX 오일러 yaw를 계산함."""
        t = trans.transform.translation
        q = trans.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, t.z, yaw)

    def add(self, stamp_sec, pose, points_map, capture_active, rejected):
        """프레임 1개를 기록함. points_map이 None이면 pose/flag만 남김."""
        flags = (self.FLAG_CAPTURE if capture_active else 0) | (self.FLAG_REJECTED if rejected else 0)
        self._stamps.append(stamp_sec)
        self._poses.append(pose)
        self._flags.append(flags)

        if points_map is None:
            self._n_raw.append(0)
            self._counts.append(0)
            return

        z = points_map[:, 2]
        kept = points_map[(z >= self.z_min) & (z <= self.z_max)].astype(np.float32, copy=False)
        self._n_raw.append(points_map.shape[0])
        self._counts.append(kept.shape[0])
        if kept.shape[0]:
            self._points.append(kept)

    def __len__(self):
        return len(self._stamps)

    def save(self, path):
        """누적된 기록을 .npz로 저장함. 프레임이 하나도 없으면 저장하지 않고 False를 반환함."""
        if not self._stamps:
            return False
        offsets = np.zeros(len(self._counts) + 1, dtype=np.int64)
        np.cumsum(self._counts, out=offsets[1:])
        points = np.vstack(self._points) if self._points else np.zeros((0, 3), dtype=np.float32)
        np.savez(
            path,
            stamps=np.asarray(self._stamps, dtype=np.float64),
            poses=np.asarray(self._poses, dtype=np.float64),
            flags=np.asarray(self._flags, dtype=np.uint8),
            n_raw=np.asarray(self._n_raw, dtype=np.int32),
            offsets=offsets,
            points=points,
            meta_only_capture_at_waypoints=np.bool_(self.only_capture_at_waypoints),
            meta_z_min=np.float64(self.z_min),
            meta_z_max=np.float64(self.z_max),
        )
        return True


def load_frame_log(path):
    """save()로 만든 .npz를 dict로 읽어 반환함."""
    with np.load(path) as f:
        return {k: f[k] for k in f.files}


def used_frame_mask(log):
    """최종 결과(all_points)에 실제로 반영됐을 프레임 마스크.

    기각되지 않았고(캡처 구간이거나 연속 수집 모드) 점이 저장된 프레임임."""
    flags = log['flags']
    rejected = (flags & FrameRecorder.FLAG_REJECTED) != 0
    captured = (flags & FrameRecorder.FLAG_CAPTURE) != 0
    in_scope = captured | (not bool(log['meta_only_capture_at_waypoints']))
    return in_scope & ~rejected
