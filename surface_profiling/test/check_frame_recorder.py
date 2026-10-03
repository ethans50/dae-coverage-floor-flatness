#!/usr/bin/env python3
# surface_profiling/test/check_frame_recorder.py
"""
utils/frame_recorder.py 의 intensity 기록 검증. 점과 intensity 의 순서/길이가 z 밴드 절단 뒤에도 맞고,
intensity 가 없는 프레임은 NaN 으로 남는지 확인함.

사용 예:
  python3 check_frame_recorder.py
"""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils.frame_recorder import FrameRecorder, load_frame_log  # noqa: E402


def main():
    rec = FrameRecorder(z_min=-0.5, z_max=0.5)
    pose = (0.0, 0.0, 0.3, 0.0)
    pts = np.array([[1, 0, 0.0], [2, 0, 0.9], [3, 0, 0.01]], dtype=np.float32)   # 두 번째 점은 z 밴드 밖
    rec.add(0.0, pose, pts, True, False, intensity=np.array([10, 20, 30], dtype=np.float32))
    rec.add(0.1, pose, pts, True, False)                                          # intensity 없는 프레임
    rec.add(0.2, pose, None, False, False)                                        # 점 없는 프레임
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'frames_test.npz')
        assert rec.save(path)
        log = load_frame_log(path)
    assert log['points'].shape == (4, 3) and log['intensity'].shape == (4,)
    assert log['intensity'][:2].tolist() == [10.0, 30.0], log['intensity']      # 밴드 밖 점의 intensity 가 같이 빠짐
    assert np.isnan(log['intensity'][2:]).all()
    assert log['offsets'].tolist() == [0, 2, 4, 4]
    print("OK  intensity 길이/순서/NaN 처리")


if __name__ == '__main__':
    main()
