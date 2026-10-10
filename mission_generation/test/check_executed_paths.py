"""coverage_check.executed_paths의 코너 컷 예측 자체 점검: 사용: python3 mission_generation/test/check_executed_paths.py"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mission_planning.utils import coverage_check as cc  # noqa: E402


def wp(x, y, yaw, kind):
    return {'pose': {'position': {'x': x, 'y': y}, 'orientation': {'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}},
            'header': {'task_type': kind, 'record_pcd': True}}


# A(0,0) -> B(0,3) -> C(3,3): B에서 90도 꺾임. 보조점 없으면 B가 있는 sub-segment [B, C]에서 B가 반경 0.7 m 앞에서 지워져 코너가 잘림.
cut = cc.executed_paths([wp(0, -3, math.pi / 2, 'coverage_start'), wp(0, 0, math.pi / 2, 'coverage_turn'),
                         wp(0, 3, 0.0, 'coverage_turn'), wp(3, 3, 0.0, 'coverage_end')])
reach = cc.executed_paths([wp(0, -3, math.pi / 2, 'coverage_start'), wp(0, 2.9, math.pi / 2, 'coverage_turn'),
                           wp(0, 3, 0.0, 'coverage_turn'), wp(3, 3, 0.0, 'coverage_end')])
near = lambda g: min(float(math.hypot(*(p - (0, 3)))) for p in g['xy'])
assert max(near(g) for g in cut) > 0.6, 'corner should be cut'
assert min(near(g) for g in reach) < 0.15, 'corner should be reached with the helper point'
print('ok')
