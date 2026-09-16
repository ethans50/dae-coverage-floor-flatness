# surface_profiling/tf_sync_mixin.py
"""
SurfaceProfiler의 TF-PointCloud 동기화 담당 믹스인.

map->odom(AMCL)이 velodyne_points보다 느려서 단순 lookup_transform은 대부분
실패함. rclpy에는 C++ 전용인 tf2_ros.MessageFilter가 없으므로
`Buffer.wait_for_transform_async` 코루틴으로 같은 효과를 직접 구현함 - 메시지를
바로 처리하지 않고 그 stamp를 커버하는 TF가 도착한 뒤에 처리함. 여기에 프레임
단위 저역통과 필터(순간 선속도/각속도 임계)를 얹어 AMCL 점프 등으로 튄 프레임을
버림. 자세한 동작은 DETAILS.md §7, 저역통과 기준값 리셋 경위는 HISTORY.md §11 참고.

믹스인으로 둔 이유는 `nav2_drive_mixin.py`와 같음 - 포인트 버퍼
(`all_points`/`current_waypoint_points`), 캡처 상태(`capture_active`), 저역통과
기준값(`last_tf_*`)을 SurfaceProfiler와 그대로 공유해야 해서 협력 객체로 빼면
양쪽이 서로를 참조하게 됨.

SurfaceProfiler 쪽에 다음이 있다고 전제함: `profiling_cfg`, `is_sim`,
`spin_executor`, `stop_requested`, `capture_active`, `only_capture_at_waypoints`,
`all_points`, `current_waypoint_points`, `enable_tf_lowpass_filter`,
`tf_lowpass_max_linear_vel`, `tf_lowpass_max_angular_vel_deg`, `last_tf_*`.
"""

import math

import numpy as np
import torch
import tf_transformations
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class TfSyncMixin:
    """TF 준비를 기다렸다가 PointCloud2를 map 좌표계로 변환해 적재함."""

    # ------------------------------------------------------------------
    # TF / PointCloud2 구독 설정 
    # ------------------------------------------------------------------

    def _setup_tf(self):
        self.target_frame = self.profiling_cfg.get('target_frame', 'map')
        self.source_frame = self.profiling_cfg.get('source_frame', 'velodyne_link')
        self.tf_timeout_sec = self.profiling_cfg.get('tf_timeout_sec', 0.1)

        self.tf_buffer = Buffer()
        # [변경] spin_thread=True를 쓰지 않는다(기본값 False).
        # 예전 코드(lookup_transform을 콜백 안에서 timeout까지 블로킹 대기)에서는
        # tf 구독을 별도 스레드로 분리하는 게 필수였지만, 지금은
        # wait_for_transform_async(코루틴, await로 양보)로 바뀌어서 콜백이
        # 스레드를 블로킹하지 않음. 따라서 tf 구독도 self.spin_executor
        # 하나로 충분히 처리되고, 별도 스레드/executor가 없으니 노드 종료
        # 시점에 "아직 살아있는 백그라운드 스레드 vs rclpy.shutdown()" 같은
        # 레이스 컨디션(ExternalShutdownException)도 원천적으로 사라짐.
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _setup_pointcloud_subscription(self):
        self.voxel_size = self.profiling_cfg.get('voxel_size', 0.01)
        # [EVAL 준비] 알고리즘 비교 실험에서 다운샘플 이전 raw 포인트가 필요함
        # (셀당 다중 리턴 수/z-표준편차 계산용) - 기본은 false로 평소 실행에는
        # 영향 없음. EVAL.md 참고.
        self.save_raw_pcd = self.profiling_cfg.get('save_raw_pcd', False)

        topic_name = (
            self.profiling_cfg.get('pcd_topic_sim', '/velodyne_points')
            if self.is_sim
            else self.profiling_cfg.get('pcd_topic_real', '/velodyne_points')
        )
        self.get_logger().info(f"Execution Mode: {'Simulation' if self.is_sim else 'Real Hardware'}")
        self.get_logger().info(f"Subscribing to topic: {topic_name}")

        # GPU 사용 가능 여부 확인
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        # [TF-PointCloud 동기화 구조]
        # map->odom(AMCL, 실측 약 5Hz)이 velodyne_points(10Hz)보다 느려서, 단순
        # lookup_transform(exact_stamp, timeout)만으로는 tf2가 그 시각을 보간할
        # 다음 샘플이 아직 안 왔다는 이유로 대부분 실패한다(timeout을 늘려도 무의미).
        #
        # tf2_ros.MessageFilter는 C++ 전용 API라 rclpy(Python)에는 존재하지 않으므로,
        # 여기서는 Buffer.wait_for_transform_async(코루틴)를 이용해 동일한 효과를
        # 직접 구현함: 메시지를 즉시 처리하지 않고, 해당 stamp를 커버하는 TF가
        # 실제로 도착할 때까지 코루틴으로 기다렸다가 준비되면 콜백을 실행함.
        # 대기 중에도 콜백/실행기는 블로킹되지 않고(await로 양보), 그동안 새로 들어오는
        # 메시지도 계속 큐에 쌓임. _pending_queue_maxlen은 TF 최대 주기(약 0.5s)
        # 동안 들어오는 pointcloud 개수(~5개)보다 넉넉하게 잡음.
        # 대기 중인(TF 준비를 기다리는) 메시지에 대한 참조 보관용.
        # 자동 만료(maxlen) 없이 완료 시점에만 명시적으로 제거한다 —
        # deque(maxlen=N)을 쓰면 아직 처리 중인 태스크의 메시지가
        # 강제로 밀려나 예기치 않게 끊길 수 있기 때문.
        self._pending_pc_msgs = set()

        self.pc_subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self._pc_enqueue_callback,
            10
        )

    def _pc_enqueue_callback(self, msg: PointCloud2):
        """PointCloud2 메시지를 즉시 처리하지 않고, TF 준비를 기다리는 비동기 태스크로 등록만 함."""
        if self.stop_requested:
            return
        self._pending_pc_msgs.add(id(msg))
        # 코루틴을 태스크로 등록 -> executor가 spin하는 동안 백그라운드에서 진행됨.
        self.spin_executor.create_task(self._wait_and_process(msg))

    async def _wait_and_process(self, msg: PointCloud2):
        """해당 msg의 stamp를 커버하는 TF가 준비될 때까지 기다린 뒤 처리함."""
        try:
            await self.tf_buffer.wait_for_transform_async(
                self.target_frame,
                self.source_frame,
                msg.header.stamp,
            )
        except Exception as e:
            self.get_logger().warn(f"TF 대기 실패 원인: {e}")
            return
        finally:
            self._pending_pc_msgs.discard(id(msg))

        self._pc_callback(msg)

    def _tf_passes_lowpass_filter(self, trans: TransformStamped, msg_stamp):
        """
        연속된 두 PointCloud2 프레임 사이의 TF(target_frame->source_frame) 순간
        선속도/각속도를 계산해서, params.yaml의 임계치(tf_lowpass_max_linear_vel,
        tf_lowpass_max_angular_vel_deg)를 넘으면 False(버림)를 반환함.

        기준값(last_tf_*)은 "통과한" 프레임에서만 갱신함. 튄 프레임을 다음
        비교의 새 기준으로 삼아버리면, 그 다음 정상 프레임까지 연쇄적으로 오탐
        처리될 수 있기 때문임.
        """
        if not self.enable_tf_lowpass_filter:
            return True

        curr_t = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ])
        curr_q = trans.transform.rotation
        curr_yaw = 2.0 * math.atan2(curr_q.z, curr_q.w)
        curr_stamp = msg_stamp.sec + msg_stamp.nanosec * 1e-9

        if self.last_tf_translation is None:
            self.last_tf_translation = curr_t
            self.last_tf_yaw = curr_yaw
            self.last_tf_stamp = curr_stamp
            return True

        dt = curr_stamp - self.last_tf_stamp
        if dt <= 1e-4:
            # 타임스탬프가 동일하거나 역행함(비교 불가). 기준은 갱신하지 않고 일단 통과.
            return True

        linear_vel = np.linalg.norm(curr_t - self.last_tf_translation) / dt

        dyaw = curr_yaw - self.last_tf_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # -pi ~ +pi 정규화
        angular_vel_deg = abs(math.degrees(dyaw)) / dt

        passes = (linear_vel <= self.tf_lowpass_max_linear_vel and
                  angular_vel_deg <= self.tf_lowpass_max_angular_vel_deg)

        if passes:
            self.last_tf_translation = curr_t
            self.last_tf_yaw = curr_yaw
            self.last_tf_stamp = curr_stamp
        else:
            self.get_logger().warn(
                f"[TF Low-pass] Frame rejected: linear_vel={linear_vel:.2f}m/s "
                f"(limit {self.tf_lowpass_max_linear_vel}), "
                f"angular_vel={angular_vel_deg:.1f}deg/s "
                f"(limit {self.tf_lowpass_max_angular_vel_deg})."
            )

        return passes

    def _pc_callback(self, msg: PointCloud2):
        """_wait_and_process에서 TF 준비가 확인된 뒤에만 호출됨."""
        if self.stop_requested:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                msg.header.stamp,
            )
        except Exception as e:
            # wait_for_transform_async 통과 직후라 정상적으로는 거의 발생하지 않지만,
            # 버퍼 캐시 만료(오래된 TF가 밀려난 경우) 등 극히 드문 경합에 대비한 안전망.
            self.get_logger().warn(f"TF 변환 실패 원인: {e}")
            return

        # TF 저역통과 필터: 직전 프레임 대비 순간 선속도/각속도가 임계치를 넘으면
        # (AMCL 점프, 회전 중 잔여 프레임 등) 이 프레임 전체를 버림. 새 주행
        # 설계(mission_executor.py)가 회전 중엔 애초에 캡처를 켜지 않지만, 이건
        # 그래도 남을 수 있는 잔여 오차에 대한 방어선(defense in depth)임.
        if not self._tf_passes_lowpass_filter(trans, msg.header.stamp):
            return

        # PointCloud2 → numpy array 변환
        raw_points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))
        points_np = np.array([(p[0], p[1], p[2]) for p in raw_points], dtype=np.float32)
        if len(points_np) == 0:
            return

        # 행렬 연산을 위해 GPU(또는 CPU)로 전송
        points_t = torch.tensor(points_np, device=self.device)
        trans_mat_t = torch.tensor(self._transform_to_matrix(trans), device=self.device, dtype=torch.float32)

        # Homogeneous transformation
        ones = torch.ones((points_t.shape[0], 1), device=self.device)
        points_homo = torch.cat([points_t, ones], dim=1)

        # (4, 4) @ (4, N) 연산 후 전치
        transformed_t = (trans_mat_t @ points_homo.T).T

        # 결과 저장 (최종 저장 시에만 CPU로 복사)
        transformed_np = transformed_t[:, :3].cpu().numpy()

        if self.only_capture_at_waypoints:
            # 정지-캡처 구간(capture_active=True)의 포인트만 적재함.
            # 이동(transit) 중 수집된 포인트는 모션 블러/타임스탬프 오차 우려로 버림.
            if self.capture_active:
                self.current_waypoint_points.append(transformed_np)
        else:
            # 기존 동작(전 구간 연속 수집) 유지. 캡처 구간 포인트는 부가적으로
            # 지점별 버퍼에도 동시에 적재해서 별도 PCD로도 저장할 수 있게 함.
            self.all_points.append(transformed_np)
            if self.capture_active:
                self.current_waypoint_points.append(transformed_np)

    def _transform_to_matrix(self, trans: TransformStamped):
        t = [trans.transform.translation.x,
             trans.transform.translation.y,
             trans.transform.translation.z]
        q = [trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w]
        mat = tf_transformations.quaternion_matrix(q)
        mat[:3, 3] = t
        return mat
