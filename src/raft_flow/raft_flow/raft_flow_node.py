"""
RAFT Optical Flow ROS 2 노드

/flir/image_raw 토픽을 구독하고, 연속된 두 프레임으로 RAFT 광학 흐름을
추정하여 결과를 퍼블리시합니다. 카메라 제어는 flir_camera_node가 담당합니다.

Subscribed topics:
  /flir/image_raw      (sensor_msgs/Image, mono8)   - FLIR 카메라 원본 프레임

Published topics:
  /raft/flow           (sensor_msgs/Image, 32FC2)   - 광학 흐름 (ch0=u, ch1=v, 픽셀/프레임)
  /raft/flow_viz       (sensor_msgs/Image, bgr8)    - 컬러휠 시각화 (GPU 처리)
  /raft/quiver_viz     (sensor_msgs/Image, bgr8)    - Quiver 화살표 시각화 (CPU, 선택)

Parameters:
  raft_path        RAFT 소스 루트 경로 (core/ 폴더의 상위)
  model_path       모델 가중치 .pth 경로
  crop_width       센터 크롭 너비  (0 = 비활성, 8의 배수)
  crop_height      센터 크롭 높이  (0 = 비활성, 8의 배수)
  iters            RAFT 반복 횟수  (default: 20, 줄이면 속도↑ 정확도↓)
  small            RAFT-Small 모델 사용 여부
  mixed_precision  혼합 정밀도(FP16) 사용 여부
  alternate_corr   AlternateCorrBlock 사용 여부
  quiver_step      화살표 간격 픽셀
  quiver_scale     화살표 길이 배율
  quiver_thickness 화살표 두께
  publish_flow_viz /raft/flow_viz 퍼블리시 여부
  publish_quiver   /raft/quiver_viz 퍼블리시 여부 (CPU 부하 있음)
  frame_id         출력 메시지의 TF 프레임 ID
"""

import argparse
import os
import queue
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class RaftFlowNode(Node):
    """/flir/image_raw 프레임 쌍을 받아 RAFT로 광학 흐름을 추정·발행하는 노드."""

    def __init__(self):
        super().__init__('raft_flow_node')
        self._declare_params()

        # RAFT core 디렉토리를 Python 경로에 추가
        raft_path = self.get_parameter('raft_path').value
        sys.path.insert(0, os.path.join(raft_path, 'core'))

        # Jetson에서 CUDA 메모리 단편화 완화
        os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'expandable_segments:True'

        self._bridge = CvBridge()

        # /flir/image_raw 구독 (큐 크기 1: 항상 최신 프레임만 처리)
        self._sub = self.create_subscription(
            Image, 'flir/image_raw', self._image_callback, 1)

        self._pub_flow     = self.create_publisher(Image, 'raft/flow', 10)
        self._pub_flow_viz = self.create_publisher(Image, 'raft/flow_viz', 10)
        self._pub_quiver   = self.create_publisher(Image, 'raft/quiver_viz', 10)

        # 콜백 스레드 → 추론 스레드 간 프레임 전달 큐
        # maxsize=1: 추론이 느려도 최신 프레임만 유지하고 오래된 건 버림
        self._frame_queue: queue.Queue = queue.Queue(maxsize=1)

        self._stop_event = threading.Event()
        self._flow_thread = threading.Thread(
            target=self._flow_loop, daemon=True, name='raft_flow')
        self._flow_thread.start()

    # ──────────────────────────────────────────────
    #  파라미터
    # ──────────────────────────────────────────────

    def _declare_params(self):
        self.declare_parameter('raft_path',        '/home/hd/raft')
        self.declare_parameter('model_path',       '/home/hd/raft/models/raft-things.pth')
        self.declare_parameter('crop_width',       0)
        self.declare_parameter('crop_height',      0)
        self.declare_parameter('iters',            20)
        self.declare_parameter('small',            False)
        self.declare_parameter('mixed_precision',  True)
        self.declare_parameter('alternate_corr',   False)
        self.declare_parameter('quiver_step',      24)
        self.declare_parameter('quiver_scale',     2.0)
        self.declare_parameter('quiver_thickness', 1)
        self.declare_parameter('publish_flow_viz', True)
        self.declare_parameter('publish_quiver',   False)
        self.declare_parameter('frame_id',         'flir_camera')

    def _read_params(self) -> dict:
        return {
            'model_path':       self.get_parameter('model_path').value,
            'crop_width':       self.get_parameter('crop_width').value,
            'crop_height':      self.get_parameter('crop_height').value,
            'iters':            self.get_parameter('iters').value,
            'small':            self.get_parameter('small').value,
            'mixed_precision':  self.get_parameter('mixed_precision').value,
            'alternate_corr':   self.get_parameter('alternate_corr').value,
            'quiver_step':      self.get_parameter('quiver_step').value,
            'quiver_scale':     self.get_parameter('quiver_scale').value,
            'quiver_thickness': self.get_parameter('quiver_thickness').value,
            'pub_flow_viz':     self.get_parameter('publish_flow_viz').value,
            'pub_quiver':       self.get_parameter('publish_quiver').value,
            'frame_id':         self.get_parameter('frame_id').value,
        }

    # ──────────────────────────────────────────────
    #  이미지 구독 콜백 (ROS executor 스레드에서 실행)
    # ──────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """
        최신 프레임만 큐에 유지합니다.
        추론 스레드가 처리 중이면 오래된 프레임을 꺼내 버리고 새 것을 넣습니다.
        이렇게 해야 카메라 30fps 와 RAFT 추론 속도가 달라도 지연이 누적되지 않습니다.
        """
        try:
            self._frame_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._frame_queue.get_nowait()   # 오래된 프레임 버림
            except queue.Empty:
                pass
            self._frame_queue.put_nowait(msg)

    # ──────────────────────────────────────────────
    #  전처리 유틸
    # ──────────────────────────────────────────────

    @staticmethod
    def _center_crop(frame: np.ndarray, crop_w: int, crop_h: int) -> np.ndarray:
        """
        프레임 중앙에서 crop_w × crop_h 영역을 잘라냅니다.
        시작점을 8의 배수로 맞추는 이유: RAFT는 내부적으로 해상도가
        8의 배수여야 padding 없이 동작하기 때문입니다.
        """
        h, w = frame.shape[:2]
        if crop_w > w or crop_h > h:
            return frame
        start_x = ((w - crop_w) // 2 // 8) * 8
        start_y = ((h - crop_h) // 2 // 8) * 8
        return frame[start_y:start_y + crop_h, start_x:start_x + crop_w]

    @staticmethod
    def _mono_to_tensor(frame: np.ndarray, crop_w, crop_h, device):
        """
        mono8 numpy 배열 (H, W) → RAFT 입력 텐서 (1, 3, H, W).
        RAFT는 RGB 3채널을 기대하므로 그레이 채널을 3번 복제합니다.
        반환: (tensor, bgr_frame_for_quiver)
        """
        import torch
        if crop_w and crop_h:
            frame = RaftFlowNode._center_crop(frame, crop_w, crop_h)
        # (H, W) → (H, W, 3): 그레이를 RGB처럼 취급
        frame_rgb = np.stack([frame, frame, frame], axis=-1)
        tensor = torch.from_numpy(frame_rgb).permute(2, 0, 1).float()  # (3, H, W)
        return tensor[None].to(device), frame_rgb  # (1, 3, H, W), (H, W, 3)

    # ──────────────────────────────────────────────
    #  시각화 유틸 (GPU)
    # ──────────────────────────────────────────────

    @staticmethod
    def _make_colorwheel(device):
        """Middlebury 컬러 휠을 GPU 텐서로 생성합니다. 초기화 시 1회만 호출합니다."""
        import torch
        RY, YG, GC, CB, BM, MR = 15, 6, 4, 11, 13, 6
        ncols = RY + YG + GC + CB + BM + MR
        cw = torch.zeros((ncols, 3), dtype=torch.float32, device=device)
        col = 0
        cw[0:RY, 0] = 255
        cw[0:RY, 1] = torch.arange(0, RY, device=device) * 255 / RY
        col += RY
        cw[col:col+YG, 0] = 255 - torch.arange(0, YG, device=device) * 255 / YG
        cw[col:col+YG, 1] = 255
        col += YG
        cw[col:col+GC, 1] = 255
        cw[col:col+GC, 2] = torch.arange(0, GC, device=device) * 255 / GC
        col += GC
        cw[col:col+CB, 1] = 255 - torch.arange(0, CB, device=device) * 255 / CB
        cw[col:col+CB, 2] = 255
        col += CB
        cw[col:col+BM, 2] = 255
        cw[col:col+BM, 0] = torch.arange(0, BM, device=device) * 255 / BM
        col += BM
        cw[col:col+MR, 2] = 255 - torch.arange(0, MR, device=device) * 255 / MR
        cw[col:col+MR, 0] = 255
        return cw

    @staticmethod
    def _flow_to_color(flow_tensor, colorwheel) -> np.ndarray:
        """
        flow (1, 2, H, W) → BGR uint8 numpy (H, W, 3).
        GPU에서 정규화·색상 매핑까지 처리하고 마지막에 한 번만 CPU로 복사합니다.
        """
        import torch
        flo = flow_tensor[0].permute(1, 2, 0)   # (H, W, 2)
        u, v = flo[..., 0], flo[..., 1]
        rad = torch.sqrt(u ** 2 + v ** 2)
        rad_max = rad.max().clamp(min=1e-5)
        u, v, rad = u / rad_max, v / rad_max, rad / rad_max

        a  = torch.atan2(-v, -u) / np.pi
        fk = (a + 1) / 2 * (colorwheel.shape[0] - 1)
        k0 = fk.long()
        k1 = (k0 + 1) % colorwheel.shape[0]
        f  = fk - k0.float()

        img = torch.zeros((*u.shape, 3), dtype=torch.float32, device=flow_tensor.device)
        for ch in range(3):
            col0 = colorwheel[k0, ch] / 255.0
            col1 = colorwheel[k1, ch] / 255.0
            col  = (1 - f) * col0 + f * col1
            col  = torch.where(rad <= 1, 1 - rad * (1 - col), col * 0.75)
            img[..., ch] = (255 * col).clamp(0, 255)

        # RGB → BGR, GPU → CPU (한 번만 전송)
        return img[:, :, [2, 1, 0]].to(torch.uint8).cpu().numpy()

    @staticmethod
    def _draw_quiver(bgr: np.ndarray, flow_np: np.ndarray,
                     step: int, scale: float, thickness: int) -> np.ndarray:
        """flow_np (H, W, 2) 위에 화살표를 그려 반환합니다."""
        import cv2
        h, w = bgr.shape[:2]
        ys, xs = np.mgrid[step // 2:h:step, step // 2:w:step]
        ys = ys.reshape(-1).astype(int)
        xs = xs.reshape(-1).astype(int)
        vis = bgr.copy()
        for x1, y1, dx, dy in zip(xs, ys,
                                    flow_np[ys, xs, 0],
                                    flow_np[ys, xs, 1]):
            x2 = int(x1 + dx * scale + 0.5)
            y2 = int(y1 + dy * scale + 0.5)
            if 0 <= x2 < w and 0 <= y2 < h:
                cv2.arrowedLine(vis, (x1, y1), (x2, y2),
                                (0, 255, 0), thickness, tipLength=0.3)
        return vis

    # ──────────────────────────────────────────────
    #  RAFT 추론 루프 (백그라운드 스레드)
    # ──────────────────────────────────────────────

    def _flow_loop(self):
        import torch

        p = self._read_params()
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'추론 디바이스: {device}')

        # CUDA 없이 autocast(FP16)를 쓰면 CPU에서 오류가 발생하므로 강제 비활성화
        mixed_precision = p['mixed_precision'] and device == 'cuda'
        if p['mixed_precision'] and not mixed_precision:
            self.get_logger().warn('CUDA 없음 — mixed_precision을 강제로 False로 설정합니다.')

        # RAFT 모델 로드
        try:
            from raft import RAFT
        except ImportError as e:
            self.get_logger().error(f'RAFT import 실패 (raft_path 확인): {e}')
            return

        raft_args = argparse.Namespace(
            small=p['small'],
            mixed_precision=mixed_precision,
            alternate_corr=p['alternate_corr'],
        )
        # DataParallel로 감싸야 saved state_dict의 'module.' 키와 일치합니다
        model = torch.nn.DataParallel(RAFT(raft_args))
        try:
            model.load_state_dict(torch.load(p['model_path'], map_location=device, weights_only=True))
        except TypeError:
            # PyTorch < 2.0 은 weights_only 인수를 지원하지 않음
            model.load_state_dict(torch.load(p['model_path'], map_location=device))
        # DataParallel 껍데기를 제거하고 단일 GPU 모델로 사용
        model = model.module.to(device).eval()
        torch.backends.cudnn.benchmark = True  # 고정 해상도에서 속도 최적화

        colorwheel = self._make_colorwheel(device)
        self.get_logger().info(f'RAFT 모델 로드 완료: {p["model_path"]}')

        crop_w = p['crop_width']  if p['crop_width']  > 0 else None
        crop_h = p['crop_height'] if p['crop_height'] > 0 else None

        prev_tensor = None   # 이전 프레임 텐서 (GPU 상주)

        with torch.no_grad():
            while not self._stop_event.is_set():
                # 새 프레임 대기 (1초 타임아웃 → 종료 신호 체크)
                try:
                    msg: Image = self._frame_queue.get(timeout=1.0)
                except queue.Empty:
                    continue

                # sensor_msgs/Image (mono8) → numpy (H, W)
                frame = self._bridge.imgmsg_to_cv2(msg, 'mono8')
                curr_tensor, frame_rgb = self._mono_to_tensor(
                    frame, crop_w, crop_h, device)

                # 첫 프레임은 이전 프레임으로만 저장하고 추론 건너뜀
                if prev_tensor is None:
                    prev_tensor = curr_tensor
                    continue

                try:
                    # RAFT 추론: prev → curr 방향의 dense flow 계산
                    _, flow_up = model(prev_tensor, curr_tensor,
                                      iters=p['iters'], test_mode=True)
                    torch.cuda.synchronize()
                except Exception as e:
                    self.get_logger().error(f'RAFT 추론 오류: {e}')
                    prev_tensor = curr_tensor
                    continue

                header = Header(stamp=self.get_clock().now().to_msg(),
                                frame_id=p['frame_id'])

                # /raft/flow  — 32FC2: ch0=u(수평), ch1=v(수직), 단위: 픽셀/프레임
                flow_np = flow_up[0].permute(1, 2, 0).cpu().numpy().astype(np.float32)
                flow_msg = self._bridge.cv2_to_imgmsg(flow_np, encoding='32FC2')
                flow_msg.header = header
                self._pub_flow.publish(flow_msg)

                # /raft/flow_viz  — GPU 컬러휠 시각화
                if p['pub_flow_viz']:
                    viz = self._flow_to_color(flow_up, colorwheel)
                    viz_msg = self._bridge.cv2_to_imgmsg(viz, encoding='bgr8')
                    viz_msg.header = header
                    self._pub_flow_viz.publish(viz_msg)

                # /raft/quiver_viz  — CPU 화살표 시각화 (기본 비활성)
                if p['pub_quiver']:
                    import cv2
                    bgr = cv2.cvtColor(frame_rgb[..., 0], cv2.COLOR_GRAY2BGR) \
                          if frame_rgb.shape[2] == 3 \
                          else frame_rgb
                    bgr = cv2.cvtColor(frame_rgb[:, :, 0], cv2.COLOR_GRAY2BGR)
                    quiver = self._draw_quiver(bgr, flow_np,
                                               p['quiver_step'],
                                               p['quiver_scale'],
                                               p['quiver_thickness'])
                    quiver_msg = self._bridge.cv2_to_imgmsg(quiver, encoding='bgr8')
                    quiver_msg.header = header
                    self._pub_quiver.publish(quiver_msg)

                prev_tensor = curr_tensor

    # ──────────────────────────────────────────────
    #  종료 처리
    # ──────────────────────────────────────────────

    def destroy_node(self):
        self._stop_event.set()
        self._flow_thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RaftFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
