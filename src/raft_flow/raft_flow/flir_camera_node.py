"""
FLIR Blackfly S 카메라 ROS 2 드라이버 노드

PySpin(Spinnaker SDK)으로 FLIR 카메라를 초기화·설정·스트리밍하고
/flir/image_raw 토픽으로 그레이스케일 프레임을 발행합니다.

Published topics:
  /flir/image_raw  (sensor_msgs/Image, mono8)  - 그레이스케일 원본 프레임

Parameters:
  width       캡처 해상도 너비   (default: 2048)
  height      캡처 해상도 높이   (default: 1536)
  fps         목표 프레임레이트  (default: 30.0)
  frame_id    TF 프레임 ID       (default: flir_camera)
"""

import threading

import PySpin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class FlirCameraNode(Node):
    """FLIR Blackfly S 카메라로부터 프레임을 캡처해 /flir/image_raw 로 발행하는 노드."""

    def __init__(self):
        super().__init__('flir_camera_node')
        self._declare_params()

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, 'flir/image_raw', 10)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    # ──────────────────────────────────────────────
    #  파라미터
    # ──────────────────────────────────────────────

    def _declare_params(self):
        self.declare_parameter('width', 2048)
        self.declare_parameter('height', 1536)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'flir_camera')

    def _read_params(self) -> dict:
        return {
            'width':    self.get_parameter('width').value,
            'height':   self.get_parameter('height').value,
            'fps':      self.get_parameter('fps').value,
            'frame_id': self.get_parameter('frame_id').value,
        }

    # ──────────────────────────────────────────────
    #  카메라 설정
    # ──────────────────────────────────────────────

    def _configure_camera(self, cam, target_w: int, target_h: int, target_fps: float):
        """
        카메라 해상도·FPS·버퍼 모드를 설정합니다.

        왜 Offset → Width/Height 순서인가:
          Width/Height를 크게 늘릴 때 기존 Offset이 남아있으면
          '센서 경계 초과' 에러가 발생합니다. 그러므로 Offset을 0으로
          초기화한 뒤 크기를 조정하고, 마지막에 센터 Offset을 설정합니다.
        """
        nodemap = cam.GetNodeMap()

        node_offset_x = PySpin.CIntegerPtr(nodemap.GetNode('OffsetX'))
        node_offset_y = PySpin.CIntegerPtr(nodemap.GetNode('OffsetY'))
        node_width    = PySpin.CIntegerPtr(nodemap.GetNode('Width'))
        node_height   = PySpin.CIntegerPtr(nodemap.GetNode('Height'))

        # 1단계: Offset 초기화
        if PySpin.IsWritable(node_offset_x):
            node_offset_x.SetValue(0)
        if PySpin.IsWritable(node_offset_y):
            node_offset_y.SetValue(0)

        # 2단계: 해상도 설정
        if PySpin.IsWritable(node_width) and node_width.GetMax() >= target_w:
            node_width.SetValue(target_w)
        else:
            self.get_logger().warn(
                f'Width {target_w} 설정 불가 (최대: {node_width.GetMax()})')

        if PySpin.IsWritable(node_height) and node_height.GetMax() >= target_h:
            node_height.SetValue(target_h)
        else:
            self.get_logger().warn(
                f'Height {target_h} 설정 불가 (최대: {node_height.GetMax()})')

        # 3단계: 센터 Offset
        actual_w = node_width.GetValue()
        actual_h = node_height.GetValue()
        center_x = (node_width.GetMax() - actual_w) // 2
        center_y = (node_height.GetMax() - actual_h) // 2
        if PySpin.IsWritable(node_offset_x):
            node_offset_x.SetValue(center_x)
        if PySpin.IsWritable(node_offset_y):
            node_offset_y.SetValue(center_y)

        self.get_logger().info(
            f'해상도: {actual_w}×{actual_h}  OffsetX={center_x}  OffsetY={center_y}')

        # 4단계: FPS 설정
        fps_enable = PySpin.CBooleanPtr(nodemap.GetNode('AcquisitionFrameRateEnable'))
        if PySpin.IsWritable(fps_enable):
            fps_enable.SetValue(True)

        node_fps = PySpin.CFloatPtr(nodemap.GetNode('AcquisitionFrameRate'))
        if PySpin.IsWritable(node_fps):
            capped_fps = min(target_fps, node_fps.GetMax())
            node_fps.SetValue(capped_fps)
            self.get_logger().info(f'FPS: {node_fps.GetValue():.1f}')

        # 5단계: 트리거 모드 Off
        # 트리거 모드가 켜져 있으면 외부 신호가 올 때까지 프레임을 전송하지 않아
        # GetNextImage 가 타임아웃됩니다. 이전 설정이 남아 있을 수 있으므로 강제 Off.
        try:
            if PySpin.IsWritable(cam.TriggerMode):
                cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
                self.get_logger().info('트리거 모드: Off')
        except Exception as e:
            self.get_logger().warn(f'트리거 모드 설정 실패: {e}')

        # 6단계: 픽셀 포맷 → Mono8
        # 카메라 기본 포맷이 BayerRG8 등 컬러 포맷이면 ImageProcessor.Convert 가
        # 실패할 수 있습니다. Mono8로 명시해 변환 없이 직접 수신합니다.
        try:
            if PySpin.IsWritable(cam.PixelFormat):
                cam.PixelFormat.SetValue(PySpin.PixelFormat_Mono8)
                self.get_logger().info('픽셀 포맷: Mono8')
        except Exception as e:
            self.get_logger().warn(f'픽셀 포맷 설정 실패: {e}')

        # 7단계: 버퍼 모드 → NewestOnly (최신 프레임만 유지, 지연 최소화)
        try:
            s_nodemap = cam.GetTLStreamNodeMap()
            handling_mode = PySpin.CEnumerationPtr(
                s_nodemap.GetNode('StreamBufferHandlingMode'))
            if PySpin.IsReadable(handling_mode) and PySpin.IsWritable(handling_mode):
                entry = PySpin.CEnumEntryPtr(handling_mode.GetEntryByName('NewestOnly'))
                handling_mode.SetIntValue(entry.GetValue())
                self.get_logger().info('버퍼 모드: NewestOnly')
        except Exception as e:
            self.get_logger().warn(f'버퍼 모드 설정 실패: {e}')

    # ──────────────────────────────────────────────
    #  캡처 루프 (백그라운드 스레드)
    # ──────────────────────────────────────────────

    def _capture_loop(self):
        p = self._read_params()

        system   = PySpin.System.GetInstance()
        cam_list = system.GetCameras()

        if cam_list.GetSize() == 0:
            self.get_logger().error('FLIR 카메라를 찾을 수 없습니다.')
            cam_list.Clear()
            system.ReleaseInstance()
            return

        cam = cam_list.GetByIndex(0)
        cam.Init()
        self.get_logger().info(f'카메라 연결: {cam.DeviceModelName.GetValue()}')

        self._configure_camera(cam, p['width'], p['height'], p['fps'])

        cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
        cam.BeginAcquisition()

        processor = PySpin.ImageProcessor()
        processor.SetColorProcessing(
            PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

        frame_id = p['frame_id']
        self.get_logger().info('/flir/image_raw 발행 시작')

        try:
            while not self._stop_event.is_set():
                try:
                    image_result = cam.GetNextImage(3000)  # 3000 ms 타임아웃
                except PySpin.SpinnakerException as e:
                    self.get_logger().warn(f'프레임 취득 실패: {e}')
                    continue

                if image_result.IsIncomplete():
                    image_result.Release()
                    continue

                # Mono8으로 변환 후 numpy 배열 복사
                converted = processor.Convert(image_result, PySpin.PixelFormat_Mono8)
                frame = converted.GetNDArray().copy()
                image_result.Release()

                # ROS 2 Image 메시지로 변환 후 발행
                msg = self._bridge.cv2_to_imgmsg(frame, encoding='mono8')
                msg.header = Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=frame_id,
                )
                self._pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'캡처 루프 오류: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info('카메라 자원 해제 중...')
            try:
                cam.EndAcquisition()
                cam.DeInit()
            except Exception:
                pass
            del cam
            cam_list.Clear()
            system.ReleaseInstance()

    # ──────────────────────────────────────────────
    #  종료 처리
    # ──────────────────────────────────────────────

    def destroy_node(self):
        self._stop_event.set()
        self._thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlirCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
