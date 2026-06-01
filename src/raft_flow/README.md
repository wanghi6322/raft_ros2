# raft_flow ROS2 패키지

FLIR Blackfly S 카메라 + RAFT Optical Flow ROS2 패키지입니다.

---

## 패키지 구조

```
raft_ws/
└── src/
    └── raft_flow/
        ├── package.xml                  ← ROS2 패키지 메타데이터
        ├── setup.py / setup.cfg         ← Python 빌드 설정
        ├── raft_flow/
        │   ├── flir_camera_node.py      ← FLIR 카메라 노드 (PySpin)
        │   └── raft_flow_node.py        ← RAFT Optical Flow 노드
        ├── launch/
        │   └── raft_flow.launch.py      ← 전체 실행 launch 파일
        └── config/
            └── params.yaml              ← 파라미터 설정 (해상도, fps, 모델 경로 등)
```

---

## 토픽 구조

```
[FLIR 카메라]
     ↓ PySpin
[flir_camera_node]
     ↓ /flir/image_raw (sensor_msgs/Image, mono8)
[raft_flow_node]
     ↓ /raft/flow        (sensor_msgs/Image, 32FC2)  ← SLAM/제어 노드에서 구독
     ↓ /raft/flow_viz    (sensor_msgs/Image, bgr8)   ← RViz 시각화용
```

---

## Jetson Orin AGX 설치 가이드

### 환경 요구사항
- JetPack 6.2.2
- ROS2 Humble
- Python 3.10

---

### 1단계 — ROS2 Humble 설치

```bash
# UTF-8 로케일 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS2 저장소 추가
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt install ros-humble-desktop
```

---

### 2단계 — Jetson용 PyTorch 설치

> ⚠️ 일반 pip install torch는 Jetson에서 동작하지 않습니다. 반드시 NVIDIA 제공 wheel을 사용하세요.

```bash
# pip 업그레이드
pip3 install --upgrade pip

# Jetson JetPack 6.x용 PyTorch (NVIDIA wheel)
# https://developer.nvidia.com/embedded/downloads 에서 최신 버전 확인
pip3 install torch torchvision --index-url https://developer.download.nvidia.com/compute/redist/jp/v62

# 설치 확인
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
# cuda.is_available()이 True여야 정상
```

---

### 3단계 — PySpin SDK 설치

> ⚠️ PySpin은 pip으로 설치되지 않습니다. FLIR 공식 사이트에서 다운로드해야 합니다.

1. https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-download/ 접속
2. **Spinnaker SDK for Linux ARM64 (JetPack)** 다운로드
3. 설치:

```bash
# 압축 해제 후 설치 스크립트 실행
tar -xvf spinnaker-*.tar.gz
cd spinnaker-*/
sudo sh install_spinnaker_arm.sh

# PySpin Python 바인딩 설치
pip3 install spinnaker_python-*-cp310-cp310-linux_aarch64.whl
```

---

### 4단계 — 기타 패키지 설치

```bash
pip3 install numpy opencv-python matplotlib
sudo apt install python3-colcon-common-extensions
```

---

### 5단계 — RAFT 모델 파일 복사

```bash
# 개발 PC에서 Jetson으로 전송 (scp 사용)
scp ~/raft/models/raft-things.pth user@jetson-ip:~/raft/models/

# 또는 USB로 직접 복사 후
mkdir -p ~/raft/models
cp /media/usb/raft-things.pth ~/raft/models/
```

---

### 6단계 — 패키지 빌드

```bash
# 작업공간 생성
mkdir -p ~/raft_ws/src
cd ~/raft_ws/src

# 패키지 복사 (USB 또는 scp로 전달받은 raft_flow 폴더를 여기에 넣기)

# 빌드
cd ~/raft_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 7단계 — params.yaml 설정 확인

`config/params.yaml` 열어서 모델 경로 확인:

```yaml
raft:
  # 기본 모델 (정확도/속도 균형)
  model_path: /home/user/raft/models/raft-things.pth

  # 다른 모델 테스트 시 위 경로만 변경:
  # model_path: /home/user/raft/models/raft-small.pth    ← 가볍고 빠름 (성능 부족 시)
  # model_path: /home/user/raft/models/raft-sintel.pth   ← 실내 환경에 강함
  # model_path: /home/user/raft/models/raft-kitti.pth    ← 야외/자율주행 환경에 강함

camera:
  width: 2048     # 해상도 조정 가능 (Orin 부하 클 경우 1024로 낮추기)
  height: 1536
  fps: 30.0       # fps 조정 가능 (부하 클 경우 15.0으로 낮추기)
```

---

## 실행 방법

```bash
# 매번 실행 전 소싱 (터미널 새로 열 때마다)
source /opt/ros/humble/setup.bash
source ~/raft_ws/install/setup.bash

# 전체 실행 (카메라 + RAFT 동시)
ros2 launch raft_flow raft_flow.launch.py

# 모델 경로 직접 지정 실행
ros2 launch raft_flow raft_flow.launch.py \
  model_path:=/home/user/raft/models/raft-things.pth

# 카메라 노드만 단독 실행
ros2 run raft_flow flir_camera_node

# RAFT 노드만 단독 실행 (카메라 노드가 별도로 실행 중일 때)
ros2 run raft_flow raft_flow_node
```

---

## 토픽 확인 명령어

```bash
# 현재 활성화된 토픽 목록
ros2 topic list

# 토픽 fps 확인
ros2 topic hz /raft/flow
ros2 topic hz /flir/image_raw

# 토픽 정보 확인
ros2 topic info /raft/flow
```

---

## SLAM/제어 노드에서 구독하는 방법

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

self.sub_flow = self.create_subscription(
    Image, '/raft/flow', self.flow_callback, 10)

def flow_callback(self, msg):
    flow = bridge.imgmsg_to_cv2(msg, '32FC2')  # shape: (H, W, 2)
    u = flow[..., 0]  # 수평 흐름 (pixels/frame)
    v = flow[..., 1]  # 수직 흐름 (pixels/frame)
```

---

## 성능 튜닝 가이드 (Orin 부하 시)

| 상황 | 해결 방법 |
|------|----------|
| fps가 너무 낮음 | `iters` 줄이기 (20 → 12 → 6) |
| 메모리 부족 | 해상도 낮추기 (2048x1536 → 1024x768) |
| 전체적으로 느림 | `raft-small.pth` 모델 사용 |
| 최대 성능 필요 | TensorRT 변환 (별도 가이드 참고) |

---

## 문의

패키지 관련 문의사항은 개발자에게 연락하세요.
