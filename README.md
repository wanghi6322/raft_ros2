# raft_ros2

FLIR 카메라(PySpin) + RAFT Optical Flow를 ROS 2 노드로 패키징한 워크스페이스입니다.

---

## 테스트 환경

| 항목 | 내용 |
|------|------|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | **Humble Hawksbill** |
| Python | **3.10.12** (시스템 Python) |
| PyTorch | **2.5.1+cu121** |
| OpenCV | 4.13.0 |
| 가상환경 | **Python venv** (`--system-site-packages`) |
| GPU | NVIDIA GeForce RTX 4090 (테스트), 타겟: Jetson Orin AGX (JetPack 6.2.2) |

가상환경 venv 써서 테스트 했습니다.
그리고 desktop에서 테스트 시
raft_node와 RAFT 추론 모두 raft_venv 환경의 PyTorch를 사용해서 테스트했습니다.
### 가상환경 구성 방식

ROS 2 노드(`flir_camera_node`, `raft_flow_node`) **모두 venv 기반**으로 실행됩니다.

ROS 2의 엔트리포인트 스크립트가 `/usr/bin/python3`(시스템 Python)으로 고정 실행되기 때문에,  
venv를 직접 활성화하는 방식은 동작하지 않습니다. 대신 `launch_raft_flow.sh`에서  
`PYTHONPATH`를 직접 주입하는 방식을 사용합니다.

```bash
# launch_raft_flow.sh 핵심 구조
export PYTHONPATH=/path/to/raft_venv/lib/python3.10/site-packages:${PYTHONPATH}
ros2 launch raft_flow raft_flow.launch.py
```

즉, **conda는 사용하지 않으며**, venv의 `torch`, `PySpin` 등 패키지를  
시스템 Python이 `PYTHONPATH`를 통해 참조하는 구조입니다.

### RAFT 원본 수정 여부

**RAFT 원본 코드를 수정하지 않았습니다.**  
`raft_flow_node.py`에서 `sys.path`에 RAFT `core/` 디렉토리를 추가해  
원본 그대로 import합니다.
raft 원본 깃허브
https://github.com/princeton-vl/raft
에서 RAFT 모델 가중치와 소스코드 직접 받아야 합니다.
```python
sys.path.insert(0, os.path.join(raft_path, 'core'))
from raft import RAFT  # ~/raft/core/raft.py 원본 그대로 사용
```

---

## 패키지 구조

```
raft_ros2/
├── src/
│   ├── raft_flow/              # 메인 ROS 2 노드 패키지 (ament_python)
│   │   ├── raft_flow/
│   │   │   ├── flir_camera_node.py   # FLIR 카메라 드라이버 노드
│   │   │   └── raft_flow_node.py     # RAFT 광학 흐름 추론 노드
│   │   ├── config/
│   │   │   └── params.yaml           # 파라미터 설정 파일
│   │   ├── launch/
│   │   │   └── raft_flow.launch.py   # 전체 스택 launch 파일
│   │   ├── requirements.txt          # 의존성 설치 가이드
│   │   └── package.xml
│   └── raft_flow_msgs/         # 커스텀 메시지 패키지 (ament_cmake)
│       └── msg/
│           └── FlowMean.msg          # 평균 flow 벡터 메시지 정의
├── launch_raft_flow.sh         # PYTHONPATH 포함 실행 스크립트
└── .gitignore
```

---

## 데이터 흐름 및 토픽

```
[FLIR 카메라]
      │ USB 3.0
      ▼
┌─────────────────────┐
│  flir_camera_node   │
└─────────────────────┘
      │
      │  /flir/image_raw
      │  sensor_msgs/Image
      │  encoding: mono8 (그레이스케일)
      │  크기: 카메라 설정 해상도
      ▼
┌─────────────────────┐
│   raft_flow_node    │
└─────────────────────┘
      │              │              │
      ▼              ▼              ▼
/raft/flow_mean  /raft/flow_viz  /raft/quiver_viz
```

### 발행 토픽 상세

| 토픽 | 메시지 타입 | 설명 |
|------|------------|------|
| `/flir/image_raw` | `sensor_msgs/Image` (mono8) | FLIR 카메라 원본 그레이스케일 프레임 |
| `/raft/flow_mean` | `raft_flow_msgs/FlowMean` | 프레임 전체 픽셀의 **평균** 광학 흐름 벡터 |
| `/raft/flow_viz` | `sensor_msgs/Image` (bgr8) | flow 방향·크기를 색상으로 표현한 컬러휠 시각화 |
| `/raft/quiver_viz` | `sensor_msgs/Image` (bgr8) | flow를 화살표로 표현한 시각화 (기본 비활성) |

> `/raft/flow` (픽셀별 dense flow, 32FC2) 는 코드에 주석으로 보존되어 있습니다.  
> 필요 시 `raft_flow_node.py`에서 주석 해제하면 활성화됩니다.

### FlowMean 메시지 구조

```
std_msgs/Header header    # ROS time 타임스탬프 + frame_id
float32 u                 # 수평 방향 평균 광학 흐름 (단위: 픽셀/프레임, 오른쪽 양수)
float32 v                 # 수직 방향 평균 광학 흐름 (단위: 픽셀/프레임, 아래쪽 양수)
```

---

## 사용자가 반드시 수정해야 하는 파라미터

`src/raft_flow/config/params.yaml` 파일을 환경에 맞게 수정합니다.

### `flir_camera_node` 파라미터

```yaml
flir_camera_node:
  ros__parameters:
    width:  2048    # ← 카메라 최대 해상도 이하로 설정 (카메라 모델마다 다름)
    height: 1536    # ← 카메라 최대 해상도 이하로 설정
    fps:    30.0    # ← 카메라 지원 최대 FPS 이하로 설정
```

### `raft_flow_node` 파라미터

```yaml
raft_flow_node:
  ros__parameters:
    raft_path:  "/home/hd/raft"                          # ← RAFT 소스 경로 (반드시 수정)
    model_path: "/home/hd/raft/models/raft-things.pth"  # ← 모델 가중치 경로 (반드시 수정)

    crop_width:  1024   # ← 센터 크롭 너비 (8의 배수, 0이면 크롭 안 함)
    crop_height: 768    # ← 센터 크롭 높이 (8의 배수, 0이면 크롭 안 함)
    iters: 20           # ← RAFT 반복 횟수 (줄이면 속도↑ 정확도↓, 권장: 12~20)
```

### 모델 선택 가이드

| 모델 파일 | 특징 |
|-----------|------|
| `raft-things.pth` | 범용 (기본 권장) |
| `raft-sintel.pth` | 실내/일반 환경에 강함 |
| `raft-kitti.pth` | 야외/자율주행 환경에 강함 |
| `raft-small.pth` | 경량 모델, 속도 우선 시 사용 |

---

## 설치 및 실행

### 1. 의존성 설치

```bash
# ROS 2 패키지 (apt)
sudo apt install ros-humble-cv-bridge ros-humble-desktop

# Python venv 생성 (시스템 ROS 2 패키지 상속)
python3 -m venv ~/raft_venv --system-site-packages
source ~/raft_venv/bin/activate

# PyTorch 설치
# Jetson: NVIDIA 공식 wheel 사용 (requirements.txt 참고)
# 데스크탑(x86): pip install torch --index-url https://download.pytorch.org/whl/cu121

pip install numpy opencv-python-headless scipy matplotlib

# PySpin: Spinnaker SDK 설치 후 wheel 설치 (requirements.txt 참고)
```

### 2. FLIR 카메라 설정 (최초 1회)

#### 2-1. Spinnaker SDK 설치
FLIR 공식 사이트에서 JetPack용 Spinnaker SDK 다운로드 후 설치합니다.
```bash
tar -xvf spinnaker-*.tar.gz
cd spinnaker-*/
sudo sh install_spinnaker_arm.sh   # Jetson (ARM64)
```

#### 2-2. PySpin Python 바인딩 설치
FLIR 공식 사이트에서 Python 버전에 맞는 wheel 파일 다운로드 후 설치합니다.
```bash
pip install spinnaker_python-*-cp310-cp310-linux_aarch64.whl
```

#### 2-3. USB 버퍼 크기 설정
고해상도 카메라 사용 시 프레임 타임아웃 방지를 위해 필요합니다.
```bash
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

#### 2-4. 카메라 USB 권한 설정
```bash
sudo groupadd flirimaging
sudo usermod -aG flirimaging $USER
sudo udevadm control --reload-rules && sudo udevadm trigger
# 이후 로그아웃 → 재로그인
```

#### 2-5. 카메라 연결 확인

USB 연결 확인:
```bash
lsusb | grep -i "Point Grey\|FLIR\|Teledyne"
# 출력 예: Bus 002 Device 004: ID 1e10:3300 Point Grey Research, Inc.
```

PySpin으로 카메라 인식 확인:
```python
import PySpin
system = PySpin.System.GetInstance()
cam_list = system.GetCameras()
print(f"감지된 카메라 수: {cam_list.GetSize()}")  # 1 이상이면 정상
cam_list.Clear()
system.ReleaseInstance()
```

> `lsusb`에는 보이는데 카메라 수가 0이면 권한 문제입니다. 로그아웃 → 재로그인 후 다시 확인하세요.

### 3. 빌드

```bash
cd ~/raft_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 4. 실행

RAFT 원본 모델과 소스 코드 다운 받았다면
launch_raft_flow.sh 안에서 수정할 곳은 딱 한 줄입니다.

# 이 줄에서 본인의 venv 경로로만 바꾸면 됩니다
VENV_SITE=/home/hd/raft_venv/lib/python3.10/site-packages

그리고 params.yaml에서도 두 곳을 수정해야 합니다.

raft_path:  "/home/hd/raft"                          # ← 본인의 RAFT 경로
model_path: "/home/hd/raft/models/raft-things.pth"  # ← 본인의 모델 경로
즉 launch_raft_flow.sh 1줄 + params.yaml 2줄, 총 3줄만 본인 환경에 맞게 고치면 바로 실행 가능합니다.
```bash
# launch_raft_flow.sh 안의 PYTHONPATH 경로를 자신의 venv 경로로 수정 후:
~/raft_ws/launch_raft_flow.sh
```

### 5. 토픽 확인

```bash
source /opt/ros/humble/setup.bash
source ~/raft_ws/install/setup.bash

ros2 topic list
ros2 topic hz /flir/image_raw      # 카메라 FPS 확인
ros2 topic hz /raft/flow_mean      # RAFT 추론 속도 확인
ros2 topic echo /raft/flow_mean    # 데이터 확인
```

---

## 다른 노드에서 구독하는 방법

```python
from raft_flow_msgs.msg import FlowMean

self.sub = self.create_subscription(
    FlowMean, '/raft/flow_mean', self.flow_callback, 10)

def flow_callback(self, msg):
    u = msg.u   # 수평 평균 흐름 (픽셀/프레임)
    v = msg.v   # 수직 평균 흐름 (픽셀/프레임)
    stamp = msg.header.stamp  # ROS time
```

---

## 성능 튜닝 (Jetson Orin AGX 기준)

| 상황 | 조치 |
|------|------|
| FPS가 낮음 | `iters` 줄이기 (20 → 12) |
| 메모리 부족 | `crop_width/height` 줄이기 |
| 더 빠른 추론 필요 | `raft-small.pth` + `mixed_precision: true` |
