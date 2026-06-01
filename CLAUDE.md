# CLAUDE.md — `~/raft_ws/` ROS2 워크스페이스

RAFT Optical Flow를 ROS2 노드로 래핑한 워크스페이스. FLIR 카메라(PySpin) + RAFT 추론 결과를 ROS2 토픽으로 publish.
전역 규칙(`~/CLAUDE.md`)을 상속합니다.

---

## 1. 워크스페이스 구조

```
~/raft_ws/
├── src/
│   └── raft_flow/
│       ├── raft_flow/               # Python 패키지 (노드 코드)
│       ├── launch/                  # launch 파일
│       ├── config/                  # 파라미터 YAML
│       ├── resource/
│       ├── package.xml              # ROS2 패키지 매니페스트
│       ├── setup.py
│       └── setup.cfg
├── build/                            # colcon build 산출물 (자동 생성)
├── install/                          # colcon install 산출물 (자동 생성)
└── log/                              # colcon log
```

### 패키지 정보 (`src/raft_flow/package.xml`)
- **이름**: `raft_flow`
- **버전**: 0.1.0
- **설명**: RAFT Optical Flow node using FLIR camera via PySpin
- **빌드 타입**: `ament_python`
- **의존성**: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`

---

## 2. ROS2 환경

- **배포판**: ROS2 **Humble** (Ubuntu 22.04)
- **DDS**: Cyclone DDS 유니캐스트 설정 가능 (`~/cyclone_unicast.xml`)
  ```bash
  export CYCLONEDDS_URI=file:///home/hd/cyclone_unicast.xml
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

---

## 3. Python 환경

- 노드 실행 시 **시스템 Python(3.10) + ROS2 의존성**을 우선 사용합니다.
- RAFT 추론을 위한 PyTorch는 conda `raft` 환경의 라이브러리를 참조하거나, 시스템 venv에 별도 설치합니다.
- conda 환경과 ROS2를 함께 쓰면 `ament_python` 빌드가 꼬일 수 있으므로 **purely python ROS2 노드는 시스템 환경에서 실행**하는 것을 우선 검토하세요.

> 정확한 PyTorch / RAFT 의존성은 작업 시작 시 `pip list` 로 직접 확인합니다.

---

## 4. 빌드 / 실행 패턴

```bash
# (1) ROS2 환경 source
source /opt/ros/humble/setup.bash

# (2) 워크스페이스로 이동 후 빌드
cd ~/raft_ws
colcon build --symlink-install            # 개발 중에는 symlink로 (수정 즉시 반영)

# (3) overlay source
source install/setup.bash

# (4) 노드 실행 (launch 파일이 있는 경우)
ros2 launch raft_flow <launch_file>.launch.py

# 또는 ros2 run
ros2 run raft_flow <executable>
```

### 토픽 확인
```bash
ros2 topic list
ros2 topic echo /raft_flow/<topic>
ros2 topic hz   /raft_flow/<topic>
```

---

## 5. 코드 작성 규칙 (프로젝트 고유)

1. **노드 클래스에 한 줄 요약 docstring**:
   ```python
   class RaftFlowNode(Node):
       """FLIR 카메라 프레임을 받아 RAFT로 optical flow를 추정하고 sensor_msgs/Image로 publish하는 노드."""
   ```

2. **콜백 함수 안의 무거운 연산(추론)은 별도 스레드 또는 비동기 처리** 권장.
   - `MultiThreadedExecutor` 또는 별도 worker thread 사용.

3. **이미지 메시지 변환**은 항상 `cv_bridge.CvBridge` 통해 처리.
   - encoding 명시 (`"bgr8"`, `"mono8"`, `"32FC2"` 등).
   - flow 시각화 결과는 `bgr8`, 원본 flow(2채널 float)는 `32FC2`.

4. **파라미터는 `config/*.yaml` 로 외부화**합니다. 코드 내 매직 넘버 금지.

5. **launch 파일에 단일 진입점**을 둡니다. ex: `raft_flow.launch.py`.

---

## 6. PySpin / Spinnaker SDK

FLIR(Teledyne) 산업용 카메라 사용에 필요합니다. Python 패키지가 아니라 **시스템 SDK + Python wheel** 형태입니다.

```bash
# (1) Spinnaker SDK 시스템 설치 (FLIR/Teledyne 공식 사이트에서 .tar.gz 받은 뒤)
sudo ./install_spinnaker.sh

# (2) USB 권한 설정 (재부팅 권장)
sudo /opt/spinnaker/bin/configure_usbfs.sh

# (3) Python 바인딩 설치 (현재 Python 버전에 맞는 whl 사용)
pip install spinnaker_python-*.whl
```

> 정확한 SDK / Python 버전 호환은 작업 시작 시 FLIR 공식 페이지에서 확인.

---

## 7. 디버깅 팁

- **`colcon build` 실패 시**: `rm -rf build/ install/ log/` 후 재빌드 (사용자 확인 후).
- **메시지 타입을 못 찾을 때**: `source install/setup.bash` 누락 가능성.
- **노드는 떠 있는데 토픽이 안 나올 때**: DDS 도메인 ID (`ROS_DOMAIN_ID`) 확인.
- **PySpin 권한 에러**: `sudo /opt/spinnaker/bin/configure_usbfs.sh` 또는 udev rule 확인.
- **`PySpin not found`**: 시스템 Spinnaker SDK 설치 + Python whl 둘 다 필요.

---

## 8. 주의사항

- 이 워크스페이스는 `~/raft/` 의 RAFT 코드를 **모듈로 import 하지 않고 복제/포팅**된 형태일 가능성이 높습니다.
  - RAFT 모델 코드가 양쪽에 중복 존재할 수 있으니, 수정 시 어느 쪽을 기준으로 할지 사용자에게 확인.
- `build/`, `install/`, `log/` 폴더는 colcon 자동 생성물 — 절대 직접 편집 금지.
- **빌드 중간 산출물 삭제는 항상 사용자 승인 후**.
