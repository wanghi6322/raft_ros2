#!/bin/bash
# RAFT Flow 전체 스택 실행 스크립트
# venv의 torch/PySpin을 시스템 Python에서도 찾을 수 있도록 PYTHONPATH를 설정합니다.
# (ros2 launch가 생성하는 엔트리포인트 스크립트의 shebang이 /usr/bin/python3로 고정되어 있어
#  venv 활성화만으로는 패키지를 인식하지 못하기 때문입니다.)

VENV_SITE=/home/hd/raft_venv/lib/python3.10/site-packages

source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"

export PYTHONPATH=${VENV_SITE}:${PYTHONPATH}

ros2 launch raft_flow raft_flow.launch.py "$@"
