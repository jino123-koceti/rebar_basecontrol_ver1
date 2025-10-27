#!/bin/bash
# Iron-MD 텔레옵 테스트 시작 스크립트

echo "🎮 Iron-MD 텔레옵 테스트"
echo "======================="
echo ""

# 워크스페이스 소싱
cd ~/ros2_ws_backup
source install/setup.bash

echo "✅ ROS2 워크스페이스 로드됨"
echo ""
echo "📋 테스트 환경:"
echo "  - CAN 인터페이스: can0"
echo "  - 디버그 모드: 활성화"
echo "  - 모터: 없음 (터미널 출력만)"
echo ""
echo "🚀 텔레옵 노드 시작..."
echo "━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 런치 파일 실행
ros2 launch rebar_control test_teleop.launch.py
