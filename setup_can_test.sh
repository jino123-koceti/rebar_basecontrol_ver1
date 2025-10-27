#!/bin/bash
# CAN 인터페이스 테스트용 설정 스크립트

echo "🔧 CAN 인터페이스 설정 (테스트용)"
echo "================================"

# can0 설정
echo ""
echo "📌 can0 설정 (250Kbps)..."
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

# 상태 확인
echo ""
echo "✅ CAN 인터페이스 상태:"
ip -details link show can0

echo ""
echo "================================"
echo "🎉 CAN 설정 완료!"
echo ""
echo "다음 단계:"
echo "  1. candump can0          # CAN 메시지 확인"
echo "  2. Iron-MD 조종기 전원 ON"
echo "  3. ros2 launch rebar_control test_teleop.launch.py"
echo ""
