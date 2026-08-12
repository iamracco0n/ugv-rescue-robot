#!/usr/bin/env bash
# 카메라 토픽 수신율을 잰다.
#
#     DOM=218 bash tools/cam_hz.sh
#
# 왜 이걸 봐야 하나
# -----------------
# YOLO 노드는 RGB+깊이 동기화 구독이다. 한쪽이 끊기면 콜백이 안 돌고
# 검출이 0 이 되는데, 이 고장은 완전히 조용하다 — Xid 없음, nvidia-smi
# 정상, 탐사 목표도 계속 발행된다. 라이다와 Nav2 는 멀쩡해서 런이 '정상
# 종료' 되고, 눈만 먼 로그가 '못 찾음' 으로 집계에 섞인다.
#
# 실측으로 겪은 값:
#   정상   RGB 14.8Hz(편차 0.010)  깊이 14.9Hz(편차 0.008)  최대공백 0.14s
#   고장   RGB 13.4Hz              깊이  8.6Hz(편차 0.484)  최대공백 6.15s
#
# 원인은 앞선 런의 노드가 고아로 남아 CPU 를 먹은 것이었다(로드 27/16코어).
# tools/kill_sim.sh 로 정리한다.
WS=${WS:-$HOME/ugv_ws}
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${DOM:-0}
echo "=== RGB ==="
timeout 20 ros2 topic hz /camera/camera/color/image_raw 2>&1 | tail -2
echo "=== Depth ==="
timeout 20 ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw 2>&1 | tail -2
