#!/usr/bin/env bash
# 시뮬 한 벌을 통째로 정리한다.
#
# 왜 노드까지 하나하나 죽이나
# ---------------------------
# ros2 launch 를 죽여도 그 아래 노드들은 고아로 살아남는다. 오늘 여러 번
# kill -9 를 쓰면서 런마다 한 벌씩 쌓였고, 오로라에 5벌이 붙어 CPU 를
# 다 먹었다(로드 27 / 16코어). 그러자 깊이 영상이 6초씩 끊겨 RGB+깊이
# 동기화가 깨졌고 YOLO 검출이 0 이 됐다.
#
# 무서운 건 이 고장이 조용하다는 점이다 — Xid 없음, nvidia-smi 정상,
# 탐사 목표도 계속 발행된다. 라이다와 Nav2 는 멀쩡해서 런이 정상 종료되고,
# 눈만 먼 로그가 '못 찾음' 으로 집계에 섞인다. 실제로 이걸 GPU 고장으로
# 오진했다.
#
# 패턴에 괄호를 넣는 이유
# -----------------------
# pkill -f 는 자기 자신의 명령줄도 검사한다. ssh 로 'pkill -f yolo_pose_node'
# 를 보내면 그 원격 셸의 명령줄에 그 문자열이 들어 있어 스스로를 죽이고,
# 뒤 명령이 실행되지 않는다. 실제로 그래서 정리가 절반만 됐다.
# 'yolo_pose_nod[e]' 로 쓰면 정규식은 같은 것을 찾지만 문자열은 달라진다.
NODES="fire_detection_nod[e] yolo_pose_nod[e] patrol_navigato[r]
       target_manager_nod[e] tf_relay_nod[e] map_merge_nod[e]
       visibility_overlay_nod[e] controller_serve[r] planner_serve[r]
       behavior_serve[r] bt_navigato[r] waypoint_followe[r]
       smoother_serve[r] velocity_smoothe[r] lifecycle_manage[r]
       slam_toolbo[x] robot_state_publishe[r] parameter_bridg[e]"

pkill -f "patrol_si[m]" 2>/dev/null
pkill -f "multi_robot_si[m]" 2>/dev/null
sleep 3
for n in $NODES; do pkill -f "$n" 2>/dev/null; done
sleep 3
pkill -f "gz si[m]" 2>/dev/null
sleep 2
for n in $NODES; do pkill -9 -f "$n" 2>/dev/null; done
pkill -9 -f "gz si[m]" 2>/dev/null
sleep 2

left=0
for n in $NODES "rub[y]"; do
  # pgrep -c 는 못 찾아도 0 을 찍고 종료코드 1 을 낸다.
  # '|| echo 0' 을 붙이면 0 이 두 줄이 되어 산술 비교가 깨진다.
  c=$(pgrep -c -f "$n" 2>/dev/null); c=${c:-0}
  [ "$c" -gt 0 ] && { echo "남음: $n x$c"; left=$((left + c)); }
done
[ "$left" -eq 0 ] && echo "정리 완료 — 남은 프로세스 없음"
exit 0
