#!/usr/bin/env bash
# 로봇별로 검출 파이프라인이 살아 있는지 본다.
#
# 왜 로봇별로 봐야 하나
# ---------------------
# gate_reject.sh 는 로그 전체의 기각 건수를 센다. 2대 런에서 한 대만 눈이
# 멀면 다른 대의 기각이 잡혀 런이 정상처럼 보인다. 실제로 그런 런이 있었다 —
# ugv2 가 목표 28회를 내고도 구조 0건이었는데 로그 전체로는 기각이 있었다.
#
# 한 대가 눈먼 채로 자기 구역을 훑으면 그 구역 조난자는 통째로 못 찾는다.
# 2대 구성에서 가장 조용하고 치명적인 고장이다.
for f in "$@"; do
  echo "=== $(basename "$f" .log) ==="
  for r in ugv1 ugv2; do
    kpt=$(grep -a "$r\.yolo_pose_node" "$f" 2>/dev/null \
          | grep -ao '키포인트=[0-9]*' | cut -d= -f2 \
          | awk '{s+=$1} END {print s+0}')
    det=$(grep -ac "$r\.target_manager.*후보 발견" "$f" 2>/dev/null)
    reg=$(grep -ac "$r\.target_manager.*구조 로그" "$f" 2>/dev/null)
    printf '  %-6s 키포인트기각 %-6s 후보 %-4s 등록 %s\n' "$r" "$kpt" "$det" "$reg"
  done
done
