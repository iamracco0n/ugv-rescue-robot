#!/usr/bin/env bash
# 2대 구성이 시간을 넉넉히 주면 전원 발견(7/7)까지 가는지 본다.
#
# 왜 필요한가
# -----------
# 스윕(1500초)에서 7/7 달성이 값에 관계없이 절반 수준이었다(기본값 3/6).
# 그런데 1500초에서 끊긴 것이라 '시간이 모자란 것' 인지 '원리적으로 못
# 찾는 곳이 있는 것' 인지 구분이 안 된다. 처방이 완전히 다르다.
#
#   시간 문제면   -> 더 빨리 훑게 만들면 된다
#   못 찾는 문제면 -> 안 가는/안 보는 자리를 찾아 고쳐야 한다
#
# 그래서 제한을 2400초로 늘려 같은 설정으로 돌린다. 여기서도 미달성이면
# 시간이 아니라 커버리지 문제다.
#
# 방 보너스는 0(끔) — 지금 기본값 그대로의 완성도를 재는 것이 목적이다.
set -e
WS=${WS:-$HOME/ugv_ws}
NAME=${NAME:-cp}
DOM=${DOM:-223}
PART=${PART:-cp}
DUR=${DUR:-2400}
RUNS=${RUNS:-3}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

for i in $(seq 1 "$RUNS"); do
  bash "$HOME/kill_sim.sh" > /dev/null 2>&1 || true
  echo "[$(date +%H:%M)] $NAME run$i 시작 (제한 ${DUR}s)"
  UGV_ROOM_BONUS=0 ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART \
    UGV_N_ROBOTS=2 UGV_BOUNDS=-27,-19,27,19 \
    setsid timeout --signal=INT --kill-after=30 "$DUR" \
    ros2 launch ugv_bringup multi_robot_sim.launch.py \
      world:=rescue_building_large headless:=true expected_victims:=7 \
      > "$HOME/${NAME}_${i}.log" 2>&1 || true
  echo "[$(date +%H:%M)] $NAME run$i 종료"
done
echo "$NAME 완료"
