#!/usr/bin/env bash
# 작은 맵(조난자 3명, 그중 하나는 바닥에 누움)을 1대로 반복해 돌린다.
#
# 로봇 1대인 이유: 오로라의 3080 은 카메라 2대를 렌더하다 Xid 16 으로
# 두 번 죽었다. 1대는 몇 주간 멀쩡히 돌리던 부하다.
set -e
WS=${WS:-$HOME/ugv_ws}
NAME=${NAME:-lying}
DOM=${DOM:-213}
PART=${PART:-lying}
DUR=${DUR:-900}
RUNS=${RUNS:-5}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

for i in $(seq 1 "$RUNS"); do
  pkill -f "patrol_si[m]" 2>/dev/null || true
  sleep 8
  echo "[$(date +%H:%M)] $NAME run$i 시작"
  ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART \
    setsid timeout --signal=INT --kill-after=30 "$DUR" \
    ros2 launch ugv_bringup patrol_sim.launch.py \
      world:=rescue_building headless:=true expected_victims:=3 \
      > "$HOME/${NAME}_${i}.log" 2>&1 || true
  echo "[$(date +%H:%M)] $NAME run$i 종료"
done
echo "$NAME 완료"
