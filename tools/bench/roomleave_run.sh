#!/usr/bin/env bash
# 큰 월드 2대로 돌리며 '방 이탈' 계측을 모은다.
#
# 지표가 둘이다.
#   · 방 이탈 횟수 / 남긴 미관측 면적  ← 이번에 새로 재는 것
#   · 1200초 시점 발견 인원            ← 기존 지표. 이탈을 줄이려다 사람을
#                                       덜 찾으면 소용없으므로 같이 본다.
set -e
WS=${WS:-/home/user/ugv_ws/.claude/worktrees/patrol-inspect-explore}
NAME=${NAME:-rl}
DOM=${DOM:-215}
PART=${PART:-rl}
DUR=${DUR:-1500}
RUNS=${RUNS:-4}
EXTRA=${EXTRA:-}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

for i in $(seq 1 "$RUNS"); do
  pkill -f "multi_robot_si[m]" 2>/dev/null || true
  sleep 10
  echo "[$(date +%H:%M)] $NAME run$i 시작"
  env $EXTRA ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART UGV_N_ROBOTS=2 \
    UGV_BOUNDS=-27,-19,27,19 \
    setsid timeout --signal=INT --kill-after=30 "$DUR" \
    ros2 launch ugv_bringup multi_robot_sim.launch.py \
      world:=rescue_building_large headless:=true expected_victims:=7 \
      > "$HOME/${NAME}_${i}.log" 2>&1 || true
  echo "[$(date +%H:%M)] $NAME run$i 종료"
done
echo "$NAME 완료"
