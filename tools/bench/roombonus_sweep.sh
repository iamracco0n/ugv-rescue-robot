#!/usr/bin/env bash
# 방 우선 보너스 크기를 0 부터 올려 가며 잰다.
#
# 두 가지가 반대로 움직일 것이다.
#   보너스를 키우면 -> 방 재진입이 준다 (원하는 것)
#   너무 키우면     -> 방에서 못 나와 발견 인원이 준다 (예전 고장)
# 그 사이가 답이므로 양쪽을 같이 본다.
#
# 값 고르기: 경계 후보 점수는 n*res*view_r - lam*d 다. 실측에서 목표
# 거리가 대개 5~25m 였으므로 거리 벌점은 2.5~12.5m^2 범위다. 보너스가
# 그보다 훨씬 크면 거리가 무의미해지고, 훨씬 작으면 아무 영향이 없다.
# 그래서 0 / 3 / 6 / 12 로 훑는다.
#
# 순서를 라운드마다 돌린다. 고정이면 아침에 잘릴 때 늘 마지막 값만
# 표본이 빠지고, 머신이 더워지는 효과도 특정 값에 몰린다.
set -e
WS=${WS:-/home/user/ugv_ws/.claude/worktrees/patrol-inspect-explore}
NAME=${NAME:-rb}
DOM=${DOM:-220}
PART=${PART:-rb}
DUR=${DUR:-1500}
ROUNDS=${ROUNDS:-3}
OFFSET=${OFFSET:-0}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

VALS=(0 3 6 12)

for round in $(seq 1 "$ROUNDS"); do
  start=$(( (round - 1 + OFFSET) % 4 ))
  for i in 0 1 2 3; do
    b=${VALS[$(( (start + i) % 4 ))]}
    # 앞 런 찌꺼기를 통째로 죽인다. 런처만 죽이면 노드가 고아로 남고,
    # 그게 쌓이면 CPU 를 먹어 카메라가 끊기고 검출이 조용히 0 이 된다.
    bash "$HOME/kill_sim.sh" > /dev/null 2>&1 || true
    echo "[$(date +%H:%M)] $NAME round$round 보너스=$b 시작"
    UGV_ROOM_BONUS=$b ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART \
      UGV_N_ROBOTS=2 UGV_BOUNDS=-27,-19,27,19 \
      setsid timeout --signal=INT --kill-after=30 "$DUR" \
      ros2 launch ugv_bringup multi_robot_sim.launch.py \
        world:=rescue_building_large headless:=true expected_victims:=7 \
        > "$HOME/${NAME}_${round}_b${b}.log" 2>&1 || true
    echo "[$(date +%H:%M)] $NAME round$round 보너스=$b 종료"
  done
done
echo "$NAME 스윕 완료"
