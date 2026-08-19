#!/usr/bin/env bash
# 큰 월드에서 누운 사람 관문 켬/끔을 비교한다.
#
# 왜 큰 월드에서 다시 재나
# ------------------------
# 작은 맵 48런에서 효과가 없어 관문 완화를 되돌렸는데, 작은 맵에는 누운
# 사람이 하나뿐이라 신호가 묻혔을 수 있다. 큰 월드 27런을 조난자별로
# 갈라 보니 못 찾는 두 명이 둘 다 누운 자세였다.
#
#   침대 남동(bed_s4)   11/27  41%
#   바닥 남서(lying_s2) 19/27  70%
#   나머지 다섯         89~100%
#
# 의도적으로 '가려지게' 배치한 조난자는 100% 였다. 어려운 배치는 잘 찾고
# 누운 자세를 못 찾는다. 7/7 완주가 절반에 그치는 이유가 이 둘이다.
#
# 지표는 이 두 명의 발견률이다. 전체 인원으로 보면 다섯 명이 거의 항상
# 잡혀 차이가 희석된다.
#
# 두 조건을 번갈아 돌린다. 몰아서 돌리면 중간에 뻗었을 때 뒤쪽 조건만
# 표본이 통째로 사라진다.
set -e
WS=${WS:-$HOME/ugv_ws}
DOM=${DOM:-224}
PART=${PART:-lb}
DUR=${DUR:-1500}
PAIRS=${PAIRS:-5}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

for i in $(seq 1 "$PAIRS"); do
  for cond in on off; do
    case $cond in
      on)  K=3; C=0.30 ;;      # 완화 적용
      off) K=6; C=0.50 ;;      # 지금 기본값
    esac
    bash "$HOME/kill_sim.sh" > /dev/null 2>&1 || true
    echo "[$(date +%H:%M)] lb $cond $i 시작 (관절 $K / 신뢰도 $C)"
    UGV_LYING_KPTS=$K UGV_LYING_CONF=$C UGV_ROOM_BONUS=0 \
      ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART \
      UGV_N_ROBOTS=2 UGV_BOUNDS=-27,-19,27,19 \
      setsid timeout --signal=INT --kill-after=30 "$DUR" \
      ros2 launch ugv_bringup multi_robot_sim.launch.py \
        world:=rescue_building_large headless:=true expected_victims:=7 \
        > "$HOME/lb${cond}_${i}.log" 2>&1 || true
    echo "[$(date +%H:%M)] lb $cond $i 종료"
  done
done
echo "lb 완료"
