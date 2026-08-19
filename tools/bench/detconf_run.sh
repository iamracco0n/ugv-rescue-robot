#!/usr/bin/env bash
# YOLO 박스 신뢰도 하한(det_conf)을 낮추면 누운 사람을 더 찾는지 본다.
#
# 왜 이 손잡이인가
# ----------------
# 검출이 두 단계다.
#   1단계  YOLO 가 사람 박스를 만든다        <- det_conf
#   2단계  그 박스의 관절점을 검사한다        <- lying_* 관문
#
# 이틀 동안 2단계를 만졌는데 아무 효과가 없었다. 이유는 실측으로 나왔다 —
# 누운 조난자를 놓친 런에서 '후보로 잡았는데 관문에서 떨어짐' 은 0건이고,
# 거의 전부 '후보로 잡은 적도 없음' 이었다. 1단계에서 이미 버려진 것은
# 2단계에 오지 않는다.
#
# 위험: det_conf 를 낮추면 벽·가구가 박스로 들어온다. 그건 2단계 관문이
# 막아야 하는데, 지금 그 관문이 놀고 있으니 오히려 제 역할을 하게 된다.
# 그래서 유령(오탐) 건수를 반드시 같이 본다.
#
#   low  : det_conf 0.30
#   high : det_conf 0.50 (지금 기본값)
set -e
WS=${WS:-$HOME/ugv_ws}
DOM=${DOM:-226}
PART=${PART:-dc}
DUR=${DUR:-1500}
PAIRS=${PAIRS:-5}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

for i in $(seq 1 "$PAIRS"); do
  for cond in low high; do
    case $cond in
      low)  D=0.30 ;;
      high) D=0.50 ;;
    esac
    bash "$HOME/kill_sim.sh" > /dev/null 2>&1 || true
    echo "[$(date +%H:%M)] dc $cond $i 시작 (det_conf $D)"
    UGV_DET_CONF=$D UGV_ROOM_BONUS=0 \
      ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART \
      UGV_N_ROBOTS=2 UGV_BOUNDS=-27,-19,27,19 \
      setsid timeout --signal=INT --kill-after=30 "$DUR" \
      ros2 launch ugv_bringup multi_robot_sim.launch.py \
        world:=rescue_building_large headless:=true expected_victims:=7 \
        > "$HOME/dc${cond}_${i}.log" 2>&1 || true
    echo "[$(date +%H:%M)] dc $cond $i 종료"
  done
done
echo "dc 완료"
