#!/usr/bin/env bash
# 누운 사람 관문 완화를 A/B 로 잰다.
#
# 두 조건을 번갈아 돌린다
# ----------------------
# 한 조건을 몰아서 돌리면, 오로라가 중간에 Xid 로 죽었을 때 뒤쪽 조건만
# 표본이 통째로 없어진다. 어젯밤 실제로 GPU 가 6시간 만에 죽었다.
# 번갈아 돌리면 어디서 끊겨도 두 조건이 비슷하게 남는다.
#
# 조건은 파라미터로만 가른다 — 다시 빌드하지 않으므로 두 조건이 같은
# 바이너리다. 빌드 차이가 결과에 섞일 여지를 없앤다.
#
#   fix  : 완화 적용 (관절 3개 / 신뢰도 0.30)
#   base : 완화 이전 (관절 6개 / 신뢰도 0.50)
set -e
WS=${WS:-$HOME/ugv_ws}
DOM=${DOM:-213}
PART=${PART:-lyab}
DUR=${DUR:-900}
PAIRS=${PAIRS:-10}        # 조건당 런 수

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

for i in $(seq 1 "$PAIRS"); do
  for cond in fix base; do
    case $cond in
      fix)  K=3; C=0.30 ;;
      base) K=6; C=0.50 ;;
    esac
    # 앞 런의 찌꺼기를 확실히 죽인다.
    # OMEN 에서 가제보 서버가 19시간·9시간씩 살아남아 CPU 를 7% 씩 먹고
    # 있었다. 새 런이 그 위에서 돌면 로봇이 굼떠 목표를 못 내고, 결과는
    # 조용히 나빠진다. ros2 launch 를 죽여도 gz 서버는 따로 남는다.
    pkill -f "patrol_si[m]" 2>/dev/null || true
    sleep 5
    pkill -f "gz sim" 2>/dev/null || true
    sleep 3
    pkill -9 -f "gz sim" 2>/dev/null || true
    sleep 2
    echo "[$(date +%H:%M)] lyab $cond $i 시작 (관절 $K / 신뢰도 $C)"
    UGV_LYING_KPTS=$K UGV_LYING_CONF=$C \
      ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART \
      setsid timeout --signal=INT --kill-after=30 "$DUR" \
      ros2 launch ugv_bringup patrol_sim.launch.py \
        world:=rescue_building headless:=true expected_victims:=3 \
        > "$HOME/lyab${cond}_${i}.log" 2>&1 || true
    echo "[$(date +%H:%M)] lyab $cond $i 종료"
  done
done
echo "lyab 완료"
