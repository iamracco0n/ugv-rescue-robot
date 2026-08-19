#!/usr/bin/env bash
# 누운 조난자 검출만 집중해서 잰다 (동선 운 제거).
#
# 왜 이렇게 재나
# --------------
# 앞선 A/B 는 결과가 런마다 뒤집혔다. 오로라 fix 0/3 base 3/3, OMEN 은
# fix 3/3 base 2/3 으로 정반대였고, 오로라는 오전(fix 4/5)과 오후(fix 0/3)
# 에 자기 자신과도 모순됐다.
#
# 원인은 머신이 아니라 지표였다. '900초 안에 Room D 에 갔나' 가 결과를
# 지배했다 — 가면 관문과 무관하게 찾고, 안 가면 무조건 못 찾는다. 관문
# 효과가 그 운에 파묻혔다.
#
# 그래서 로봇을 누운 사람과 같은 방(Room D)에서 시작시킨다. 반드시
# 마주치므로 '갔나' 가 빠지고 '봤을 때 통과시키나' 만 남는다.
#
#   Room D : x -1~14, y -10~-4   문은 x 5~7 (y=-4)
#   누운 사람 : 원점 (10,-7), 몸 중심 약 (9.2,-7.4)
#   잔해_7 : (7,-7.5) 1.0x0.8   <- 스폰 자리는 여기를 피한다
#   스폰   : (2,-7)  같은 방, 사람에서 7m, 잔해와 안 겹침
#
# 두 조건을 번갈아 돌린다. 한 조건을 몰아 돌리면 중간에 죽었을 때 뒤쪽만
# 표본이 사라진다(오로라가 실제로 GPU 로 죽은 적 있다).
set -e
WS=${WS:-$HOME/ugv_ws}
DOM=${DOM:-216}
PART=${PART:-lyfoc}
DUR=${DUR:-420}
PAIRS=${PAIRS:-8}

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
    # 앞 런 찌꺼기를 통째로 죽인다. 런처만 죽이면 노드가 고아로 남고,
    # 그게 쌓여 CPU 를 먹으면 깊이 영상이 끊겨 검출이 조용히 0 이 된다.
    bash "$HOME/kill_sim.sh" > /dev/null 2>&1 || true
    echo "[$(date +%H:%M)] focus $cond $i 시작 (관절 $K / 신뢰도 $C)"
    UGV_LYING_KPTS=$K UGV_LYING_CONF=$C \
      UGV_SPAWN_X=2.0 UGV_SPAWN_Y=-7.0 \
      ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART \
      setsid timeout --signal=INT --kill-after=30 "$DUR" \
      ros2 launch ugv_bringup patrol_sim.launch.py \
        world:=rescue_building headless:=true expected_victims:=3 \
        > "$HOME/foc${cond}_${i}.log" 2>&1 || true
    echo "[$(date +%H:%M)] focus $cond $i 종료"
  done
done
echo "focus 완료"
