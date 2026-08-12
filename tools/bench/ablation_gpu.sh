#!/usr/bin/env bash
# 개선 항목을 하나씩 켜 가며 잰다(ablation).
#
# 한꺼번에 켜면 뭐가 효과였는지 알 수 없다. 오늘 이미 여러 번 겪었다 —
# lambda=1.5, 방기준 탐사처럼 '꼼꼼하게' 만든 것이 오히려 나빴던 사례가
# 많다. 특히 방향 기록(seen_min_dirs=2)은 수색을 길게 만드는 방향이라
# 단독으로 재야 한다.
#
#   base : 전부 끔 (예전 동작)
#   dirs : 관측 방향 기록만
#   ghost: 반복 유령 자리 기억만
#   all  : 전부 켬
#
# 지표는 '1200초 시점 발견 인원' 이다. '전원 발견 시각' 은 최댓값 통계라
# 편차가 크고, 미달성 런이 표본에서 빠져 편향이 생긴다.
#
# 순서를 라운드마다 한 칸씩 돌린다 (ablation.sh 와 다른 점)
# ----------------------------------------------------------
# 고정 순서면 두 가지가 조건에 섞여 든다.
#   1. 밤새 돌리다 아침에 끊으면 잘리는 건 늘 마지막 조건이다 — all 만
#      표본이 적어진다.
#   2. 머신이 밤새 더워지면 base 는 늘 찬 상태, all 은 늘 더운 상태에서
#      돈다. 조건 차이인지 온도 차이인지 구분이 안 된다.
# 돌려 주면 둘 다 조건에 골고루 퍼져 상쇄된다. 1대/2대를 짝으로 비교해
# 머신 성능차를 지운 것과 같은 이유다.
set -e
WS=${WS:-$HOME/ugv_ws}
NAME=${NAME:-abl}
DOM=${DOM:-200}
PART=${PART:-abl}
DUR=${DUR:-1500}
ROUNDS=${ROUNDS:-3}
EXTRA=${EXTRA:-}          # 느린 머신용: "UGV_SPAWN_DELAY=30"
# 조건 순서를 몇 칸 더 돌려서 시작할지. 꼬리에 1라운드만 덧붙일 때 쓴다 —
# 그 라운드는 아침에 잘릴 가능성이 높은데, 늘 base 부터 시작하면 잘리는
# 쪽만 계속 손해를 본다. 머신마다 다른 값을 주면 골고루 퍼진다.
OFFSET=${OFFSET:-0}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

CONDS=(base dirs ghost all)

# 매 런 전에 GPU 가 응답하는지 본다.
#
# 오로라에서 Xid 16(디스플레이 엔진 행)이 나면 nvidia-smi 가 멈추고,
# 가제보가 렌더를 못 해 odom 이 안 나온다. 로봇은 가만히 서 있는데
# 런은 정상 종료되므로 '조난자 0명' 이 멀쩡한 결과처럼 남는다.
# 실제로 오늘 그 로그를 진짜 결과로 착각해 1대/2대 비교에 넣을 뻔했다.
#
# 걸린 GPU 는 재부팅 전엔 안 돌아온다. 그러니 건너뛰지 말고 통째로
# 멈춘다 — 몇 시간 더 돌려 봐야 쓰레기 로그만 쌓인다.
gpu_ok() {
  timeout 15 nvidia-smi -L >/dev/null 2>&1
}

for round in $(seq 1 "$ROUNDS"); do
  start=$(( (round - 1 + OFFSET) % 4 ))
  for i in 0 1 2 3; do
    cond=${CONDS[$(( (start + i) % 4 ))]}
    case $cond in
      base)  D=1; G=0 ;;
      dirs)  D=2; G=0 ;;
      ghost) D=1; G=3 ;;
      all)   D=2; G=3 ;;
    esac
    for p in $(pgrep -f "multi_robot_si[m]" 2>/dev/null); do
      pg=$(ps -o pgid= -p "$p" 2>/dev/null | tr -d ' ')
      [ -n "$pg" ] && kill -- -"$pg" 2>/dev/null || true
    done
    sleep 10
    if ! gpu_ok; then
      echo "[$(date +%H:%M)] GPU 무응답(Xid 행 의심) — $NAME 중단. 재부팅 필요"
      exit 2
    fi
    echo "[$(date +%H:%M)] $NAME round$round $cond (dirs=$D ghost=$G) 시작"
    env $EXTRA ROS_DOMAIN_ID=$DOM GZ_PARTITION=$PART UGV_N_ROBOTS=2 \
      UGV_BOUNDS=-27,-19,27,19 UGV_GHOST_NEED=$G UGV_SEEN_DIRS=$D \
      setsid timeout --signal=INT --kill-after=30 "$DUR" \
      ros2 launch ugv_bringup multi_robot_sim.launch.py \
        world:=rescue_building_large headless:=true expected_victims:=7 \
        > "$HOME/${NAME}_${round}_${cond}.log" 2>&1 || true
    echo "[$(date +%H:%M)] $NAME round$round $cond 종료"
  done
done
echo "$NAME ablation 완료"
