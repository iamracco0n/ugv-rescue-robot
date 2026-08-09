#!/usr/bin/env bash
# 수색 시뮬을 헤드리스로 정해진 시간만큼 돌리고 자동 채점한다.
#
#   tools/run_eval.sh [월드] [실종자수] [초] [로그경로]
#
# 예)
#   tools/run_eval.sh rescue_building_large 7 2400
#   tools/run_eval.sh rescue_building       3  900
#
# 종료 코드 0=합격, 1=불합격 — CI 에서 그대로 쓸 수 있다.
#
# set -u 는 쓰지 않는다. ROS 의 setup.bash 가 미설정 변수를 참조해
# (AMENT_TRACE_SETUP_FILES) 소싱 단계에서 바로 죽는다.

WORLD=${1:-rescue_building_large}
VICTIMS=${2:-7}
DURATION=${3:-2400}
LOG=${4:-/tmp/ugv_eval_${WORLD}.log}

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TRUTH=$(mktemp /tmp/ugv_truth_XXXX.json)

cleanup() {
  [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2>/dev/null
  sleep 3
  # 패턴 kill 은 자기 셸까지 잡으므로 PID 로만 정리한다
  for p in $(ps -eo pid,args --no-headers \
             | grep -E 'gz sim|ugv_vision|nav2_|slam_toolbox|ros_gz_bridge|robot_state_publisher|lifecycle_manager' \
             | grep -v grep | awk '{print $1}'); do
    [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null
  done
}
trap cleanup EXIT

# ── 정답 만들기 ───────────────────────────────────────────────────────
GEN="$WS/src/ugv_bringup/worlds/gen_${WORLD#rescue_building_}.py"
if [ "$WORLD" = rescue_building_large ]; then
  GEN="$WS/src/ugv_bringup/worlds/gen_rescue_large.py"
fi
if [ -f "$GEN" ]; then
  python3 "$GEN" --truth "$TRUTH" >/dev/null || { echo "정답 생성 실패"; exit 2; }
else
  # 생성기가 없는 월드(손으로 쓴 SDF)는 truth/<월드>.json 을 쓴다
  if [ -f "$WS/tools/truth/${WORLD}.json" ]; then
    cp "$WS/tools/truth/${WORLD}.json" "$TRUTH"
  else
    echo "정답 파일 없음: $WS/tools/truth/${WORLD}.json"; exit 2
  fi
fi

# ── 실행 ──────────────────────────────────────────────────────────────
# 한 머신에서 여러 개를 동시에 돌릴 때는 반드시 분리해야 한다.
#   ROS_DOMAIN_ID  — ROS 토픽 격리
#   GZ_PARTITION   — Gazebo transport 격리 (스폰 서비스가 여기 붙는다)
# 이걸 안 하면 로봇 스폰 요청이 다른 월드로 가서 타임아웃난다
#   [ros_gz_sim] Request to create entity ... timed out
# START_DELAY 로 기동 시차를 주면 스폰 경합도 줄어든다.
if [ -n "${START_DELAY:-}" ]; then
  echo "기동 지연 ${START_DELAY}초 대기(동시 실행 시 스폰 경합 방지)"
  sleep "$START_DELAY"
fi
echo "월드=$WORLD 실종자=$VICTIMS 시간=${DURATION}초  로그=$LOG"
echo "  예산=${BUDGET:-180.0}초 반경=${RADIUS:-5.0}m 도메인=${ROS_DOMAIN_ID:-0} 파티션=${GZ_PARTITION:-기본}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

# 탐사 성향은 환경변수로 덮어쓸 수 있다(파라미터 스윕용)
#   BUDGET=90 RADIUS=4 tools/run_eval.sh ...
ros2 launch ugv_bringup patrol_sim.launch.py \
     world:="$WORLD" expected_victims:="$VICTIMS" headless:=true \
     room_clear_budget_s:="${BUDGET:-180.0}" \
     sweep_first_radius:="${RADIUS:-5.0}" \
     > "$LOG" 2>&1 &
LAUNCH_PID=$!

# 조기 종료 조건: 전원 발견 + 회차 완료가 모두 나오면 더 볼 필요가 없다
elapsed=0
while [ "$elapsed" -lt "$DURATION" ]; do
  sleep 10
  elapsed=$((elapsed + 10))
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "런치가 조기 종료됨(${elapsed}초)"; break
  fi
  if grep -q '회차 완료' "$LOG" 2>/dev/null; then
    echo "회차 완료 감지 — 조기 종료(${elapsed}초)"; break
  fi
  # 임무의 1차 목표는 전원 발견이다. 그 뒤 보충 수색까지 기다리면
  # 검증 한 번에 한 시간씩 드니, 여기서 끊고 '전원 발견까지 걸린 시간'
  # 을 성능 지표로 쓴다.
  if [ "${STOP_ON_ALL_FOUND:-1}" = 1 ] && grep -q '전원 발견' "$LOG" 2>/dev/null; then
    echo "전원 발견 감지 — 조기 종료(${elapsed}초)"; break
  fi
done

cleanup
sleep 2

# ── 채점 ──────────────────────────────────────────────────────────────
echo
python3 "$WS/tools/score_run.py" "$LOG" --truth "$TRUTH"
rc=$?
rm -f "$TRUTH"
exit $rc
