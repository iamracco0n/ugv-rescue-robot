#!/usr/bin/env bash
# YOLO 재학습용 데이터를 시뮬에서 자동 수집한다.
#
#   tools/collect_dataset.sh [월드] [실종자수] [초] [출력디렉토리]
#
# 예) tools/collect_dataset.sh ghost_bench 3 1800 ~/ugv_dataset
#
# 정답 위치를 알고 있으므로 사람이 라벨을 붙일 필요가 없다.
#   검출이 정답 근처   → 양성(사람 박스)
#   그렇지 않으면      → 유령. 빈 라벨 = 배경(하드 네거티브)

WORLD=${1:-ghost_bench}
VICTIMS=${2:-3}
DURATION=${3:-1800}
OUT=${4:-$HOME/ugv_dataset}

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TRUTH=$(mktemp /tmp/ugv_truth_XXXX.json)
LOG=/tmp/collect_${WORLD}.log

cleanup() {
  [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2>/dev/null
  [ -n "${CAP_PID:-}" ] && kill "$CAP_PID" 2>/dev/null
  sleep 3
  for p in $(ps -eo pid,args --no-headers \
             | grep -E 'gz sim|ugv_vision|nav2_|slam_toolbox|ros_gz_bridge|robot_state_publisher|lifecycle_manager' \
             | grep -v grep | awk '{print $1}'); do
    [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null
  done
}
trap cleanup EXIT

case "$WORLD" in
  rescue_building_large) GEN="$WS/src/ugv_bringup/worlds/gen_rescue_large.py" ;;
  ghost_bench)           GEN="$WS/src/ugv_bringup/worlds/gen_ghost_bench.py" ;;
  *) echo "정답 생성기 없는 월드: $WORLD"; exit 2 ;;
esac
python3 "$GEN" --truth "$TRUTH" >/dev/null || { echo "정답 생성 실패"; exit 2; }

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$WS/install/ugv_description/share:${GZ_SIM_RESOURCE_PATH:-}"

echo "월드=$WORLD  시간=${DURATION}초  출력=$OUT"
ros2 launch ugv_bringup patrol_sim.launch.py \
     world:="$WORLD" expected_victims:="$VICTIMS" headless:=true \
     > "$LOG" 2>&1 &
LAUNCH_PID=$!

# 비전 노드가 뜬 뒤에 수집을 붙인다(런치 58초 스케줄)
sleep 70
ros2 run ugv_vision dataset_capture_node --ros-args \
     -p truth_json:="$TRUTH" -p out_dir:="$OUT" \
     >> "$LOG" 2>&1 &
CAP_PID=$!
echo "수집 노드 시작 (PID $CAP_PID)"

elapsed=70
while [ "$elapsed" -lt "$DURATION" ]; do
  sleep 30; elapsed=$((elapsed + 30))
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "런치 종료(${elapsed}초)"; break; }
done

cleanup
sleep 2
LATEST=$(ls -dt "$OUT"/*/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
  n_img=$(ls "$LATEST/images" 2>/dev/null | wc -l)
  n_pos=$(grep -l . "$LATEST"/labels/*.txt 2>/dev/null | wc -l)
  echo "수집 완료: $LATEST"
  echo "  이미지 ${n_img}장 · 양성 ${n_pos}장 · 유령(배경) $((n_img - n_pos))장"
fi
rm -f "$TRUTH"
