#!/usr/bin/env bash
# 단위 테스트 전부 실행. 실패가 하나라도 있으면 1 을 반환한다.
#
#     bash tools/run_tests.sh
#
# 이 테스트들은 ROS 없이 도는 순수 함수 검사다. 노드를 통째로 띄우지 않고
# 계산 규칙만 떼어 확인하므로 몇 초면 끝난다 — 시뮬을 25분 돌려 확인하던
# 것들을 여기로 옮겨 왔다.
cd "$(dirname "$0")/.." || exit 1
# 판정은 종료 코드로 한다. 출력 마지막 줄로 가르려 했더니, 통과 요약 뒤에
# 설명 문장을 더 붙이는 테스트들이 실패로 잡혔다.
fails=0
for t in tools/test_*.py; do
  out=$(python3 "$t" 2>&1)
  rc=$?
  summary=$(echo "$out" | grep -E "사례 전부 통과|실패 [0-9]+건" | tail -1)
  [ -z "$summary" ] && summary="(요약 없음)"
  if [ "$rc" -eq 0 ]; then
    printf '%-34s%s\n' "$(basename "$t")" "$summary"
  else
    printf '%-34s%s\n' "$(basename "$t")" "실패 — $summary"
    fails=$((fails + 1))
  fi
done
echo
if [ "$fails" -gt 0 ]; then
  echo "테스트 파일 ${fails}개에서 실패"
  exit 1
fi
echo "전부 통과"
