#!/usr/bin/env bash
# 앞 작업이 끝나면 이어서 ablation 을 더 돌린다.
#
# 패턴(pgrep -f)이 아니라 PID 로 기다린다. 패턴은 자기 자신을 잡거나
# 반대로 못 잡는 사고가 나기 쉽고, 그러면 시뮬 두 벌이 겹쳐 돈다.
# 오늘 가제보 두 벌이 겹쳐 돌아 결과가 오염된 적이 있다.
#
#   $1 = 기다릴 PID
#   $2 = 실행할 스크립트 경로
#   나머지 = 그 스크립트에 넘길 환경변수
WAIT_PID="$1"; shift
ABL="$1"; shift
while kill -0 "$WAIT_PID" 2>/dev/null; do sleep 60; done
sleep 30                      # 가제보/노드가 완전히 내려갈 틈을 준다
exec env "$@" bash "$ABL"
