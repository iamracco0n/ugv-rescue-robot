#!/usr/bin/env bash
# 런별 조사 시작 횟수와 목표 발행 수.
#
# det_conf 를 낮추면 후보가 늘어난다. 늘어난 후보가 전부 진짜면 이득이지만,
# 벽·가구면 조사(정지->조준->포기)에 건당 10~15초를 버린다. 등록이 안 되면
# 유령 집계에는 안 잡히는데 시간은 이미 쓴 것이므로 따로 봐야 한다.
for f in "$@"; do
  n=$(basename "$f" .log)
  printf '%-12s 조사 %-5s 목표 %-5s 구조 %s\n' "$n" \
    "$(grep -ac '정지 요청 후 조준' "$f" 2>/dev/null)" \
    "$(grep -ac '탐사 목표' "$f" 2>/dev/null)" \
    "$(grep -ac '구조 로그' "$f" 2>/dev/null)"
done
