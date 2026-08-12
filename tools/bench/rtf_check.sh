#!/usr/bin/env bash
# 로그의 첫/마지막 타임스탬프로 시뮬이 실제로 몇 초를 진행했는지 본다.
#
# 노드가 use_sim_time 을 쓰므로 로그 타임스탬프는 시뮬 시각이다.
# 벽시계 900초를 돌렸는데 시뮬이 450초만 갔다면 RTF 0.5 — 로봇 입장에서는
# 절반의 시간만 준 셈이라 늦게 찾는 대상부터 잘려 나간다.
for f in "$@"; do
  first=$(grep -aoE '\[1[0-9]{9}\.[0-9]+\]' "$f" 2>/dev/null | head -1 | tr -d '[]')
  last=$(grep -aoE '\[1[0-9]{9}\.[0-9]+\]' "$f" 2>/dev/null | tail -1 | tr -d '[]')
  if [ -z "$first" ] || [ -z "$last" ]; then
    printf '%-16s %s\n' "$(basename "$f" .log)" "타임스탬프 없음"
    continue
  fi
  printf '%-16s 시뮬 %6.0fs\n' "$(basename "$f" .log)" \
    "$(echo "$last - $first" | bc)"
done
