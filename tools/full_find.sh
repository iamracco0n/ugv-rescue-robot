#!/usr/bin/env bash
# 런마다 '전원 발견(7/7)' 에 도달했는지, 도달했다면 언제인지 본다.
#
# 1200초 시점 발견 인원은 속도 지표라 완주 여부를 안 알려준다.
# 대표 결과가 '조난자 7/7 전원 발견' 이므로 그것도 따로 봐야 한다.
for f in "$@"; do
  name=$(basename "$f" .log)
  line=$(grep -a "전원 발견" "$f" 2>/dev/null | head -1)
  if [ -z "$line" ]; then
    # 못 찾았으면 마지막으로 센 인원을 보여 준다
    last=$(grep -ao "조난자 [0-9]*/[0-9]*명" "$f" 2>/dev/null | tail -1)
    printf '%-14s 미달성  (마지막 %s)\n' "$name" "${last:-기록없음}"
    continue
  fi
  t0=$(grep -aoE '\[1[0-9]{9}\.' "$f" | head -1 | tr -d '[.')
  t1=$(echo "$line" | grep -oE '\[1[0-9]{9}\.' | head -1 | tr -d '[.')
  if [ -n "$t0" ] && [ -n "$t1" ]; then
    printf '%-14s 전원발견  %ss\n' "$name" "$((t1 - t0))"
  else
    printf '%-14s 전원발견\n' "$name"
  fi
done
