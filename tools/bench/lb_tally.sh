#!/usr/bin/env bash
# 누운 사람 관문 켬/끔 집계 — 누운 3명만 따로 센다.
#   $1 = 로그 디렉터리 (기본: 홈)
#
# 전체 7명으로 보면 서있는 3명이 거의 항상 잡혀 차이가 희석된다.
# 관문이 바꾸는 것은 누운 사람뿐이므로 그 셋만 본다.
DIR=${1:-$HOME}
for c in on off; do
  files=$(find "$DIR" -maxdepth 1 -name "lb${c}_[0-9].log" -mmin +3 2>/dev/null | sort)
  n=$(echo "$files" | grep -c .)
  [ -z "$files" ] && { printf '%-4s 런 없음\n' "$c"; continue; }
  out=$(python3 "$HOME/which_missed.py" $files 2>/dev/null)
  # 설명 칸에 공백이 들어 있어 $3 으로 자르면 설명을 숫자로 읽는다.
  # 'n/runs' 는 끝에서 두 번째 칸이다.
  lying=$(echo "$out" | grep -E "lying_n3|lying_s2|lying_s4" \
          | awk '{printf "%s=%s ", $1, $(NF-1)}')
  tot=$(echo "$out" | grep -E "lying_n3|lying_s2|lying_s4" \
        | awk '{split($(NF-1), a, "/"); s += a[1]} END {print s+0}')
  printf '%-4s n=%-2s  %s  합계 %s/%s\n' \
    "$c" "$n" "$lying" "$tot" "$((n * 3))"
done
