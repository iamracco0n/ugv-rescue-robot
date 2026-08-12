#!/usr/bin/env bash
# 방 우선 보너스 집계 — 한 머신 것만.
#   $1 = 로그가 있는 디렉터리 (기본: 홈 = 메인)
#
# 머신을 섞으면 절대값이 머신마다 달라 흐려진다. 한 머신 안에서 보너스
# 값끼리만 비교하는 것이 가장 깨끗하다.
DIR=${1:-$HOME}
for b in 0 3 6 12; do
  files=$(find "$DIR" -maxdepth 1 -name "rb*_b${b}.log" -mmin +3 2>/dev/null | sort)
  n=$(echo "$files" | grep -c .)
  [ -z "$files" ] && { printf '보너스 %-3s  런 없음\n' "$b"; continue; }
  re=$(python3 "$HOME/roomleave_tally.py" $files 2>/dev/null \
       | awk '/런당 평균/ {print $NF}')
  lv=$(python3 "$HOME/roomleave_tally.py" $files 2>/dev/null \
       | awk '/런당 평균/ {print $4}')
  vals=$(python3 "$HOME/victims_at.py" 1200 $files 2>/dev/null \
       | awk 'NF==2 && $2 ~ /^[0-9]+$/ {printf "%s ", $2}')
  av=$(python3 "$HOME/victims_at.py" 1200 $files 2>/dev/null \
       | sed -n 's/.*평균 \([0-9.]*\)).*/\1/p')
  printf '보너스 %-3s n=%-2s 재진입 %-6s 후반이탈 %-6s 발견 %-4s [%s]\n' \
    "$b" "$n" "$re" "$lv" "$av" "$vals"
done
