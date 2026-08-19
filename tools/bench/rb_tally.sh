#!/usr/bin/env bash
# 방 우선 보너스 스윕 집계.
#
# 두 지표를 같이 본다.
#   재진입  — 덜 보고 나왔던 방으로 되돌아온 횟수. 줄어야 성공
#   발견    — 1200초 시점 발견 인원. 안 떨어져야 함
# 보너스를 키우면 재진입은 줄지만, 어느 지점부터 방에서 못 나와 발견이
# 떨어진다. 그 사이가 답이다.
#
# 진행 중인 로그(최근 3분 내 수정)는 뺀다.
cd "$HOME" || exit 1
for b in 0 3 6 12; do
  files=$(find "$HOME" "$HOME/aurora_rb" "$HOME/omen_rb" -maxdepth 1 \
          -name "rb*_b${b}.log" -mmin +3 2>/dev/null | sort)
  n=$(echo "$files" | grep -c . )
  [ -z "$files" ] && { printf '보너스 %-3s  런 없음\n' "$b"; continue; }
  re=$(python3 "$HOME/roomleave_tally.py" $files 2>/dev/null \
       | awk '/런당 평균/ {print $NF}')
  lv=$(python3 "$HOME/roomleave_tally.py" $files 2>/dev/null \
       | awk '/런당 평균/ {print $4}')
  vi=$(python3 "$HOME/victims_at.py" 1200 $files 2>/dev/null \
       | awk '/중앙값/ {print $2}')
  av=$(python3 "$HOME/victims_at.py" 1200 $files 2>/dev/null \
       | sed -n 's/.*평균 \([0-9.]*\)).*/\1/p')
  printf '보너스 %-3s  n=%-3s 재진입 %-6s 후반이탈 %-6s 발견 중앙 %-3s 평균 %s\n' \
    "$b" "$n" "$re" "$lv" "$vi" "$av"
done
