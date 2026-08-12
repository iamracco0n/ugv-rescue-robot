#!/usr/bin/env bash
# 보너스 값별 '7/7 전원 발견' 달성률.
#
# 1200초 시점 발견 인원은 속도 지표다. 그런데 이 프로젝트의 대표 결과는
# '조난자 7/7 전원 발견' 이고, 2대 구성이 그걸 얼마나 자주 해내는지가
# 진짜 완성도다. 실측으로 마지막 1~2명이 잘 안 잡히는데, 그게 바로
# '방에 빈 공간 남기고 나가서 벽 뒤 사람을 놓친다' 는 문제와 같은 자리다.
#
# 고정 시간 안의 달성 여부(0/1)라 미달성 런도 표본에 남는다 —
# '전원 발견 시각' 만 보면 완주한 런만 비교하게 되어 편향이 생긴다.
for b in 0 3 6 12; do
  files=$(find "$HOME" "$HOME/aurora_rb" "$HOME/omen_rb" -maxdepth 1 \
          -name "rb*_b${b}.log" -mmin +3 2>/dev/null | sort)
  [ -z "$files" ] && { printf '보너스 %-3s 런 없음\n' "$b"; continue; }
  n=0; ok=0; times=""
  for f in $files; do
    n=$((n + 1))
    line=$(grep -a "전원 발견" "$f" 2>/dev/null | head -1)
    [ -z "$line" ] && continue
    ok=$((ok + 1))
    t0=$(grep -aoE '\[1[0-9]{9}\.' "$f" | head -1 | tr -d '[.')
    t1=$(echo "$line" | grep -oE '\[1[0-9]{9}\.' | head -1 | tr -d '[.')
    [ -n "$t0" ] && [ -n "$t1" ] && times="$times $((t1 - t0))s"
  done
  printf '보너스 %-3s  7/7 달성 %s/%-3s [%s]\n' "$b" "$ok" "$n" "$times"
done
