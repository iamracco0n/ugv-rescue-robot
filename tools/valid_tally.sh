#!/usr/bin/env bash
# 로봇별 검출 생존을 확인해 무효 런을 거른 뒤 완주율을 센다.
#
# 왜 로봇별로 봐야 하나
# ---------------------
# 2대 런에서 한 대만 눈이 멀면 다른 대가 돌아다녀 런이 정상처럼 보인다.
# 로그 전체의 기각 건수를 보는 검사는 그걸 통과시킨다. 그런데 눈먼 로봇의
# 구역 조난자는 통째로 못 찾으므로 완주는 실패한다 — 알고리즘 탓으로
# 오해하기 딱 좋다.
#
#   $@ = 로그들
LIMIT=${LIMIT:-1800}
valid=(); invalid=()
for f in "$@"; do
  bad=0
  for r in ugv1 ugv2; do
    kpt=$(grep -a "$r\.yolo_pose_node" "$f" 2>/dev/null \
          | grep -ao '키포인트=[0-9]*' | cut -d= -f2 | awk '{s+=$1} END {print s+0}')
    g=$(grep -ac "$r\.patrol_navigator.*탐사 목표" "$f" 2>/dev/null)
    # 목표를 냈는데(=움직였는데) 기각이 0 이면 그 로봇은 눈이 멀었다
    [ "$g" -gt 5 ] && [ "$kpt" -eq 0 ] && bad=1
  done
  if [ "$bad" -eq 1 ]; then invalid+=("$(basename "$f" .log)")
  else valid+=("$f"); fi
done

echo "무효 ${#invalid[@]}런: ${invalid[*]}"
echo "유효 ${#valid[@]}런"
[ ${#valid[@]} -eq 0 ] && exit 0

times=$(bash "$HOME/full_find.sh" "${valid[@]}" 2>/dev/null \
        | grep "전원발견" | awk '{print $3}' | tr -d 's' | sort -n)
done_=$(echo "$times" | grep -c .)
within=$(echo "$times" | awk -v L="$LIMIT" '$1 <= L' | grep -c .)
echo "  7/7 완주        $done_/${#valid[@]}"
echo "  ${LIMIT}초 안에    $within/${#valid[@]}"
echo "  완주 시각: $(echo "$times" | tr '\n' ' ')"
