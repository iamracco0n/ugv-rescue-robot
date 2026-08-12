#!/usr/bin/env bash
# 키포인트 관문 기각 건수를 센다.
#
# 왜 이걸 보나
# ------------
# '누운 사람을 찾았나' 는 로봇이 그 방에 갔는지에 크게 좌우된다. 동선 운이
# 결과를 지배해서, 관문 수정의 효과가 그 밑에 묻힌다. 실제로 두 머신이
# 정반대 결과를 냈다(오로라 fix 0/2 base 2/2, OMEN fix 2/2 base 1/2).
#
# 관문 수정이 실제로 바꾸는 것은 '사람이 화면에 들어왔을 때 통과시키느냐'
# 하나뿐이다. 그건 기각 건수로 직접 볼 수 있고, 런당 수십~수백 건이라
# 이분법 지표보다 훨씬 예민하다.
#
# 로그 형식:
#   [오탐 게이트] 기각 15s: 키포인트=86 박스크기=0 depth불일치=0
for f in "$@"; do
  name=$(basename "$f" .log)
  kpt=$(grep -ao '키포인트=[0-9]*' "$f" 2>/dev/null \
        | cut -d= -f2 | awk '{s+=$1} END {print s+0}')
  geom=$(grep -ao '박스크기=[0-9]*' "$f" 2>/dev/null \
        | cut -d= -f2 | awk '{s+=$1} END {print s+0}')
  dep=$(grep -ao 'depth불일치=[0-9]*' "$f" 2>/dev/null \
        | cut -d= -f2 | awk '{s+=$1} END {print s+0}')
  printf '%-16s 키포인트기각 %-7s 박스 %-5s depth %-5s\n' \
    "$name" "$kpt" "$geom" "$dep"
done
