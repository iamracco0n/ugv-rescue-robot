#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""특정 자리 근처에서 '후보 발견 → 조준' 을 시도한 적이 있는지 본다.

    python3 inspect_near.py <x> <y> <반경> <로그...>

왜 필요한가
-----------
'가까이 갔는데 못 찾았다' 도 두 갈래다.

  후보로 잡긴 했는데 관문/등록에서 떨어졌다 -> 관문을 고치면 산다
  후보로 잡은 적조차 없다                  -> YOLO 가 아예 못 본다.
                                             관문을 고쳐도 소용없다

target_manager 의 '후보 발견 (x,y)' 로그에 좌표가 있어 자리별로 갈린다.
"""
import math
import re
import sys

CAND = re.compile(r'후보 발견 \(([^,]+),([^)]+)\)')
LOG = re.compile(r'\[구조 로그\].*?위치:\(([^,]+),([^)]+)\)')


def main():
    tx, ty, r = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    tried = 0          # 후보로 잡은 런
    tried_miss = 0     # 후보로 잡았는데 등록 못 한 런
    never = 0          # 후보로 잡은 적 없는데 못 찾은 런
    found = 0
    for p in sys.argv[4:]:
        try:
            lines = open(p, encoding='utf-8', errors='replace').readlines()
        except OSError:
            continue
        cand = False
        got = False
        for line in lines:
            m = CAND.search(line)
            if m and math.hypot(float(m.group(1)) - tx,
                                float(m.group(2)) - ty) <= r:
                cand = True
            g = LOG.search(line)
            if g and math.hypot(float(g.group(1)) - tx,
                                float(g.group(2)) - ty) <= 3.0:
                got = True
        if got:
            found += 1
            continue
        if cand:
            tried += 1
            tried_miss += 1
        else:
            never += 1
    total = found + tried_miss + never
    print(f'({tx}, {ty}) 반경 {r}m — 총 {total}런')
    print(f'  발견                    {found}런')
    print(f'  후보로는 잡았으나 등록 실패  {tried_miss}런  <- 관문/등록 문제')
    print(f'  후보로 잡은 적도 없음      {never}런  <- YOLO 가 아예 못 봄')


if __name__ == '__main__':
    main()
