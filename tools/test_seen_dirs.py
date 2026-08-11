#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""관측 방향 비트 단위 테스트.

    python3 tools/test_seen_dirs.py

왜 방향을 기록하나
------------------
지금은 한 번 본 칸을 '봤음' 으로만 표시한다. 그래서 한 방향에서 스쳐 본
구석도 다시 안 간다. 가려진 조난자(다른 물체 뒤, 특정 각도에서만 보이는)를
못 찾는 원인으로 보인다.

실측: 큰 월드에서 2대도 6런 중 3런만 7/7 을 냈다. 미달성 런도 대부분
6명까지는 찾았다 — 편차가 통째로 '7번째를 언제 찾느냐' 에 몰려 있다.

방향을 4구획으로 나눠 비트로 쌓는다. 두 방향 이상에서 본 칸만 '제대로
봤다' 로 친다. numpy 불리언이 이미 1바이트라 uint8 비트마스크로 바꿔도
메모리가 안 늘어난다.
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load(name):
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'math': math}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


def main():
    dir_bit = load('dir_bit')
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<46}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # 4구획: 동(0), 북(1), 서(2), 남(3) — 각 90도
    check('동쪽(0도)은 비트 1', dir_bit(0.0), 1)
    check('북쪽(90도)은 비트 2', dir_bit(math.pi / 2), 2)
    check('서쪽(180도)은 비트 4', dir_bit(math.pi), 4)
    check('남쪽(-90도)은 비트 8', dir_bit(-math.pi / 2), 8)

    # 경계 근처가 같은 구획으로 묶이는지 — 44도는 아직 동쪽
    check('44도는 동쪽 구획', dir_bit(math.radians(44)), 1)
    check('46도는 북쪽 구획', dir_bit(math.radians(46)), 2)

    # 한 바퀴 돌아도 같은 값 (wrap)
    check('360도는 0도와 같다', dir_bit(2 * math.pi), dir_bit(0.0))
    check('-180도는 180도와 같다',
          dir_bit(-math.pi), dir_bit(math.pi))

    # 반대 방향은 다른 비트여야 한다 — 이게 핵심이다.
    # 같은 칸을 앞뒤로 지나가며 봤다면 가림이 풀렸을 가능성이 높다.
    check('동과 서는 다른 비트', dir_bit(0.0) != dir_bit(math.pi), True)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('9개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
