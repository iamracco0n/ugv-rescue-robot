#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""눈멂 감시 단위 테스트.

    python3 tools/test_blind_watch.py

무엇을 잡으려는 건가
--------------------
2대 런에서 로봇 한 대만 카메라가 죽는 일이 실제로 있었다. 그 로봇은 탐사
목표를 28회 내며 자기 구역을 멀쩡히 돌았고 카메라 토픽도 살아 있었다.
다만 사람 박스가 하나도 안 나와 그 구역 조난자 4명을 통째로 놓쳤다.

다른 로봇이 정상이라 런은 정상처럼 보였다. 로그 전체의 기각 건수를 보는
검사도 통과했다 — 멀쩡한 로봇의 기각이 잡히기 때문이다.

판정 조건을 한 번 갈아엎었다
----------------------------
처음엔 '60초 동안 박스가 없으면 경고' 로 만들었다. 실전에 넣어 보니 전
런에서 9~13건씩 찍혔다 — 7/7 완주한 정상 런도 마찬가지였다. 큰 월드에서
복도나 빈 방을 지날 때 1분 넘게 아무도 안 보이는 것은 당연하다. 임계값을
측정 없이 추측으로 잡은 탓이다.

실제로 잡아야 할 고장은 '런 내내 검출 0' 이었다. 그래서 '한동안 못 봤다'
가 아니라 '한 번도 못 봤다' 로 바꿨다. 한 번이라도 봤으면 카메라는 살아
있는 것이므로 이 조건은 헛경보가 날 수 없다.
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'yolo_pose_node.py')


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, msg):
        self.errors.append(msg)


class Node:
    """_check_blind 만 떼어 붙인 최소 객체."""

    def __init__(self):
        self.blind_frames = 1500
        self._frames_seen = 0
        self._last_detect_t = None
        self._blind_warned = False
        self._logger = FakeLogger()

    def get_logger(self):
        return self._logger


def load():
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_check_blind':
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'math': math}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns['_check_blind']
    raise SystemExit(f'{SRC} 에 _check_blind 가 없다')


def main():
    check = load()
    fails = 0

    def verify(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<54}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # ── 기동 중에는 조용 ────────────────────────────────────────────
    n = Node()
    n._frames_seen = 100
    check(n)
    verify('프레임 100장 — 아직 판단 안 함', len(n._logger.errors), 0)

    # ── 충분히 받았는데 한 번도 못 봤으면 경고 ──────────────────────
    n._frames_seen = 1500
    check(n)
    verify('프레임 1500장 · 검출 0 — 경고', len(n._logger.errors), 1)

    # ── 같은 상태로 계속 떠들지 않는다 ──────────────────────────────
    n._frames_seen = 3000
    check(n)
    verify('계속 못 봐도 경고는 한 번만', len(n._logger.errors), 1)

    # ── 헛경보 방지: 한 번이라도 봤으면 절대 경고 안 한다 ───────────
    # 여기가 핵심이다. 예전 판정('한동안 못 봤다')은 정상 런에도 런당
    # 9~13건씩 찍혔다 — 복도를 지나는 동안 아무도 안 보이는 것은 정상이다.
    n2 = Node()
    n2._frames_seen = 100000        # 아주 오래 돌았고
    n2._last_detect_t = 1.0         # 딱 한 번 봤다
    check(n2)
    verify('한 번이라도 봤으면 조용 (헛경보 방지)', len(n2._logger.errors), 0)

    # ── 경계값 ──────────────────────────────────────────────────────
    n3 = Node()
    n3._frames_seen = 1499
    check(n3)
    verify('한 장 모자라면 아직 조용', len(n3._logger.errors), 0)
    n3._frames_seen = 1500
    check(n3)
    verify('   딱 채우면 경고', len(n3._logger.errors), 1)

    # ── 임계값을 바꿀 수 있다 ───────────────────────────────────────
    n4 = Node()
    n4.blind_frames = 200
    n4._frames_seen = 250
    check(n4)
    verify('임계값을 낮추면 더 일찍 경고', len(n4._logger.errors), 1)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('7개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
