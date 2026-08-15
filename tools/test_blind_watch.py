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
검사도 통과했다 — 멀쩡한 로봇의 기각이 잡히기 때문이다. 로봇별로 봐야
드러났다.

실기에서는 더 위험하다. 로봇 한 대가 눈먼 채 구역을 헛돌아도 아무도
모른다. 그래서 노드가 스스로 알리게 했다.

양방향으로 잠근다
-----------------
  · 기동 중이거나 잠깐 아무도 안 보이는 것은 정상 — 헛경보를 내면 안 된다
  · 프레임은 오는데 오래 아무것도 못 보면 반드시 알려야 한다
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'yolo_pose_node.py')


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        outer = self

        class N:
            @property
            def nanoseconds(self):
                return outer.t * 1e9
        return N()


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, msg):
        self.errors.append(msg)


class Node:
    """_check_blind 만 떼어 붙인 최소 객체."""

    def __init__(self):
        self.blind_warn_s = 60.0
        self._frames_seen = 0
        self._last_detect_t = None
        self._blind_warned = False
        self._clock = FakeClock()
        self._logger = FakeLogger()

    def get_clock(self):
        return self._clock

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
        print(f'{desc:<52}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # ── 기동 중에는 아무 말 안 한다 ──────────────────────────────────
    n = Node()
    n._frames_seen = 5
    n._clock.t = 1000.0
    check(n)
    verify('프레임 5장 — 기동 중이라 조용', len(n._logger.errors), 0)

    # ── 첫 호출은 기준 시각만 잡는다 ────────────────────────────────
    n = Node()
    n._frames_seen = 100
    n._clock.t = 1000.0
    check(n)
    verify('첫 호출 — 기준 시각만 잡고 조용', len(n._logger.errors), 0)
    # 나노초를 거쳐 오므로 부동소수 오차가 남는다. 근사로 본다.
    verify('   기준 시각이 잡혔다', abs(n._last_detect_t - 1000.0) < 1e-6, True)

    # ── 잠깐 못 봐도 조용 ───────────────────────────────────────────
    n._clock.t = 1030.0          # 30초 경과, 한계 60초
    check(n)
    verify('30초 못 봄 — 아직 조용', len(n._logger.errors), 0)

    # ── 오래 못 보면 알린다 ─────────────────────────────────────────
    n._clock.t = 1070.0          # 70초 경과
    check(n)
    verify('70초 못 봄 — 경고', len(n._logger.errors), 1)

    # ── 같은 상태로 계속 떠들지 않는다 ──────────────────────────────
    n._clock.t = 1100.0
    check(n)
    verify('계속 못 봐도 경고는 한 번만', len(n._logger.errors), 1)

    # ── 회복하면 다시 감시한다 ──────────────────────────────────────
    n._last_detect_t = 1100.0    # 뭔가 봤다
    n._clock.t = 1110.0
    check(n)
    verify('회복 후 조용', len(n._logger.errors), 1)
    verify('   경고 플래그가 풀렸다', n._blind_warned, False)

    n._clock.t = 1180.0          # 다시 80초 못 봄
    check(n)
    verify('다시 오래 못 보면 또 경고', len(n._logger.errors), 2)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('9개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
