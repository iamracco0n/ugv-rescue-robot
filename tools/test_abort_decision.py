#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav2 목표 포기 판정 단위 테스트.

    python3 tools/test_abort_decision.py

왜 필요한가
-----------
순찰기는 /goal_pose **토픽**으로 목표를 던진다. 액션 클라이언트가 아니라서
Nav2 의 결과를 못 받았다. 그래서 Nav2 가 복구행동(backup·spin·wait)을 다
쓰고 손을 든 뒤에도, 순찰기는 자기 제한시간(거리 비례, 최대 150초)이 다
흐를 때까지 그 목표를 붙들고 서 있었다.

박힘 감지도 이 구간을 못 잡는다. Nav2 가 속도를 안 내고 있으므로
stuck_decision 이 '일부러 선 것' 으로 보고 넘긴다 — 복구행동 중에는 옳은
판단이지만, 포기한 뒤의 정지도 같이 가려진다.

실측 (XL·XXL 3대 36런 / 2대 23런):

    구성   Nav2 포기(런당)  회당 대기   런당 누적
    2대        2.7회          60초       2.7분
    3대        5.9회         101초       9.9분

45분 런에서 3대가 서 있기만 한 시간이 9.9분이다.

무엇을 가려야 하나
------------------
포기가 **목표 탓**이면 다른 목표를 고르면 된다. **로봇 탓**이면(팽창영역
안에 들어가 어디로도 경로가 안 나온다) 다른 목표를 골라도 똑같이 실패하고,
재선정만 반복하면 프론티어 목록을 몇 초 만에 태운다. 그래서 연속 포기를
세어 갈랐다. 시뮬을 돌리지 않고 확인할 수 있는 규칙이다.
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')

GRACE = 3.0    # 발행 직후 이 시간 안의 통보는 직전 목표 것
MAX   = 3      # 연속 이 횟수면 로봇 탓으로 본다


def load(name):
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'math': math}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


fails = 0


def check(label, got, want):
    global fails
    ok = got == want
    if not ok:
        fails += 1
    print(f'  {"OK " if ok else "실패"}  {label}   (기대 {want}, 실제 {got})')


def main():
    abort_decision = load('abort_decision')

    # ── 1. 보낸 목표가 없으면 볼 것도 없다 ──────────────────────────
    check('진행 중인 목표 없음 → 무시',
          abort_decision(None, 100.0, GRACE, 0, MAX), 'ignore')

    # ── 2. 발행 직후 통보는 직전 목표의 뒤늦은 결과다 ───────────────
    # 우리가 목표를 갈아끼우면 이전 목표는 CANCELED 로 끝나지만, 포기가
    # 거의 동시에 났으면 통보가 새 목표 발행 뒤에 도착할 수 있다. 그걸
    # 새 목표의 실패로 읽으면 멀쩡한 목표를 즉시 버린다.
    check('발행 0.5초 뒤 통보 → 무시',
          abort_decision(100.0, 100.5, GRACE, 0, MAX), 'ignore')
    check('발행 2.9초 뒤 통보 → 무시(경계 안)',
          abort_decision(100.0, 102.9, GRACE, 0, MAX), 'ignore')

    # Nav2 는 복구행동을 다 쓰고 나서야 포기하므로 실제 포기는 수십 초
    # 뒤에 온다. 3초 여유로 충분히 갈린다.
    check('발행 3.0초 뒤 통보 → 받는다(경계)',
          abort_decision(100.0, 103.0, GRACE, 0, MAX), 'retarget')
    check('발행 45초 뒤 통보 → 받는다(전형적인 경우)',
          abort_decision(100.0, 145.0, GRACE, 0, MAX), 'retarget')

    # ── 3. 한두 번은 목표 탓으로 보고 다른 데로 간다 ────────────────
    check('첫 포기 → 재선정',
          abort_decision(100.0, 145.0, GRACE, 0, MAX), 'retarget')
    check('두 번째 연속 포기 → 재선정',
          abort_decision(100.0, 145.0, GRACE, 1, MAX), 'retarget')

    # ── 4. 연속 3회면 로봇 탓 ───────────────────────────────────────
    # streak 는 '이 통보 이전까지의 연속 횟수' 다. 2 에서 한 번 더 나면
    # 3회 연속이므로 탈출로 넘어간다.
    check('세 번째 연속 포기 → 탈출',
          abort_decision(100.0, 145.0, GRACE, 2, MAX), 'escape')
    check('그 뒤로도 계속 탈출',
          abort_decision(100.0, 145.0, GRACE, 5, MAX), 'escape')

    # ── 5. 연속 판정보다 여유 판정이 먼저다 ─────────────────────────
    # 연속 횟수가 차 있어도, 통보 자체가 직전 목표 것이면 세면 안 된다.
    # 이 순서가 뒤집히면 새 목표를 던지자마자 탈출이 돌아 로봇이 뒤로
    # 밀려난다.
    check('streak 가 차 있어도 발행 직후면 무시',
          abort_decision(100.0, 100.5, GRACE, 5, MAX), 'ignore')

    # ── 6. max_streak=1 이면 첫 포기부터 탈출 ───────────────────────
    # 설정으로 '포기하면 무조건 빠져나온다' 를 만들 수 있어야 한다.
    check('max_streak=1 → 첫 포기부터 탈출',
          abort_decision(100.0, 145.0, GRACE, 0, 1), 'escape')

    # ── 7. 여유를 0 으로 두면 즉시 받는다 ───────────────────────────
    check('grace=0 → 발행 직후 통보도 받는다',
          abort_decision(100.0, 100.0, 0.0, 0, MAX), 'retarget')

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('12개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
