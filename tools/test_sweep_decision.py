#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""수색 종료·재수색 판정 규칙 단위 테스트.

    python3 tools/test_sweep_decision.py

왜 단위 테스트인가
------------------
'회차 완료' 와 '재수색' 두 경로는 커버리지 완료 조건 뒤에 묶여 있다.
그런데 큰 월드에서는 70분을 돌려도 커버리지가 안 끝나서, 시뮬로는
두 경로를 한 번도 발동시키지 못했다(실측: 경계 7345셀/기준 40).
규칙만 떼어내면 시뮬 없이 확인할 수 있다.

rclpy 없이 돌도록 노드 모듈에서 함수만 읽어 실행한다.
"""
import ast
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load_sweep_decision():
    """patrol_navigator 에서 sweep_decision 함수만 떼어내 실행 가능하게 만든다."""
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'sweep_decision':
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {}
            exec(compile(mod, SRC, 'exec'), ns)   # noqa: S102 — 테스트 전용
            return ns['sweep_decision']
    raise SystemExit('sweep_decision 을 찾지 못했다 — 함수명이 바뀌었나?')


sweep_decision = load_sweep_decision()

# 실제 운용값 (patrol_navigator 기본 파라미터)
DONE_CELLS, MIN_GOALS, MIN_AREA = 40, 8, 200.0

CASES = [
    # (설명, 경계셀, 미관측, 예산, 목표수, 자유면적, 찾은수, 실종자수, 기대)
    ('아직 탐사 초반 — 계속',
     9848, 626.0, 44.0, 3, 150.0, 0, 7, 'continue'),
    ('경계는 없앴지만 미관측이 남음 — 계속',
     20, 300.0, 44.0, 50, 2000.0, 6, 7, 'continue'),
    ('미관측은 적지만 경계가 남음 — 계속',
     500, 20.0, 44.0, 50, 2000.0, 7, 7, 'continue'),
    ('다 훑고 전원 발견 — 완료',
     20, 30.0, 44.0, 50, 2000.0, 7, 7, 'done'),
    ('다 훑었는데 인원 부족 — 재수색',
     20, 30.0, 44.0, 50, 2000.0, 6, 7, 'resweep'),
    ('다 훑었고 실종자 수 모름(0) — 완료',
     20, 30.0, 44.0, 50, 2000.0, 3, 0, 'done'),
    ('찾은 수가 실종자보다 많아도 — 완료',
     20, 30.0, 44.0, 50, 2000.0, 8, 7, 'done'),
    ('기동 직후 지표만 좋아 보임 — 목표수 부족이라 계속',
     0, 0.0, 44.0, 1, 20.0, 0, 7, 'continue'),
    ('맵이 아직 작음 — 면적 부족이라 계속',
     0, 0.0, 44.0, 50, 50.0, 7, 7, 'continue'),
    ('경계 딱 기준선 — 완료(<=)',
     40, 44.0, 44.0, 8, 200.0, 7, 7, 'done'),
    ('경계 기준선+1 — 계속',
     41, 44.0, 44.0, 8, 200.0, 7, 7, 'continue'),
]


def main():
    print(f'{"사례":<42}{"기대":>9}{"결과":>9}  판정')
    bad = 0
    for (name, cells, unseen, budget, goals, area, vic, exp, want) in CASES:
        got = sweep_decision(cells, unseen, budget, DONE_CELLS,
                             goals, MIN_GOALS, area, MIN_AREA, vic, exp)
        ok = got == want
        if not ok:
            bad += 1
        print(f'{name:<42}{want:>9}{got:>9}  {"OK" if ok else "실패"}')
    print()
    if bad:
        print(f'실패 {bad}건')
        return 1
    print(f'{len(CASES)}개 사례 전부 통과')
    print('\n주의: 이 테스트는 판정 "규칙" 만 본다. 실제 시뮬에서 커버리지가')
    print('      완료 수준까지 수렴하는지는 별개 문제다(현재 미달성).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
