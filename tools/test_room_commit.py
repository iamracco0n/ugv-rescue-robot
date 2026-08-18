#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""방 커밋 판정 단위 테스트.

    python3 tools/test_room_commit.py

왜 필요한가
-----------
이 규칙은 로봇을 한 방에 묶어 둔다. 조건이 틀리면 두 방향으로 고장난다.

  · 너무 잘 걸리면  -> 한 방에 갇혀 나머지 조난자를 통째로 놓친다
  · 안 걸리면       -> 기능이 없는 것과 같다(예전 room_bonus 가 그랬다)

시뮬에서는 둘 다 '완주 못 함' 으로만 보여서 원인이 안 갈린다. 규칙만 떼어
잠가 둔다.

무엇을 잠그나
-------------
  · 기본값(0)에서는 절대 안 걸린다 — 기존 동작 보존
  · 자투리만 남았으면 안 눌러앉는다
  · 사람이 숨을 만큼 남았으면 눌러앉는다
  · 상한 시간을 넘기면 놓아 준다(한 방에 갇히는 것 방지)
  · 상한 0 은 '시간 제한 없음' 이다
"""
import ast
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load(name):
    with open(SRC, encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            ns = {}
            exec(compile(ast.Module([node], []), SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{name} 을 소스에서 못 찾았다')


decide = load('room_commit_decision')
fails = []


def check(name, cond):
    if not cond:
        fails.append(name)


# 꺼짐이 기본 — 이게 깨지면 검증한 82% 구성이 통째로 바뀐다
check('임계 0 이면 안 걸림', decide(999.0, 0.0, 0.0, 240.0) is False)
check('임계 음수면 안 걸림', decide(999.0, -1.0, 0.0, 240.0) is False)

# 자투리는 무시. 방 이탈 계측과 같은 기준이라야 서로 안 어긋난다
check('자투리면 안 걸림', decide(3.0, 20.0, 0.0, 240.0) is False)
check('임계 바로 아래면 안 걸림', decide(19.9, 20.0, 0.0, 240.0) is False)

# 사람이 숨을 만하면 눌러앉는다
check('임계 이상이면 걸림', decide(20.0, 20.0, 0.0, 240.0) is True)
check('많이 남으면 걸림', decide(84.0, 20.0, 10.0, 240.0) is True)

# 갇히지 않는다 — 이 장치가 없으면 한 방에서 런이 끝난다
check('상한 넘으면 풀림', decide(84.0, 20.0, 240.0, 240.0) is False)
check('상한 직전엔 유지', decide(84.0, 20.0, 239.0, 240.0) is True)
check('상한 0 은 무제한', decide(84.0, 20.0, 9999.0, 0.0) is True)

# 남은 넓이가 줄면 스스로 풀린다 — 다 봤으면 나가야 한다
check('다 보면 풀림', decide(0.0, 20.0, 10.0, 240.0) is False)

if fails:
    print(f'실패 {len(fails)}건')
    for f in fails:
        print('  ' + f)
    sys.exit(1)
print('10개 사례 전부 통과')
