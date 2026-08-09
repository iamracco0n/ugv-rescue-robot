#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""수색 로그를 정답과 대조해 점수표를 출력한다.

    python3 tools/score_run.py <run.log> --truth <truth.json> [--min-victims 7]

왜 필요한가
-----------
지금까지 검증이 "한 시간짜리 런을 띄워놓고 로그를 눈으로 읽는 것" 이었다.
그 방식으로 놓쳤다가 뒤늦게 잡은 회귀가 실제로 여러 건이다.
  · 침상 환자가 L1 → L2 로 떨어짐 (어깨 높이 규칙을 넣다가)
  · 서 있는 사람이 L3 → L2 로 과대평가 (임계값 오설정)
  · 같은 조난자가 8번째로 중복 등록 (중복 반경 역설)
전부 우연히 로그를 보다 발견했고 한 건당 한 시간씩 들었다.
이 스크립트는 같은 판정을 몇 초 만에, 빠짐없이 한다.

종료 코드 0 = 합격, 1 = 불합격 (CI 에서 그대로 쓸 수 있다)
"""
import argparse
import json
import math
import re
import sys

# [구조 로그] #0 L2:NeedHelp | 거리:3.1m | 위치:(11.9,9.7) | Room B | 표본5개 산포0.03m
RE_REG = re.compile(
    r'\[구조 로그\]\s*#(\d+)\s+L(\d):(\w+)\s*\|\s*거리:([\d.]+)m\s*\|\s*'
    r'위치:\(([-\d.]+),([-\d.]+)\)')
# #2 위치 갱신 (23.7,0.6) → (24.2,0.5) — 더 가까이서 재관측 4.0m → 3.1m
RE_UPD = re.compile(r'#(\d+) 위치 갱신 \([-\d.]+,[-\d.]+\) → \(([-\d.]+),([-\d.]+)\)')
RE_FIRE = re.compile(r'화재 발견!\s*map \(([-\d.]+),\s*([-\d.]+)\)')

RE_ALLFOUND = re.compile(r'전원 발견!\s*조난자 (\d+)/(\d+)명')
# ROS 로그 앞머리의 절대 시각 — 소요 시간 계산용
RE_STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
RE_SWEEP_DONE = re.compile(r'수색 (\d+)회차 완료')
RE_RESWEEP = re.compile(r'(\d+)회차 재수색 시작')

COUNTERS = {
    'nav_dead':   r'내비게이션이 죽은',
    'manual_bug': r'외부 goal 수신',
    'unreached':  r'도달 실패',
    'stuck':      r'못 움직임',
    'ghost':      r'유령 후보',
    'hold':       r'등록 보류',
    'detour':     r'우회 지점',
    'debris':     r'잔해가 빽빽',
    'traceback':  r'Traceback',
}
LV_NAME = {1: 'L1:Critical', 2: 'L2:NeedHelp', 3: 'L3:Normal'}


def strip_ansi(s):
    return re.sub(r'\x1b\[[0-9;]*m', '', s)


def parse(path):
    """로그에서 최종 등록 목록·화재·주요 카운터를 뽑는다."""
    regs, fires = {}, []
    counts = {k: 0 for k in COUNTERS}
    flags = {'all_found': None, 'sweep_done': 0, 'resweep': 0,
             't0': None, 't_end': None, 't_all': None, 't_first': None}
    pats = {k: re.compile(v) for k, v in COUNTERS.items()}

    with open(path, encoding='utf-8', errors='replace') as f:
        for raw in f:
            line = strip_ansi(raw)
            ms = RE_STAMP.search(line)
            if ms:
                t = float(ms.group(1))
                if flags['t0'] is None:
                    flags['t0'] = t
                flags['t_end'] = t
            m = RE_REG.search(line)
            if m:
                pid = int(m.group(1))
                regs[pid] = {'lv': int(m.group(2)),
                             'x': float(m.group(5)), 'y': float(m.group(6)),
                             'dist': float(m.group(4))}
                continue
            m = RE_UPD.search(line)
            if m:                       # 정밀화된 좌표가 최종값이다
                pid = int(m.group(1))
                if pid in regs:
                    regs[pid]['x'] = float(m.group(2))
                    regs[pid]['y'] = float(m.group(3))
                continue
            m = RE_FIRE.search(line)
            if m:
                fires.append((float(m.group(1)), float(m.group(2))))
                continue
            m = RE_ALLFOUND.search(line)
            if m:
                flags['all_found'] = (int(m.group(1)), int(m.group(2)))
                if flags['t_all'] is None and flags['t0'] is not None:
                    flags['t_all'] = flags['t_end'] - flags['t0']
            if RE_SWEEP_DONE.search(line):
                flags['sweep_done'] += 1
            if RE_RESWEEP.search(line):
                flags['resweep'] += 1
            for k, p in pats.items():
                if p.search(line):
                    counts[k] += 1
    return regs, fires, counts, flags


def match(items, truth, max_err):
    """등록↔정답 최근접 1:1 매칭. (매칭쌍, 미발견, 오탐)"""
    pairs, used = [], set()
    for key, it in sorted(items.items()):
        best, bd = None, 1e9
        for i, t in enumerate(truth):
            if i in used:
                continue
            d = math.hypot(it['x'] - t['x'], it['y'] - t['y'])
            if d < bd:
                bd, best = d, i
        if best is not None and bd <= max_err:
            used.add(best)
            pairs.append((key, it, truth[best], bd))
        else:
            pairs.append((key, it, None, bd))
    missed = [t for i, t in enumerate(truth) if i not in used]
    false_pos = [p for p in pairs if p[2] is None]
    return pairs, missed, false_pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--truth', required=True)
    ap.add_argument('--max-err', type=float, default=2.0,
                    help='이 거리 안이면 같은 대상으로 매칭 (m)')
    ap.add_argument('--max-mean-err', type=float, default=1.0,
                    help='평균 위치오차 합격선 (m)')
    args = ap.parse_args()

    truth = json.load(open(args.truth, encoding='utf-8'))
    tv, tf = truth['victims'], truth['fires']
    regs, fires, counts, flags = parse(args.log)

    fails = []
    print('=' * 66)
    print(f"수색 채점 — {truth.get('world', '?')}")
    print('=' * 66)

    # ── 조난자 ────────────────────────────────────────────────────────
    pairs, missed, fps = match(regs, tv, args.max_err)
    errs, tri_ok = [], 0
    print(f'\n[조난자] 등록 {len(regs)}건 / 정답 {len(tv)}명')
    print(f'  {"#":>3} {"판정":<12}{"등록위치":>16}{"정답":>16}{"오차":>7}  대상')
    for pid, it, t, d in pairs:
        got = LV_NAME.get(it['lv'], f"L{it['lv']}")
        if t is None:
            print(f'  {pid:>3} {got:<12}({it["x"]:6.1f},{it["y"]:6.1f})'
                  f'{"—":>16}{"—":>7}  ← 오탐(정답에 없음)')
            continue
        errs.append(d)
        ok = (it['lv'] == t['triage'])
        tri_ok += ok
        mark = '' if ok else f'  ← 오판(정답 {LV_NAME[t["triage"]]})'
        print(f'  {pid:>3} {got:<12}({it["x"]:6.1f},{it["y"]:6.1f})'
              f'({t["x"]:6.1f},{t["y"]:6.1f}){d:7.2f}  {t["name"]}{mark}')
    for t in missed:
        print(f'  {"—":>3} {"미발견":<12}{"—":>16}'
              f'({t["x"]:6.1f},{t["y"]:6.1f}){"—":>7}  {t["name"]}')

    mean_err = sum(errs) / len(errs) if errs else float('nan')
    if missed:
        fails.append(f'조난자 {len(missed)}명 미발견')
    if fps:
        fails.append(f'오탐 {len(fps)}건')
    if len(regs) > len(tv):
        fails.append(f'과다 등록 {len(regs)}>{len(tv)} (중복 의심)')
    if errs and tri_ok < len(errs):
        fails.append(f'트리아지 오판 {len(errs)-tri_ok}건')
    if errs and mean_err > args.max_mean_err:
        fails.append(f'평균 오차 {mean_err:.2f}m > {args.max_mean_err}m')

    # ── 화재 ──────────────────────────────────────────────────────────
    fitems = {i: {'x': x, 'y': y} for i, (x, y) in enumerate(fires)}
    fpairs, fmissed, ffps = match(fitems, tf, args.max_err)
    ferrs = [d for _, _, t, d in fpairs if t is not None]
    print(f'\n[화재] 발견 {len(fires)}건 / 정답 {len(tf)}건')
    for i, it, t, d in fpairs:
        if t is None:
            print(f'  ({it["x"]:6.1f},{it["y"]:6.1f})  ← 오탐')
        else:
            print(f'  ({it["x"]:6.1f},{it["y"]:6.1f}) → {t["name"]}  오차 {d:.2f}m')
    for t in fmissed:
        print(f'  미발견  ({t["x"]:6.1f},{t["y"]:6.1f})  {t["name"]}')
    if fmissed:
        fails.append(f'화재 {len(fmissed)}건 미발견')
    if ffps:
        fails.append(f'화재 오탐 {len(ffps)}건')

    # ── 건전성 ────────────────────────────────────────────────────────
    print('\n[건전성]')
    print(f'  Nav2 사망 {counts["nav_dead"]}  goal오인 {counts["manual_bug"]}  '
          f'도달실패 {counts["unreached"]}  박힘 {counts["stuck"]}')
    print(f'  유령 {counts["ghost"]}  등록보류 {counts["hold"]}  '
          f'우회접근 {counts["detour"]}  잔해구역 {counts["debris"]}  '
          f'예외 {counts["traceback"]}')
    for k, label in (('nav_dead', 'Nav2 사망'), ('manual_bug', 'goal 오인'),
                     ('traceback', '예외(Traceback)')):
        if counts[k]:
            fails.append(f'{label} {counts[k]}회')

    # ── 임무 보고 ─────────────────────────────────────────────────────
    print('\n[임무 보고]')
    if flags['t0'] is not None and flags['t_end'] is not None:
        print(f'  런 길이: {flags["t_end"] - flags["t0"]:.0f}초')
    if flags['all_found']:
        a, b = flags['all_found']
        t = flags['t_all']
        when = f' — {t:.0f}초 만에' if t else ''
        if fps or missed:
            # 보고는 등록 '수' 만 본다. 오탐이 수를 채우면 실제로는 못 찾은
            # 조난자가 있는데도 임무 완료로 보고된다. 구조에서 제일 위험한
            # 오류라 눈에 띄게 표시한다.
            print(f'  전원 발견 보고: {a}/{b}명 ⚠ 잘못된 보고{when}')
            print(f'     실제 발견 {len(errs)}명 · 오탐 {len(fps)}건 · '
                  f'미발견 {len(missed)}명')
            fails.append('오탐이 수를 채워 전원 발견이 잘못 보고됨')
        else:
            print(f'  전원 발견 보고: {a}/{b}명 ✅{when}')
    else:
        print('  전원 발견 보고: 없음 (시간 내 미달성)')
    print(f'  회차 완료 보고: {flags["sweep_done"]}회')
    print(f'  재수색 시작:    {flags["resweep"]}회')

    # ── 결과 ──────────────────────────────────────────────────────────
    print('\n' + '-' * 66)
    print(f'조난자 {len(errs)}/{len(tv)}  트리아지 {tri_ok}/{len(errs) or 0} 정확  '
          f'평균오차 {mean_err:.2f}m  화재 {len(ferrs)}/{len(tf)}')
    if fails:
        print('불합격:')
        for f in fails:
            print(f'  · {f}')
        print('=' * 66)
        return 1
    print('합격 — 모든 항목 통과')
    print('=' * 66)
    return 0


if __name__ == '__main__':
    sys.exit(main())
