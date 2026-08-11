#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""수집한 시뮬 데이터로 YOLO 사람 검출기를 파인튜닝한다.

    python3 tools/train_yolo.py ~/ugv_dataset --epochs 60

왜 필요한가
-----------
유령 후보(잔해를 사람으로 오인)가 탐사 시간의 30%를 먹는다.
거리·지속성 어느 신호로도 진짜와 구분되지 않았다(README 참조).
COCO 로 학습된 기본 가중치가 Gazebo 상자 더미에 사람 골격을 그리는 것이
원인이므로, 이 환경의 데이터로 다시 학습시키는 편이 근본적이다.

핵심은 '유령 프레임을 배경으로 명시적으로 학습' 시키는 것이다.
dataset_capture_node 가 유령을 빈 라벨로 저장하므로 그대로 하드
네거티브가 된다. 배경 비율이 너무 높으면 검출이 보수적으로 치우치니
--max-bg-ratio 로 상한을 둔다.

주의: 여기서 만드는 것은 '사람 검출기'(detect)다. 자세 판정은 여전히
yolov8n-pose 가 하므로, 이 모델을 쓰려면 파이프라인에서 검출 단계만
교체해야 한다. 그 연결은 별도 작업이다.
"""
import argparse
import os
import random
import shutil
import sys


def collect(root):
    """수집 디렉토리들을 훑어 (이미지, 라벨) 쌍을 모은다."""
    pairs = []
    for sub, _, files in os.walk(root):
        if os.path.basename(sub) != 'images':
            continue
        lbl_dir = os.path.join(os.path.dirname(sub), 'labels')
        for f in files:
            if not f.lower().endswith(('.jpg', '.png')):
                continue
            lbl = os.path.join(lbl_dir, os.path.splitext(f)[0] + '.txt')
            if os.path.exists(lbl):
                pairs.append((os.path.join(sub, f), lbl))
    return pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset_root')
    ap.add_argument('--out', default=os.path.expanduser('~/ugv_yolo_train'))
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=32)
    ap.add_argument('--base', default='yolov8n.pt')
    ap.add_argument('--val-frac', type=float, default=0.2)
    ap.add_argument('--max-bg-ratio', type=float, default=2.0,
                    help='양성 1장당 배경 최대 장수')
    ap.add_argument('--max-pos-ratio', type=float, default=4.0,
                    help='배경 1장당 양성 최대 장수. 실측에서 양성이 배경보다 '
                         '13배 많아(211:16) 유령 억제 학습이 묻혔다. '
                         '유령을 막는 것이 목적이므로 양성 쪽도 제한한다')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    pairs = collect(os.path.expanduser(args.dataset_root))
    if not pairs:
        sys.exit(f'데이터가 없다: {args.dataset_root}')

    pos = [p for p in pairs if os.path.getsize(p[1]) > 0]
    bg = [p for p in pairs if os.path.getsize(p[1]) == 0]
    print(f'수집본: 총 {len(pairs)}장 (양성 {len(pos)} · 배경 {len(bg)})')
    if not pos:
        sys.exit('양성 표본이 없다 — 정답 매칭(match_r)이나 수집 조건을 확인할 것')

    random.seed(args.seed)
    random.shuffle(bg)
    random.shuffle(pos)
    keep_bg = bg[:int(len(pos) * args.max_bg_ratio)]
    if len(keep_bg) < len(bg):
        print(f'배경을 {len(bg)} → {len(keep_bg)}장으로 제한 '
              f'(양성 1장당 최대 {args.max_bg_ratio}장). '
              '배경이 과하면 검출이 지나치게 보수적으로 치우친다')
    # 반대로 양성이 압도적이면 유령 억제 학습이 묻힌다(실측 211:16).
    keep_pos = pos[:max(1, int(len(keep_bg) * args.max_pos_ratio))]
    if len(keep_pos) < len(pos):
        print(f'양성을 {len(pos)} → {len(keep_pos)}장으로 제한 '
              f'(배경 1장당 최대 {args.max_pos_ratio}장). '
              '목적은 유령 억제이므로 배경이 묻히면 안 된다')
    data = keep_pos + keep_bg
    random.shuffle(data)

    n_val = max(1, int(len(data) * args.val_frac))
    splits = {'val': data[:n_val], 'train': data[n_val:]}

    out = os.path.expanduser(args.out)
    if os.path.exists(out):
        shutil.rmtree(out)
    for sp, items in splits.items():
        for sub in ('images', 'labels'):
            os.makedirs(os.path.join(out, sp, sub), exist_ok=True)
        for img, lbl in items:
            base = os.path.basename(img)
            shutil.copy2(img, os.path.join(out, sp, 'images', base))
            shutil.copy2(lbl, os.path.join(out, sp, 'labels',
                                           os.path.splitext(base)[0] + '.txt'))
        print(f'  {sp}: {len(items)}장')

    yaml_path = os.path.join(out, 'data.yaml')
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(f'path: {out}\ntrain: train/images\nval: val/images\n'
                'names:\n  0: person\n')

    from ultralytics import YOLO
    model = YOLO(args.base)
    model.train(data=yaml_path, epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, project=out, name='run', exist_ok=True)
    print(f'\n학습 완료. 가중치: {out}/run/weights/best.pt')
    print('평가는 시뮬 채점으로 할 것 — mAP 가 아니라 유령 건수가 목적이다:')
    print('  tools/run_eval.sh ghost_bench 3 900')


if __name__ == '__main__':
    main()
