"""회전행렬의 ZYX 분해와 단일축 자이로 적분 예제."""
import argparse
import csv
import math
from pathlib import Path
import random
import sys


def euler_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]]


def rot_to_euler_zyx(R, eps=1e-6):
    pitch = math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0]))
    if abs(math.cos(pitch)) < eps:
        yaw = 0.0
        roll = math.atan2(-R[1][2], R[1][1])
    else:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    return roll, pitch, yaw


def roundtrip_err(R):
    Q = euler_to_R(*rot_to_euler_zyx(R))
    return max(abs(Q[i][j]-R[i][j]) for i in range(3) for j in range(3))


def heading_from_gyro(omega, dt, bias, theta0=0.0):
    w = [x-bias for x in omega]
    return theta0 + sum(0.5*(a+b)*dt for a, b in zip(w, w[1:]))


def self_test():
    rng = random.Random(15)
    errors = []
    for _ in range(5000):
        R = euler_to_R(rng.uniform(-math.pi, math.pi),
                       rng.uniform(-1.56, 1.56), rng.uniform(-math.pi, math.pi))
        errors.append(roundtrip_err(R))
    assert max(errors) < 1e-14
    for pitch, expected in [(math.pi/2, -0.5), (-math.pi/2, 0.9)]:
        R = euler_to_R(0.2, pitch, 0.7)
        assert abs(rot_to_euler_zyx(R)[0]-expected) < 1e-14
        assert roundtrip_err(R) < 1e-14
    # 임계값 안팎의 근사 오차는 정확한 직각의 반올림 오차와 구별한다.
    for delta in (0.5e-6, 2e-6):
        assert roundtrip_err(euler_to_R(0.2, math.pi/2-delta, 0.7)) < 1e-6
    assert abs(heading_from_gyro([0.1, 0.2, 0.2, 0.1], .5, .1)-.1) < 1e-15
    # 사다리꼴과 왼쪽 사각형이 달라지는 입력으로 적분법도 확인한다.
    assert abs(heading_from_gyro([0.1, 0.3], .5, .1)-.05) < 1e-15
    return max(errors)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stdin', action='store_true')
    parser.add_argument('--out', default='.')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    cases = [('normal', euler_to_R(.3, -.5, 1.2)),
             ('plus90', euler_to_R(.2, math.pi/2, .7)),
             ('minus90', euler_to_R(.2, -math.pi/2, .7))]
    if args.stdin:
        values = list(map(float, sys.stdin.read().split()))
        if len(values) != 9 or not all(map(math.isfinite, values)):
            parser.error('유한한 숫자 아홉 개를 행 우선 순서로 입력하세요.')
        cases = [('input', [values[i:i+3] for i in (0, 3, 6)])]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with (out/'euler.csv').open('w', encoding='utf-8', newline='') as f, \
         (out/'roundtrip.csv').open('w', encoding='utf-8', newline='') as g:
        writer, check = csv.writer(f), csv.writer(g)
        writer.writerow(['case', 'roll', 'pitch', 'yaw'])
        check.writerow(['case', 'max_error'])
        for name, R in cases:
            angles = rot_to_euler_zyx(R)
            writer.writerow([name, *[f'{a:.4f}' for a in angles]])
            check.writerow([name, f'{roundtrip_err(R):.12e}'])
            print(name, *(f'{a:.4f}' for a in angles), f'error={roundtrip_err(R):.12e}')
    heading = heading_from_gyro([.1, .2, .2, .1], .5, .1)
    (out/'heading.txt').write_text(f'{heading:.4f}\n', encoding='utf-8')
    print(f'heading={heading:.4f}')
    if args.self_test:
        print(f'random 5000 roundtrips max={self_test():.12e}')


if __name__ == '__main__':
    main()
