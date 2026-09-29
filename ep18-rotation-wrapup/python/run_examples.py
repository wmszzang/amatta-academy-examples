"""지문별 산출물. 계산은 원정밀도, CSV 출력만 소수 넷째 자리."""
import csv
import math
from pathlib import Path
from rot import (unit, mul, euler_zyx_to_R, euler_zyx, rodrigues,
                 q_to_R, R_to_q, hamilton, slerp, geodesic_deg, gyro_heading)


def determinant(R):
    return sum(R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]-
                        R[1][(i+2)%3]*R[2][(i+1)%3]) for i in range(3))


def error(a, b):
    return max(abs(a[i][j]-b[i][j]) for i in range(3) for j in range(3))


def flat(R):
    return [v for row in R for v in row]


def write(name, header, rows, out):
    with (Path(out)/name).open('w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(header.split(','))
        for row in rows:
            writer.writerow([f'{0.0 if abs(v)<0.00005 else v:.4f}'
                             if isinstance(v, float) else v for v in row])
    print(name)
    for row in rows:
        print(','.join(f'{0.0 if abs(v)<0.00005 else v:.4f}'
                       if isinstance(v, float) else str(v) for v in row))


def run(problem, out='.'):
    Path(out).mkdir(parents=True, exist_ok=True)
    eye = [[float(i == j) for j in range(3)] for i in range(3)]
    q0 = [1., 0., 0., 0.]
    z90, x90 = unit([1., 0., 0., 1.]), unit([1., 1., 0., 0.])
    cols = ','.join(f'r{i}{j}' for i in range(3) for j in range(3))
    checks = []
    if problem == 1:
        rows = []
        cases = [('general', .3, -.5, 1.2), ('lock_a', .4, math.pi/2, .9),
                 ('lock_b', 0., math.pi/2, .5), ('lock_c', 1., math.pi/2, 1.5),
                 ('lock_negative', .4, -math.pi/2, .9)]
        for name, roll, pitch, yaw in cases:
            R = euler_zyx_to_R(roll, pitch, yaw)
            r, p, y, lock = euler_zyx(R)
            checks.append(error(R, euler_zyx_to_R(r, p, y)))
            rows.append([name]+flat(R)+[r, p, y, int(lock)])
        write('euler.csv', 'case,'+cols+',roll,pitch,yaw,lock', rows, out)
    elif problem == 2:
        rows = []
        for k, theta in [([1.,2.,2.],1.),([0.,0.,1.],math.pi/2),([1.,1.,1.],2*math.pi/3)]:
            R = rodrigues(k, theta)
            rows.append(k+[theta]+flat(R)+[determinant(R)])
            checks.append(error(R, q_to_R(R_to_q(R))))
        write('rod.csv', 'kx,ky,kz,theta,'+cols+',det', rows, out)
    elif problem == 3:
        rows = []
        for q in [[.7071,0.,.7071,0.], [1.,1.,1.,1.]]:
            R = q_to_R(q)
            orth = error(mul(list(map(list, zip(*R))), R), eye)
            rows.append(q+flat(R)+[orth])
            checks.append(error(R, q_to_R(R_to_q(R))))
        write('quat.csv', 'w,x,y,z,'+cols+',orth_err', rows, out)
    elif problem == 4:
        rows = []
        for name, a, b, normalized in [('290_unit_z_z',z90,z90,True),
                ('490_raw_identity_x180',q0,[0.,1.,0.,0.],False),
                ('490_raw_z_x',z90,x90,False),('490_raw_x_z',x90,z90,False)]:
            q = hamilton(a, b)
            if normalized:
                q = unit(q)
            rows.append([name]+q+[geodesic_deg(q0,q)])
            checks.append(error(q_to_R(q), mul(q_to_R(a),q_to_R(b))))
        write('hamilton.csv', 'order,w,x,y,z,geo_deg', rows, out)
    elif problem == 5:
        rows = []
        for t in [0.,.1,.25,.5,.75,.9,1.]:
            q = slerp(q0,z90,t)
            rows.append([t]+q+[geodesic_deg(q0,q),'SLERP'])
            checks.append(error(q_to_R(q),q_to_R(R_to_q(q_to_R(q)))))
        q = slerp(q0,[1.,0.,0.,.01],.5)
        rows.append([.5]+q+[geodesic_deg(q0,q),'LERP_NEAR'])
        write('slerp.csv', 't,w,x,y,z,angle_deg,mode', rows, out)
    elif problem == 6:
        cases = [('right_angle',z90),('same',q0),('opposite_sign',[-1.,0.,0.,0.]),('half_turn',[0.,0.,0.,1.])]
        rows = [[name,geodesic_deg(q0,q)] for name,q in cases]
        write('dist.csv', 'case,deg', rows, out)
        checks = [error(q_to_R(q),q_to_R(R_to_q(q_to_R(q)))) for _,q in cases]
    elif problem == 7:
        omega, dt, bias = [.1,.2,.2,.1], .5, .1
        rows = [[i,w,w-bias,gyro_heading(omega[:i+1],dt,bias)] for i,w in enumerate(omega)]
        write('gyro.csv', 'i,omega,corrected,theta', rows, out)
        theta = gyro_heading(omega,dt,bias)
        q = [math.cos(theta/2),0.,0.,math.sin(theta/2)]
        checks = [abs(geodesic_deg(q0,q)-math.degrees(theta))]
    else:
        raise ValueError('문제 묶음은 1~7입니다.')
    assert max(checks) < 1e-10, checks
    print('verification max_error < 1e-10: PASS')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='.')
    args = parser.parse_args()
    for problem in range(1,8):
        run(problem,args.out)
