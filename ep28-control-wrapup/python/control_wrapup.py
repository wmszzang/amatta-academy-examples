"""네 실기 학습 문제의 계산과 상태 저장 규약."""
from pathlib import Path
import csv


def plant_pid(kp=2., ki=1., kd=.1, gain=1., tau=1., target=1., dt=.05, count=200):
    y, integral, prev = 0., 0., 0.
    rows = []
    for k in range(count):
        start, end = k*dt, (k+1)*dt
        before = y
        error = target-before
        integral += error*dt
        derivative = (error-prev)/dt
        command = kp*error + ki*integral + kd*derivative
        y = before + dt*(gain*command-before)/tau
        rows.append((start, before, command, end, y))
        prev = error
    return rows


def limited_pid(errors, upper=15., lower=-100., kp=1., ki=1., kd=0., dt=1., hold=True):
    integral, prev = 0., 0.
    rows = []
    for n, error in enumerate(errors):
        before = integral
        trial = integral + ki*error*dt
        derivative = 0. if n == 0 else (error-prev)/dt
        raw = kp*error + trial + kd*derivative
        command = max(lower, min(upper, raw))
        if not hold or command == raw:
            integral = trial
        rows.append((n+1, error, before, trial, raw, command, integral))
        prev = error
    return rows


def kalman(moves=(1., 1.), measurements=(1.2, 2.1), process_var=.1, sensor_var=.5):
    position, variance = 0., 1.
    rows = []
    for n, (move, measured) in enumerate(zip(moves, measurements)):
        predicted = position+move
        predicted_var = variance+process_var
        weight = predicted_var/(predicted_var+sensor_var)
        residual = measured-predicted
        position = predicted+weight*residual
        variance = (1-weight)*predicted_var
        rows.append((n+1, predicted, predicted_var, weight, residual, position, variance))
    return rows


def complementary(alpha=.95, dt=.1, count=20):
    angle = 0.
    rows = []
    for k in range(1, count+1):
        time = k*dt
        measured_angle = 10*time
        predicted = angle+12*dt
        angle = alpha*predicted+(1-alpha)*measured_angle
        rows.append((k, time, predicted, measured_angle, angle, 12*time))
    return rows


def results():
    return {
        'plant': (('t_start','y_before','command','t_end','y_after'), plant_pid()),
        'normal': (('step','error','i_before','i_trial','raw','command','i_after'), limited_pid([10.]*4, upper=100.)),
        'hold': (('step','error','i_before','i_trial','raw','command','i_after'), limited_pid([10.,10.,10.,-10.])),
        'no_hold': (('step','error','i_before','i_trial','raw','command','i_after'), limited_pid([10.,10.,10.,-10.], hold=False)),
        'boundary': (('step','error','i_before','i_trial','raw','command','i_after'), limited_pid([7.5])),
        'kalman': (('step','predicted','predicted_var','weight','residual','position','variance'), kalman()),
        'complementary': (('step','time','predicted','measured','angle','gyro_only'), complementary()),
    }


def verify():
    from math import isclose
    p = plant_pid()
    assert len(p) == 200 and p[0][3] == .05 and p[-1][3] == 10.
    assert isclose(.1*(1.-0.)/.05, 2.)
    assert isclose(p[0][4], .2025, abs_tol=1e-12)
    assert isclose(p[-1][4], .9939577634244283, abs_tol=1e-12)
    assert [r[5] for r in limited_pid([10.]*4,upper=100.)] == [20.,30.,40.,50.]
    assert [r[5] for r in limited_pid([10.,10.,10.,-10.])] == [15.,15.,15.,-20.]
    assert limited_pid([10.,10.,10.,-10.],hold=False)[-1][5] == 10.
    assert limited_pid([7.5])[0][6] == 7.5
    # 적분이 보류되어도 이전 오차는 갱신해야 두 번째 미분항이 0이다.
    assert limited_pid([10.,10.],kd=2.)[1][4] == 20.
    k = kalman()
    assert isclose(k[0][5], 1.1375, abs_tol=1e-12)
    assert isclose(k[0][6], .34375, abs_tol=1e-12)
    assert isclose(k[1][5], 3201/1510, abs_tol=1e-12)
    assert isclose(k[1][6], 71/302, abs_tol=1e-12)
    for row in complementary():
        assert isclose(row[4], row[0]+3.8*(1-.95**row[0]), abs_tol=1e-12)
    assert isclose(complementary()[-1][4],22.437753494847524,abs_tol=1e-12)
    assert isclose(complementary(alpha=1.)[-1][4],24.,abs_tol=1e-12)
    assert complementary(alpha=0.)[-1][4] == 20.


def main():
    verify()
    for name, (headers, rows) in results().items():
        with Path(name+'.csv').open('w',encoding='utf-8',newline='') as f:
            writer = csv.writer(f,lineterminator='\n')
            writer.writerow(headers)
            writer.writerows([[f'{v:.9f}' for v in row] for row in rows])
        print(name+': '+', '.join(f'{v:.6f}' for v in rows[-1]))


if __name__ == '__main__':
    main()
