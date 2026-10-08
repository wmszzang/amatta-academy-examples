import csv
import math
from pathlib import Path


def pid_trace(Kp, Ki, Kd, dt, N, r=1.0):
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive and finite")
    x, integ, e_prev = 0.0, 0.0, r
    rows = []
    for k in range(N + 1):
        e = r - x
        integ += e * dt
        deriv = (e - e_prev) / dt
        u = Kp * e + Ki * integ + Kd * deriv
        rows.append((k * dt, x, e, integ, Kp * e, Ki * integ, Kd * deriv, u))
        x += u * dt
        e_prev = e
    return rows


def pid_control(Kp, Ki, Kd, dt, N, r=1.0):
    return [row[1] for row in pid_trace(Kp, Ki, Kd, dt, N, r)]


def aw_trace(errs, Kp, Ki, Kd, dt, umin, umax, enabled=True):
    if not math.isfinite(dt) or dt <= 0 or umin > umax:
        raise ValueError("invalid time step or output bounds")
    I, prev = 0.0, 0.0
    rows = []
    for n, e in enumerate(errs):
        P = Kp * e
        I_try = I + Ki * e * dt
        D = 0.0 if n == 0 else Kd * (e - prev) / dt
        u_un = P + I_try + D
        u = max(umin, min(umax, u_un))
        old_I = I
        if u == u_un or not enabled:
            I = I_try
        rows.append((n, e, old_I, I_try, P, D, u_un, u, I))
        prev = e
    return rows


def pid_aw(errs, Kp, Ki, Kd, dt, umin, umax):
    return [round(row[7], 4) for row in aw_trace(errs, Kp, Ki, Kd, dt, umin, umax)]


def write_csv(name, header, rows):
    with open(name, "w", newline="", encoding="utf-8") as out:
        out.write(header + "\n")
        for row in rows:
            out.write(",".join(f"{v:.10f}" for v in row) + "\n")


def main():
    pos = pid_trace(2, .5, .1, .1, 30)
    write_csv("position.csv", "t,x,e,J,P,I,D,u", pos)
    cases = [("A", [10]*4, 1,1,0,1,-100,100,True),
             ("B", [10,10,10,-10],1,1,0,1,-100,15,True),
             ("no_aw", [10,10,10,-10],1,1,0,1,-100,15,False),
             ("boundary",[5,-5],1,1,0,1,-10,10,True),
             ("lower",[-10,-10,10],1,1,0,1,-15,100,True),
             ("prev",[10,5,4],1,1,1,1,-100,15,True),
             ("ki_zero",[10,10,-10],1,0,1,1,-100,100,True)]
    print(f"#229 count={len(pos)} x(0)={pos[0][1]:.10f} x(3)={pos[-1][1]:.10f}")
    print(f"x(0.1)={pos[1][1]:.10f} x(0.2)={pos[2][1]:.10f}")
    for name,errs,kp,ki,kd,dt,lo,hi,on in cases:
        rows=aw_trace(errs,kp,ki,kd,dt,lo,hi,on)
        write_csv(name+".csv", "n,e,I_before,I_try,P,D,u_un,u,I",rows)
        print(name+" output="+",".join(f"{r[7]:.4f}" for r in rows))
    assert len(pos)==31 and abs(pos[-1][1]-1.0857775541183516)<1e-12
    assert pid_aw([10]*4,1,1,0,1,-100,100)==[20,30,40,50]
    assert pid_aw([10,10,10,-10],1,1,0,1,-100,15)==[15,15,15,-20]
    assert aw_trace([5],1,1,0,1,-10,10)[0][-1]==5
    assert aw_trace([-5],1,1,0,1,-10,10)[0][-1]==-5
    assert aw_trace([10,5,4],1,1,1,1,-100,15)[1][5]==-5
    assert all(r[5]==0 for r in pid_trace(2,0,.1,.1,30))
    assert all(r[-1]==0 for r in aw_trace([10,10,-10],1,0,1,1,-100,100))
    for bad_dt in [0,-1]:
        try:
            pid_control(2,.5,.1,bad_dt,30)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid dt accepted")
    print("checks=PASS")


if __name__ == "__main__":
    main()
