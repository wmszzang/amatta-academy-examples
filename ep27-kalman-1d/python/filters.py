import csv
from pathlib import Path

def kalman(measurements, moves, q, r, x=0.0, p=1.0):
    if len(measurements) != len(moves) or p < 0 or q < 0 or r <= 0:
        raise ValueError("Invalid inputs")
    rows = []
    for k, (u, z) in enumerate(zip(moves, measurements), 1):
        xp, pp = x + u, p + q
        gain = pp / (pp + r)
        residual = z - xp
        x = xp + gain * residual
        p = (1 - gain) * pp
        rows.append((k, u, z, xp, pp, gain, residual, x, p))
    return rows

def complementary():
    theta = 0.0
    rows = []
    for k in range(1, 21):
        t = k * 0.1
        angle = 10 * t
        predicted = theta + 12 * 0.1
        theta = 0.95 * predicted + 0.05 * angle
        rows.append((k, t, 12 * t, angle, predicted, theta))
    return rows

def datasets():
    z = [4.2,6.1,4.8,5.5,4.3,5.9,5.1,4.7,5.3,4.9,5.2,4.6,5.4,5.0,4.8]
    return {'stationary':kalman(z,[0]*15,0.01,1),
            'moving':kalman([1.2,2.1],[1,1],0.1,0.5),
            'complementary':complementary()}

def main():
    for name, rows in datasets().items():
        header = 'k,t,gyro,accel,predicted,theta' if name == 'complementary' else 'k,u,z,x_pred,P_pred,K,residual,x,P'
        lines = [header] + [str(row[0])+','+','.join(f'{v:.6f}' for v in row[1:]) for row in rows]
        Path(name+'.csv').write_text('\n'.join(lines)+'\n', encoding='utf-8')
        print(name+': '+lines[-1])

if __name__ == '__main__': main()
