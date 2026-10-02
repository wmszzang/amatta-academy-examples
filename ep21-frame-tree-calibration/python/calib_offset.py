import numpy as np
L1 = L2 = 0.5
def fk(q):
    return np.array([L1*np.cos(q[0])+L2*np.cos(q[0]+q[1]),L1*np.sin(q[0])+L2*np.sin(q[0]+q[1])])
def J(q):
    s1,c1 = np.sin(q[0]),np.cos(q[0]); s12,c12 = np.sin(q.sum()),np.cos(q.sum())
    return np.array([[-L1*s1-L2*s12,-L2*s12],[L1*c1+L2*c12,L2*c12]])
def calibrate(poses, measured):
    A=np.vstack([J(q) for q in poses])
    b=np.concatenate([m-fk(q) for q,m in zip(poses,measured)])
    return np.linalg.lstsq(A,b,rcond=None)[0]
def example():
    poses=np.radians([[30,45],[0,90],[60,-30]])
    measured=np.array([fk(q+np.radians([1,-0.5])) for q in poses])
    delta=calibrate(poses,measured)
    before=measured-np.array([fk(q) for q in poses])
    after=measured-np.array([fk(q+delta) for q in poses])
    return poses,measured,delta,before,after
def rms(residual):
    return np.sqrt(np.mean(np.sum(residual**2,axis=1)))
if __name__ == '__main__':
    _,_,delta,before,after=example()
    print('offset = (%.4f, %.4f) deg' % tuple(np.degrees(delta)))
    print('position RMS = %.4f -> %.4f mm' % (rms(before)*1000,rms(after)*1000))
