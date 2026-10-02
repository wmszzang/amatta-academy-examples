import numpy as np
def project(X, R, t, fx=500, fy=500, cx=320, cy=240):
    p = np.asarray(R) @ np.asarray(X,float) + np.asarray(t,float)
    if p[2] <= 0:
        raise ValueError('point must have positive camera depth')
    return np.array([fx*p[0]/p[2]+cx, fy*p[1]/p[2]+cy])
if __name__ == '__main__':
    print('(%.4f, %.4f)' % tuple(project([1,2,5],np.eye(3),[0.5,-0.5,1])))
