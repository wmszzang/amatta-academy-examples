import math
import numpy as np
Q = np.array([[0,0],[1,0],[1,1],[0,1]], float)
P = np.array([[2,3],[2,4],[1,4],[1,3]], float)
def estimate_rigid_2d(Q, P):
    Q, P = np.asarray(Q,float), np.asarray(P,float)
    if Q.shape != P.shape or Q.ndim != 2 or Q.shape[1] != 2 or len(Q) < 2:
        raise ValueError('paired 2D points required')
    cQ, cP = Q.mean(0), P.mean(0)
    H = (Q-cQ).T @ (P-cP)
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T.copy()
    if np.linalg.det(V @ U.T) < 0:
        V[:, -1] *= -1
    R = V @ U.T
    t = cP - R @ cQ
    return R, t
if __name__ == '__main__':
    R, t = estimate_rigid_2d(Q,P)
    print('angle=%.4f deg, t=(%.4f, %.4f)' % (math.degrees(math.atan2(R[1,0],R[0,0])),*t))
