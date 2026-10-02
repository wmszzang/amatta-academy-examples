import numpy as np
def make_T(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T
def apply_T(T, p):
    return T[:3, :3] @ np.asarray(p, float) + T[:3, 3]
def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)
R_BC = np.array([[0,0,1],[-1,0,0],[0,-1,0]], float)
# 중간 프레임을 거치는 두 변환도 같은 최종 변환을 만든다.
T_BASE_ARM = make_T(np.eye(3), [0.1,0,0.2])
T_ARM_CAM = make_T(R_BC, [0.2,0,0.2])
T_BASE_CAM = T_BASE_ARM @ T_ARM_CAM
if __name__ == '__main__':
    p = apply_T(T_BASE_CAM, [0.2,0,2])
    print('p_base = (%.4f, %.4f, %.4f)' % tuple(p))
    print('back = (%.4f, %.4f, %.4f)' % tuple(apply_T(inv_T(T_BASE_CAM), p)))
