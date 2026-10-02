import numpy as np
def triangulate(uL, vL, uR, fx, B, cx, cy):
    # 정렬된 영상, 공통 주점과 fx=fy를 가정한다.
    d = uL-uR
    if d <= 0 or fx <= 0 or B <= 0:
        raise ValueError('positive disparity, focal length and baseline required')
    Z = fx*B/d
    return np.array([(uL-cx)*Z/fx,(vL-cy)*Z/fx,Z])
if __name__ == '__main__':
    print('(%.4f, %.4f, %.4f)' % tuple(triangulate(370,240,345,500,0.1,320,240)))
