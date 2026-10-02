import math
import numpy as np
from frame_chain import *
from icp_step import Q,P,estimate_rigid_2d
from pinhole import project
from stereo import triangulate
from calib_offset import example,rms
p=apply_T(T_BASE_CAM,[0.2,0,2]); np.testing.assert_allclose(p,[2.3,-0.2,0.4])
np.testing.assert_allclose(apply_T(T_BASE_ARM,apply_T(T_ARM_CAM,[0.2,0,2])),p)
np.testing.assert_allclose(apply_T(inv_T(T_BASE_CAM),p),[0.2,0,2],atol=1e-12)
np.testing.assert_allclose(R_BC.T@R_BC,np.eye(3)); assert abs(np.linalg.det(R_BC)-1)<1e-12
R,t=estimate_rigid_2d(Q,P); np.testing.assert_allclose(Q@R.T+t,P,atol=1e-12)
assert abs(math.degrees(math.atan2(R[1,0],R[0,0]))-90)<1e-10
np.testing.assert_allclose(t,[2,3])
np.testing.assert_allclose(project([1,2,5],np.eye(3),[0.5,-0.5,1]),[445,365])
np.testing.assert_allclose(triangulate(370,240,345,500,.1,320,240),[.2,0,2])
_,_,delta,before,after=example()
np.testing.assert_allclose(np.degrees(delta),[.9961989733,-.4907275269],atol=1e-9)
np.testing.assert_allclose([rms(before)*1000,rms(after)*1000],[11.63001278,.04132929],atol=1e-8)
for right in (370,371):
    try: triangulate(370,240,right,500,.1,320,240)
    except ValueError: pass
    else: raise AssertionError('invalid disparity accepted')
try: project([1,2,0],np.eye(3),[0,0,0])
except ValueError: pass
else: raise AssertionError('zero depth accepted')
# 반사 자료에서도 회전행렬의 방향 보존 조건은 유지해야 한다.
R,_=estimate_rigid_2d(Q,Q*np.array([-1,1])); assert abs(np.linalg.det(R)-1)<1e-12
print('PASS: chain, inverse, SVD, projection, stereo, calibration, invalid inputs, reflection')
