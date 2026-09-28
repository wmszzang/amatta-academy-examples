import sys
from quat import qmul,qnorm,fmt
from slerp import slerp
from distance import geodesic_deg

data = list(map(float,sys.stdin.read().split()))
if len(data) != 8:
    raise SystemExit('입력: 쿼터니언 두 개(w x y z)' + "")
a,b = tuple(data[:4]),tuple(data[4:8])
print(fmt(qmul(a,b)))
