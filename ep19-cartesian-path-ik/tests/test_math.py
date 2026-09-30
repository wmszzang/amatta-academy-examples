"""수치 기준과 실패 경계 검증. 장비 제어를 실행하지 않는다."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from kin_path import *


class PathChecks(unittest.TestCase):
    def test_reference_inverse_and_roundtrip(self):
        for p,expected in [(P0,(-22.2891,79.3281)),(P1,(34.8800,80.4059))]:
            q=ik2(*p)
            self.assertLess(math.dist(fk2(*q),p),1e-12)
            for a,b in zip(map(math.degrees,q),expected):self.assertAlmostEqual(a,b,places=4)
        q=ik2(1.5,1.,1.,1.)
        self.assertAlmostEqual(math.degrees(q[0]),8.031161,places=5)

    def test_unreachable_and_atomic_commands(self):
        rows=path_ik(line_points((.35,0),(-.35,0),10))
        self.assertEqual(sum(r[-1]=='UNREACHABLE' for r in rows),9)
        with self.assertRaises(ValueError):joint_commands(rows)
        self.assertIsNone(ik2(2.101,0))
        self.assertIsNone(ik2(.299,0))
        self.assertIsNotNone(ik2(2.1,0))
        self.assertIsNotNone(ik2(.3,0))

    def test_endpoint_sampling(self):
        for dt in (.1,.3,3.):
            ts=sample_times(2.,dt)
            self.assertEqual(ts[0],0)
            self.assertEqual(ts[-1],2.)
            self.assertEqual(ts.count(2.),1)
            self.assertTrue(all(a<b for a,b in zip(ts,ts[1:])))
        self.assertEqual(len(sample_times(2.,.1)),21)

    def test_profiles(self):
        L=math.dist(P0,P1)
        self.assertAlmostEqual(trap_s(.1,L,2,.5),.0104136662,places=9)
        q0,q1=ik2(*P0),ik2(*P1)
        for degree in (3,5):
            rows=joint_cubic(q0[0],q1[0],2,20,degree)
            self.assertEqual(rows[0][2],0)
            self.assertEqual(rows[-1][2],0)
            if degree==5:
                self.assertEqual(rows[0][3],0)
                self.assertEqual(rows[-1][3],0)
        p=fk2(*[(a+b)/2 for a,b in zip(q0,q1)])
        self.assertAlmostEqual(deviation(p),.2004845,places=6)


if __name__=='__main__':unittest.main()
