"""세 언어의 제출용 CSV와 정합 텍스트를 대조한다."""
from pathlib import Path
import csv
root=Path(__file__).resolve().parents[1]/'out'
names=['wheels.csv','odom.csv','steer.csv','arc_path.csv','jointlerp_gap.csv','camera.csv','rejudge.csv','calib.txt']
for name in names:
    expected=(root/'python'/name).read_text(encoding='utf-8').splitlines()
    for lang in ['c','cpp']:
        actual=(root/lang/name).read_text(encoding='utf-8').splitlines()
        assert actual==expected,(lang,name,[(a,b) for a,b in zip(expected,actual) if a!=b])
print('PASS: 8 output files exactly identical across Python, C, C++')
