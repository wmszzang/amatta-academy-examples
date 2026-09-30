# EP.19 관절공간 vs 직교공간 경로

같은 두 점을 연결해도 관절각을 보간하면 손끝이 직선에서 **0.2005 m** 벗어납니다. 손끝 좌표를 먼저 정하는 방법, 각 점의 역기구학, 원호와 관절 스윕, 시간표 결합을 같은 팔로 대조합니다.

| 문제 | 주어진 것 | 판별 신호어 | 기대 결과 |
|---|---|---|---|
| #240 원호 보간 | 중심·반지름·시작각·끝각·구간 수 | 원호 좌표열 | 7점, 호길이 1.4137 m |
| #223 관절 스윕 | 첫 관절 30° 고정, 둘째 0°~90° | 한 관절만 회전 | 같은 원호 7점, 차이 < 1e-12 m |
| #222 역기구학 | 링크 각 1 m, 목표 (1.5,1.0) m | 좌표에서 각도로 | (8.0312°,51.3178°) |
| #225 3차 궤적 | 양끝 각도, T=2 s, 양끝 속도 0 | 부드러운 시간표 | 각도·각속도·각가속도 CSV |

일반 비교는 링크 **1.2 m·0.9 m**, 시작 `(1.6,0.3)`, 끝 `(0.6,1.5)`입니다. #222 원문 확인만 링크 각각 1 m입니다. 내부 각도는 rad, 표시 각도는 deg입니다. 원호 중심은 `(1.2*cos(30°),1.2*sin(30°))`를 반올림 없이 사용합니다.

| 항목 | 관절공간 | 직교공간 |
|---|---|---|
| 계산 | 시작각을 알면 좌표 목표 IK 1회 | 이 예제는 양끝 포함 21점 IK |
| 형상 | 끝각 보간만으로 직선 보장 안 됨 | 좌표 경로를 직접 지정 |
| 특이점 | 자동 회피 아님 | 방향별 요구 속도 확인 |
| 관절한계·연속성 | 독립 범위 안 단조 보간은 범위 유지에 유리 | 매 점의 해·한계·해 연결 확인 |

호출 횟수 1:21은 실행시간 배율이 아닙니다. 시작점 해를 재사용하면 직교 방식은 20회입니다. 충돌·실제 관절한계·도구 방향은 이 평면 위치 모델에 포함되지 않습니다.

```text
시간 t -> 이동거리 s(t) -> 진행률 u=s/L -> 손끝 점 -> IK
                                                     |
                         전체 진단 -> 실패 여부 -----+
                            | 실패          | 전부 통과
                         상태 기록         각도 명령 목록
                         명령 생성 중단
```

## 실행

아래 명령은 이 폴더에서 실행합니다. Python 3.10 이상, C 컴파일러, C++11 컴파일러를 사용합니다. Windows에서는 gcc/g++가 있는 MinGW `bin`을 PATH에 추가합니다. C++ 계산 프로그램은 같은 `struct Pt` 커널을 C 소스에서 포함하여 수학 코드의 중복을 피합니다.

```powershell
python -m pip install numpy matplotlib
python python/kin_path.py
python python/visualize.py
python python/visualize.py --frames
python -m unittest discover -s tests

gcc c/kin_path.c -o native-c.exe -lm
./native-c.exe
g++ -std=c++11 cpp/kin_path.cpp -o native-cpp.exe
./native-cpp.exe
g++ -std=c++11 cpp/plot_svg.cpp -o plot-svg.exe
./plot-svg.exe compare.csv
```

`python/p1_compare.py`부터 `p4_cubic.py`까지는 각 문제의 CSV와 PNG를 함께 생성합니다. 통합 Python 실행 후 `visualize.py`는 `result.png`를, C/C++ 통합 실행은 `trajectory.svg`를 생성합니다. `plot-svg.exe`는 실제 `compare.csv`를 읽어 `csv-trajectory.svg`를 생성합니다. `--frames`는 같은 축척의 2×2 그림 180장(15fps, 12초)을 만듭니다.

| 산출물 | 행 수 | 열·단위 |
|---|---:|---|
| compare.csv | 11 | 진행률, 관절 경로 각도·좌표·수직 이탈, 직교 경로 좌표·각도 |
| arc.csv / sweep.csv | 각 7 | x,y (m) |
| path.csv | 21 | t(s),s(m),x,y,th1,th2(deg),dth1,dth2(deg/s),w(m²) |
| cubic.csv | 21 | t, 두 관절 각각 각도·각속도·각가속도 |
| dev.csv | 21 | t,x,y,직선 수직 이탈 |
| unreachable.csv | 11 | 좌표, rad 각도 또는 빈 값, 상태 |

평균 각속도는 반올림 전 값과 실제 시간 차이로 구합니다. 첫 행은 이전 구간이 없어 속도를 비웁니다. `tests/expected`의 4종 CSV가 고정 비교 기준입니다.

## 검산·실험

- 관절 경로 길이 1.6298 m는 10,001점의 고밀도 근삿값입니다. 11점을 직선으로 연결한 길이와 혼동하지 않습니다.
- 직선 길이 1.5620 m, 가운데 각도 `(0.2660°,96.1125°)`. 관절각의 변화량은 일정하지 않습니다.
- 사다리꼴 `T=2, ta=0.5, dt=0.1`: 최고속도 1.0414 m/s, 가속도 2.0827 m/s², `s(0.1)=0.0104 m`.
- 구간 평균 관절속도 최댓값 47.092·28.896 deg/s. 조작성 지수 표본 범위 1.0613~1.0799 m². 연속시간 최고속도나 장비 안전 보증이 아닙니다.
- 3차·5차의 첫 관절 최고속도는 42.877·53.596 deg/s입니다. 두 관절에 같은 단조 진행률을 적용해야 경로가 유지됩니다.
- `(.35,0)`에서 `(-.35,0)`으로 직선을 만들면 11점 중 9점이 도달 불가입니다. 진단은 모두 기록하고, 명령 생성은 전체 목록을 거절합니다. 실패점 삭제 후 연결하지 않습니다.
- `sample_times(2,.3)`처럼 나누어떨어지지 않는 시간 간격도 마지막 시각을 한 번만 포함하는지 실험합니다.
- 원호 시작은 완전히 펴진 특이 자세입니다. 접선 방향 운동까지 전부 불가능한 것은 아닙니다.

## 제출 확인

1. 좌표 m와 각도 deg/rad를 구분한다.
2. 시간 간격 dt와 공간 구간 수 N을 구분한다.
3. 동일한 팔꿈치 굽힘 방향과 해의 연결을 확인한다.
4. 양끝을 포함하고 실패 상태를 기록한다.
5. 좌표·각도 소수 4자리 CSV와 요구 그래프를 제출한다.

## 관련 링크

- [강의](https://www.techrraforming.com/academy/cert-prac-cartesian-path-ik)
- [#240 원호 보간](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=240)
- [#223 관절 스윕](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=223)
- [#222 역기구학](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=222)
- [#225 3차 관절 궤적](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=225)
