# EP.20 · 바퀴 로봇 기구학

차동구동·메카넘·아커만·바이시클 모델과 바퀴 오도메트리를 C, C++, Python으로 계산합니다. 모든 위치는 미터, 내부 각도는 라디안입니다. #481 반환만 도 단위이며 반올림은 출력에서만 합니다.

## 문제별 입력과 기대 출력

| 문제 | 입력 | 기대 결과 |
|---|---|---|
| #261 | v=0.5, w=0.4, r=0.05, L=0.3 | 좌우 선속도 0.4400, 0.5600; 각속도 8.8000, 11.2000 |
| #483 | vx=1, vy=0.5, w=0.2, r=0.05, lx=0.3, ly=0.25 | FL, FR, RL, RR = 7.8000, 32.2000, 27.8000, 12.2000 rad/s |
| #481 | L=2.5, T=1.5, R=10 | 내측 15.1240°, 외측 13.0919° |
| #463 | 초기 (0,0,0), L=2, v=1, delta=0.1, dt=0.1, N=10 | (0.9996, 0.0226, 0.0502) |
| #224 | dL=0.1, dR=0.12, L=0.5, N=40 | 끝점 (2.7490, 2.8305), 방향 1.6000; 시작점 포함 41개 좌표 |
| #230 | r=0.05, CPR=2048, dt=0.1, 구간 ticks=[0,100,205,300,412] | 틱당 0.000153398…m; 마지막 누적거리 0.1560m, 속도 0.6320m/s |
| #254 | gy=1, Ld=5, L=2.5 | 곡률 0.0800, 대표 조향각 0.1974rad; 보조 반경 12.5000m |

메카넘 좌표는 차체 앞 +x, 왼쪽 +y, 반시계 회전 +w입니다. 바퀴 순서는 앞왼·앞오른·뒤왼·뒤오른입니다. 반지름과 축 간격은 양수, 아커만 예제는 R>T/2인 좌선회입니다. 차동구동의 L은 좌우 간격이고 조향형의 L은 앞뒤 축거입니다.

## 설치와 실행

Python 3를 설치하고 이 폴더에서 실행합니다. 계산 파일은 표준 라이브러리만 사용합니다. 시각화는 matplotlib을 설치합니다.

```powershell
python python/mobile_kin.py
python -m pip install matplotlib
python python/visualize.py --frames
```

`trajectory-python.csv`에 시작점을 포함한 41개 좌표를 저장합니다. 시각화는 `output/`에 정적 궤적·슬립 비교·엔코더 그래프와 40개의 진행 프레임을 만듭니다.

C/C++는 GCC·G++가 포함된 MinGW 또는 동등한 컴파일러를 설치하고 PATH에 추가합니다. CLion 번들 MinGW에서도 다음 명령을 사용할 수 있습니다.

```powershell
gcc -std=c11 c/mobile_kin.c -o mobile-c.exe -lm
./mobile-c.exe
g++ -std=c++11 cpp/mobile_kin.cpp -o mobile-cpp.exe
./mobile-cpp.exe
g++ -std=c++11 cpp/plot_svg.cpp -o plot-svg.exe
./plot-svg.exe
```

각 언어의 콘솔 수치와 `trajectory-c.csv`, `trajectory-cpp.csv`, `trajectory-python.csv`가 같습니다. C는 포인터 출력, C++는 바이시클 자세에 `struct Pose`를 사용합니다. `plot_svg.cpp`는 외부 그래픽 라이브러리 없이 동일 축척의 `trajectory.svg`를 생성합니다. CSV를 Excel의 XY 산점도로 그려도 같은 궤적입니다.

## 계산 순서와 흔한 실점

- 차동구동: 목표 전진 속도에 회전 몫 `w*L/2`를 빼고 더합니다. 반지름으로 나누면 바퀴 각속도입니다.
- 메카넘: `k=lx+ly`는 합입니다. 순방향 함수로 세 목표 속도를 복원해 부호·순서를 검산합니다.
- 아커만: 뒷축 중앙의 반경 R에서 윤거 절반을 빼거나 더한 길이로 두 조향각을 계산합니다.
- 바이시클: 위치와 방향 세 값을 모두 이전 방향각으로 계산합니다. 입력 delta는 이미 라디안입니다.
- 오도메트리: `theta+dtheta/2`인 중점각으로 이동하고, 시작점도 제출 목록에 넣습니다. 구간 거리 입력에 dt를 다시 곱하지 않습니다.
- 엔코더: ticks는 누적 계수기 값이 아닌 각 구간의 증가량입니다.
- Pure Pursuit: 반환 계약은 곡률과 조향각입니다. 직진에서는 곡률이 0이므로 반경 역수를 계산하지 않습니다. Ld는 양수여야 합니다.

## 직접 바꿔 볼 실험

1. `odometry(.1, .12*.97, .5, 40)`으로 오른바퀴에 3% 슬립을 가정합니다. 끝점은 (3.1891, 2.4547), 무슬립 추정 위치와 거리는 0.5787m입니다. 실측 자료가 아닌 가정 계산입니다.
2. Python `odometry(..., midpoint=False)`로 구간 시작각을 사용합니다. 원호 정확해와 끝점 거리가 0.0789m로 증가합니다.
3. `pure_pursuit(1, Ld, 2.5)`의 Ld를 5보다 크게 바꿉니다. 같은 옆거리에서 곡률과 조향각이 작아지는지 확인합니다.

계산 코드 내부 검산은 정상 수치와 직진·시작점 조건을 확인합니다. 세 언어의 전체 콘솔 출력과 좌표 41개를 실제 빌드·실행하여 대조했습니다.

## 관련 링크

- [YouTube 공개 강의](https://youtu.be/o_wsRncPKvI)

- [강의](https://www.techrraforming.com/academy/cert-prac-mobile-kinematics)
- [#261 차동구동](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=261)
- [#483 메카넘](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=483)
- [#481 아커만](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=481)
- [#463 바이시클](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=463)
- [#224 오도메트리](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=224)
- [#230 엔코더](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=230)
- [#254 Pure Pursuit](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=254)
