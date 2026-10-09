# EP.27 — 1차원 칼만 필터와 상보 필터

위치 하나의 예측·보정과 각도 결합을 C · C++ · Python으로 같은 순서로 계산합니다.

|예제|입력|기대 결과|
|---|---|---|
|정지 칼만 #232|x=0, P=1, Q=0.01, R=1; 측정 15개|x=4.826007, P=0.103646|
|이동 칼만 #399|u=[1,1], z=[1.2,2.1], Q=0.1, R=0.5|첫 x=1.137500, P=0.343750; 둘째 x=2.119868, P=0.235099|
|상보 필터 #228|각속도 12 deg/s, dt=0.1 s, alpha=0.95, 측정각도=10t|20회 후 22.437753 deg|

정지 측정값: `[4.2,6.1,4.8,5.5,4.3,5.9,5.1,4.7,5.3,4.9,5.2,4.6,5.4,5.0,4.8]`.
위치 단위는 문제에서 지정하지 않습니다. 참값 5는 그래프 비교용이며 필터 입력이 아닙니다.

계산 순서: `x_pred=x+u`, `P_pred=P+Q`, `K=P_pred/(P_pred+R)`, `x=x_pred+K*(z-x_pred)`, `P=(1-K)*P_pred`.
상보 계산: `theta=0.95*(theta+12*0.1)+0.05*accel_angle`.
반올림은 출력할 때만 하고 내부 상태는 그대로 다음 행에 넘깁니다.

## Python

Python 3를 설치하고 터미널에서 다음을 실행합니다. 계산에는 표준 라이브러리만 필요합니다.

```powershell
cd python
python filters.py
python -m pip install matplotlib
python visualize.py
python visualize.py --frames
```

## C

MinGW-w64의 gcc를 PATH에 등록하거나 Visual Studio 개발자 명령 프롬프트를 엽니다.

```powershell
cd c
gcc filters.c -o filters.exe -lm
./filters.exe
```

Visual Studio에서는 `cl /utf-8 filters.c` 후 `filters.exe`를 실행합니다.

## C++

```powershell
cd cpp
g++ -std=c++11 filters.cpp -o filters.exe
./filters.exe
g++ -std=c++11 plot_svg.cpp -o plot_svg.exe
./plot_svg.exe
```

계산 프로그램은 실행 폴더에 `stationary.csv`(15행), `moving.csv`(2행), `complementary.csv`(20행)를 만듭니다.
세 언어 출력은 소수점 아래 여섯 자리에서 같습니다. `filters.svg`는 브라우저로 열 수 있습니다.
Python 시각화는 초기값 0을 포함한 `stationary.png`, 이동 예제의 `moving.png`, 각도 예제의 `complementary.png`를 만듭니다. `--frames`를 지정하면 `frames/frame_00.png`부터 정지 예제 프레임 16장을 추가합니다.

## 실험과 흔한 실점

- R을 키워 측정을 덜 반영하는지, Q를 키워 더 반영하는지 같은 입력으로 비교하세요.
- P=Q=0, R>0이면 K=0이라 초기 위치가 바뀌지 않습니다. 분산에 음수를 넣지 않습니다.
- 큰 잔차가 자동으로 이득을 낮추지 않습니다. 이 예제의 K에는 측정값 자체가 없습니다.
- 위치와 분산을 매번 초기화하지 말고, 분산 보정에는 P_pred를 사용하세요.
- 상보 필터는 각속도에 dt를 곱합니다. 가속도계 입력은 이미 각도로 변환된 값입니다.
- alpha=1-K는 결합식의 비중을 비교한 대응이며 두 필터가 일반적으로 같다는 뜻이 아닙니다.

## 원문 수치 정정

#399 보관 출제 스크립트의 예시 답 `(1.1818,0.3636)`, `(2.1186,0.3253)`은 같은 파일의 식과 맞지 않습니다.
이 예제는 입력과 식을 다시 계산한 위 표를 사용합니다. 운영 채점 기준을 수정했다는 뜻은 아닙니다.

## 관련 링크

- 강의: https://www.techrraforming.com/academy/cert-prac-kalman-1d
- #232: https://www.techrraforming.com/practical?cert=RobotSoftware&problem=232
- #228: https://www.techrraforming.com/practical?cert=RobotSoftware&problem=228
- #399: https://www.techrraforming.com/practical?cert=RobotSoftware&problem=399
