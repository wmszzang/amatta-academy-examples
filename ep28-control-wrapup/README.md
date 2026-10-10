# EP.28 — PID·칼만·상보 필터, 지문으로 고르는 법

네 자체 학습 문제의 입력·반환값·저장값을 구별하는 예제입니다. 실제 공개 기출이라고 주장하지 않습니다. C·C++·Python은 같은 계산을 수행하며, 내부 계산은 반올림하지 않고 CSV에서 소수점 9자리, 콘솔에서 6자리로 표시합니다.

| 문제 | 입력 | 산출물 | 다음 차례에 남길 값 |
|---|---|---|---|
| [#245 PID와 1차 플랜트](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=245) | 목표·이득·대상식·시간 간격 | 구간별 명령과 출력 200행 | 출력·누적 오차·직전 오차 |
| [#500 출력 제한과 적분 보류](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=500) | 오차 목록·상하한 | 명령 목록과 적분 장부 | 이득이 적용된 적분항·직전 오차 |
| [#399 일차원 칼만](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=399) | 이동량·위치 측정·분산 | 보정 위치·분산 | 보정 위치·분산 |
| [#228 상보 필터](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=228) | 각속도·변환된 측정각·고정 비중 | 보정각 20개 | 직전 보정각 |

## 계산과 기대값

### #245 — 구간 시작 명령, 구간 끝 출력

`Kp=2, Ki=1, Kd=0.1, gain=1, tau=1, target=1, dt=0.05, N=200`이며 초기 출력·누적 오차·직전 오차는 모두 0입니다.

```text
error = target - y
integral += error*dt
command = kp*error + ki*integral + kd*(error-prev)/dt
y_next = y + dt*(gain*command-y)/tau
```

첫 미분항은 `0.1*(1-0)/0.05=2`, 첫 명령은 `4.05`, 첫 출력은 `0.2025`입니다. 200회 뒤 출력은 `0.9939577634244283`이며 정확히 1은 아닙니다.

`plant.csv` 열은 `t_start, y_before, command, t_end, y_after`입니다. 원문 기준 답안의 `(k*dt, y_after, command)`는 이 예제의 `(t_start, y_after, command)`와 대응합니다. 첫 행의 명령은 0초에 계산하지만 새 출력의 시각은 **0.05초**입니다. 마지막 출력은 10초에 해당합니다. 초기 출력까지 그리면 201개 점입니다. 출력의 물리적 단위는 지문에서 지정하지 않았으므로 임의로 미터나 전압을 붙이지 않습니다.

### #500 — 적분 후보와 저장값

`kp=1, ki=1, kd=0, dt=1, lower=-100`이며 적분항은 0에서 시작합니다. `trial=integral+ki*error*dt`로 후보를 만들고 출력 범위를 적용한 뒤, **제한 전후 출력이 같을 때만** 후보를 저장합니다. 이 문제 고유 규칙이며 일반적인 모든 안티와인드업 알고리즘과 같다는 뜻은 아닙니다. 첫 미분항은 0이고, 직전 오차는 포화 여부와 관계없이 매번 갱신합니다.

| 조건 | 출력 | 마지막 저장 적분항 |
|---|---|---:|
| 오차 `[10,10,10,10]`, 상한 100 | `[20,30,40,50]` | 40 |
| 오차 `[10,10,10,-10]`, 상한 15, 보류 | `[15,15,15,-20]` | -10 |
| 같은 입력, 보류 없음 비교 | `[15,15,15,10]` | 20 |
| 초기 상태, 오차 7.5, 상한 15 | `[15]` | 7.5 |

마지막 행은 원문 규칙으로 추가한 경계 확인입니다. 후보 출력이 경계와 같으면 저장합니다. `normal.csv`, `hold.csv`, `no_hold.csv`, `boundary.csv`에서 전체 장부를 볼 수 있습니다.

### #399 — 원문 예시 수치 불일치

초기 `x=0, P=1`, 이동량 `[1,1]`, 측정 `[1.2,2.1]`, `Q=0.1, R=0.5`입니다.

```text
x_pred=x+move
P_pred=P+Q
K=P_pred/(P_pred+R)
x=x_pred+K*(measured-x_pred)
P=(1-K)*P_pred
```

| 차례 | 보정 위치 | 보정 분산 | 소수점 4자리 표시 |
|---|---:|---:|---|
| 1 | 1.1375 | 0.34375 | `(1.1375, 0.3438)` |
| 2 | 3201/1510 | 71/302 | `(2.1199, 0.2351)` |

현재 원문 예시의 `(1.1818, 0.3636)`, `(2.1186, 0.3253)`은 지문의 계산식 및 기준 코드와 맞지 않습니다. 본 예제는 계산식에서 얻은 값을 사용합니다. **운영 문제나 채점 기준을 수정했다는 뜻은 아닙니다.** P·Q·R은 거리나 최대 오차가 아닌 분산이며, 서로 얽히지 않는 평균 0의 오차 모델을 전제합니다. K는 정답 확률이 아닙니다.

### #228 — 각속도에 시간을 곱하기

초기 각도 0, 자이로 `12 deg/s`, `dt=0.1 s`, `alpha=0.95`입니다. 차례 k의 측정각은 `10*(k*dt)`도이며 이미 변환된 각도입니다.

`angle=0.95*(angle+12*0.1)+0.05*measured_angle`

첫 결과는 1.19도, 둘째는 2.3705도, 20번째는 22.437753494847524도입니다. 2초의 자이로 누적 24도와 예제 참값 20도 사이이며, 오차 약 2.437753도는 남습니다. 실제 가속도계 각도가 항상 참값이라는 가정은 하지 않습니다. 모든 20개 값은 `complementary.csv`에 기록됩니다. 독립 대조식은 `k+3.8*(1-0.95**k)`입니다.

## 설치와 실행

Python 3, GCC, G++가 필요합니다. 그림 생성에만 matplotlib을 설치합니다. 각 언어 폴더에서 실행하면 같은 이름의 CSV 7개가 생성됩니다.

```powershell
cd python
python control_wrapup.py
python -m pip install matplotlib
python visualize.py
python visualize.py --frames
```

```powershell
cd c
gcc control_wrapup.c -o control_wrapup.exe -lm
./control_wrapup.exe
```

```powershell
cd cpp
g++ -std=c++11 control_wrapup.cpp -o control_wrapup.exe
./control_wrapup.exe
g++ -std=c++11 plot_svg.cpp -o plot_svg.exe
./plot_svg.exe
```

Windows에서 컴파일러가 PATH에 없으면 설치된 MinGW의 bin 경로를 PATH 앞에 추가합니다. 이 제작 환경에서는 CLion 2026.2.0.1 번들의 MinGW를 사용했습니다. `plot_svg.cpp`는 C++ 계산이 만든 CSV를 읽어 `control-wrapup.svg`를 작성합니다. `visualize.py --frames`는 12fps 재생용 48프레임을 생성합니다. PNG·CSV·SVG·실행파일·프레임은 실행 산출물로 커밋하지 않습니다.

## 확인한 동작과 실험

세 언어를 실제 실행해 CSV 7개의 헤더와 전체 행을 대조했습니다. Python 검증은 첫 미분항·처음과 마지막 출력·200행의 시간 규약·정상/포화/보류 없음·상한과 같은 경계·포화 중 이전 오차 갱신·칼만 분수값·상보 20개 닫힌 식·비중 0과 1을 확인합니다.

- `limited_pid(..., hold=False)`로 마지막 출력 부호가 바뀌는 이유를 장부에서 확인합니다.
- `complementary(alpha=0)`은 측정각만, `alpha=1`은 자이로 예측만 남습니다.
- `plant_pid`의 시정수를 바꾸고 같은 명령에서 변화율이 달라지는지 비교합니다. 이득 변경에 따른 모든 안정성을 보장하는 예제는 아닙니다.

흔한 실점은 누적값을 반복문 안에서 초기화하기, #245의 첫 미분항을 0으로 바꾸기, #500의 적분 이득을 두 번 곱하기, 포화 중 직전 오차 갱신을 빼먹기, #399의 잘못된 예시 수치에 식을 맞추기, #228에서 각속도를 각도에 그대로 더하기입니다.

[강의 페이지](https://www.techrraforming.com/academy/cert-prac-control-wrapup) · [실기 연습장](https://www.techrraforming.com/practical?cert=RobotSoftware)
