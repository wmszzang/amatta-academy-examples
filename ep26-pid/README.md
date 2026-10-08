# EP.26 — PID 제어와 안티와인드업

[강의 페이지](https://www.techrraforming.com/academy/cert-prac-pid)

사이트 학습 문제 #229와 #500의 실행 가능한 예제입니다. 실제 기출문제라고 주장하지 않습니다. 제어입력은 #229의 단순 위치 모형에서 위치 변화율이며 실제 모터의 전압이나 힘이 아닙니다.

## 입력과 기대 결과

| 문제 | 입력 | 기대 결과 |
|---|---|---|
| #229 이산 PID 위치 제어 응답 | Kp=2, Ki=0.5, Kd=0.1, dt=0.1, N=30, r=1 | 위치 31개, x(0)=0, x(0.1)=0.205, x(0.2)=0.352475, x(3)=1.0857775541183516 |
| #500 예제 A | Kp=Ki=1, Kd=0, dt=1, errs=[10,10,10,10], 범위 [-100,100] | [20,30,40,50] |
| #500 예제 B | 같은 이득, errs=[10,10,10,-10], 범위 [-100,15] | [15,15,15,-20] |
| 적분 보류 없는 비교 | 예제 B와 동일 | [15,15,15,10] |

#229는 x=0, 누적값=0, 이전 오차=r로 시작하므로 첫 미분항은 0입니다. 오차 → 누적값 → 변화율 → 제어입력 계산 후 **갱신 전 위치를 기록**하고 x += u*dt, 이전 오차 갱신 순으로 진행합니다. 기준 답안의 N+1 반복과 맞췄으며 마지막 기록 이후 갱신된 위치는 반환하지 않습니다. 그래프의 3초 끝점은 정착 완료를 뜻하지 않습니다.

#500은 이미 Ki를 곱한 적분항 I를 저장합니다. I_try=I+Ki*e*dt로 후보를 만들고 후보로 계산한 출력을 제한합니다. **제한 전후 출력이 같을 때만 후보를 저장**합니다. 제한됐다면 기존 I를 유지하지만 이번 출력은 그대로 반환합니다. 이전 오차는 매번 갱신하며 첫 D는 0입니다. 경계와 정확히 같은 출력은 저장합니다. 일반적인 조건부 적분(포화에서 빠져나오는 방향 허용)으로 문제 규칙을 바꾸지 않았습니다.

내부 계산은 반올림하지 않습니다. #229의 J는 이득 전 누적값이고 #500의 I는 이득을 적용한 값입니다. 두 곳에서 Ki를 중복 적용하지 마세요.

## 설치·실행

Python 3와 GCC/G++ 또는 동등한 C/C++ 컴파일러를 설치합니다. Windows에서는 CLion 번들 MinGW의 bin을 PATH에 추가할 수 있습니다. 각 언어 폴더를 작업 디렉터리로 삼으세요.

```powershell
cd ep26-pid/python
python pid.py
python -m pip install numpy matplotlib
python visualize.py
python visualize.py --frames
```

```powershell
cd ep26-pid/c
gcc pid.c -o pid.exe -lm
./pid.exe
```

```powershell
cd ep26-pid/cpp
g++ -std=c++11 pid.cpp -o pid.exe
./pid.exe
g++ -std=c++11 plot_svg.cpp -o plot_svg.exe
./plot_svg.exe
```

각 계산기는 position.csv와 A.csv, B.csv, no_aw.csv, boundary.csv, lower.csv, prev.csv, ki_zero.csv를 만듭니다. Python 시각화는 pid.png와 선택적 frames/를, C++ 시각화는 pid.svg를 만듭니다. 실행 파일·CSV·그림·프레임은 버전 관리에서 제외합니다.

세 언어의 표준 출력과 CSV 8개의 문자열이 동일함을 실제 실행으로 확인했습니다. 표시된 음의 0은 출력 단계에서만 0으로 정규화합니다. 첫 미분항 0, 상·하한 제한, 경계에서 적분 확정, 포화 중 이전 오차 갱신, Ki=0, 양수가 아닌 dt 거부를 검증했습니다.

## 실험과 흔한 실점

- Ki=0으로 바꿔 적분의 출력 기여가 없어지는지 확인하세요.
- 예제 B에서 보류를 끄면 마지막 출력이 -20에서 +10으로 달라집니다. 출력만 보지 말고 저장된 I를 비교하세요.
- 위치를 갱신한 뒤 기록하면 첫 값 0이 빠집니다.
- 적분 후보를 포화 중에도 저장하거나, 보류 뒤 출력을 다시 계산하면 #500 규칙과 달라집니다.
- dt를 누적식에서 빼거나 미분식에서 나누지 않으면 이득의 의미가 달라집니다.
- #500의 입력은 주어진 오차 목록입니다. 이 자료만으로 실제 로봇의 위치나 정착 시간을 추론하지 마세요.

## 관련 링크

- [#229 이산 PID 위치 제어 응답](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=229)
- [#500 PID 안티와인드업 — 적분 포화 클램핑 제어](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=500)
- [ROS control_toolbox PID](https://control.ros.org/master/doc/api/classcontrol__toolbox_1_1Pid.html)
- [MathWorks anti-windup 예제](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)
