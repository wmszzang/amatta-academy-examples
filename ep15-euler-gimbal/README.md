# EP.15 — 오일러각 ZYX와 짐벌락

회전행렬에서 롤·피치·요를 구하고, 피치가 직각일 때의 분기를 검산합니다.
오른손 좌표계와 열벡터를 사용하며 `R = Rz(yaw) Ry(pitch) Rx(roll)`입니다.
이동축 ZYX와 고정축 XYZ는 같은 회전입니다. 각도 단위는 라디안입니다.

## 입력·계산·기대값

기본 실행은 세 사례를 계산합니다. `--stdin`은 행 우선 순서의 유한한 숫자 9개를 받습니다.
내부 계산값을 반올림하지 않고 출력만 소수 네 자리로 표시합니다.

| 사례 | 입력 (roll, pitch, yaw) | 출력 (roll, pitch, yaw) |
|---|---|---|
| 정상 | (0.3, -0.5, 1.2) | (0.3000, -0.5000, 1.2000) |
| 양의 직각 | (0.2, pi/2, 0.7) | (-0.5000, 1.5708, 0.0000) |
| 음의 직각 | (0.2, -pi/2, 0.7) | (0.9000, -1.5708, 0.0000) |

`pitch = atan2(-r31, hypot(r11,r21))`로 피치를 구합니다.
`abs(cos(pitch)) < 1e-6`이면 요를 0으로 고정하고
`roll = atan2(-r23,r22)`로 계산합니다. 그 밖에는
`roll = atan2(r32,r33)`, `yaw = atan2(r21,r11)`입니다.
첨자 r31은 Python의 `R[2][0]`, C/C++ 일차원 배열의 `R[6]`입니다.

세 각을 다시 회전행렬로 만들어 원래 행렬과 비교합니다.
정확한 직각의 왕복 오차는 약 4e-17이며 플랫폼별 부동소수점 차이가 있습니다.
직각 근처를 같은 분기로 묶는 경우에는 근사 오차가 생깁니다.
자이로 `[0.1,0.2,0.2,0.1]`, 시간 간격 0.5초, 바이어스 0.1,
초기 헤딩 0의 사다리꼴 적분 결과는 **0.1000 rad**입니다.

## Python

Python 3를 설치하고 터미널을 이 에피소드 폴더에서 엽니다.
계산 자체는 표준 라이브러리만 사용합니다.

```powershell
python python/euler_gimbal.py --out output-python --self-test
python -m pip install numpy matplotlib
python python/visualize.py --out output-visual
python python/visualize.py --gimbal --frames --out output-gimbal
```

`--self-test`는 정상 자세 5,000회, 양·음 직각, 분기 임계값 안팎,
사다리꼴 적분을 검사합니다. 시각화의 `--frames`는 73장(12fps 재생)을 만듭니다.
`--order xyz`는 순서를 바꾼 비교 그림입니다. 오른쪽 동작축 그림은 ZYX 기준입니다.

## C / C++

GCC/G++가 포함된 MinGW를 설치하고 bin 폴더를 PATH에 추가합니다.
다음 명령은 에피소드 폴더에서 실행합니다.

```powershell
gcc c/euler_gimbal.c -o euler-c.exe -lm
g++ -std=c++11 cpp/euler_gimbal.cpp -o euler-cpp.exe
./euler-c.exe
./euler-cpp.exe
g++ -std=c++11 cpp/plot_svg.cpp -o plot-svg.exe
./plot-svg.exe
```

계산 실행은 현재 폴더에 `euler.csv`, `roundtrip.csv`, `heading.txt`를 만듭니다.
언어별 결과를 보존하려면 별도 폴더에서 각 실행 파일을 실행하세요.
`plot-svg.exe`는 현재 폴더의 `euler.csv`를 읽어 `euler.svg`를 만듭니다.
SVG는 브라우저로 열 수 있으며 Python 설치가 필요 없습니다.

## 실험과 흔한 실점

- `--stdin`으로 단위행렬 `1 0 0 0 1 0 0 0 1`을 입력하면 세 각이 0입니다.
- 직각에서 롤과 요를 함께 바꿔도 같은 자세가 되는지 행렬로 비교하세요.
- 요를 0으로 고정한 답을 원래 각도와 직접 비교하면 정답을 오답으로 볼 수 있습니다.
- `r23` 대신 `r12`를 쓰거나 각도 단위를 섞으면 왕복 검증에 실패합니다.
- `atan2`가 계산됐다는 사실만으로 입력이 올바른 회전행렬임이 보장되지는 않습니다.
- 이 예제는 유효한 회전행렬을 입력으로 가정하며 센서 행렬 정규화까지 수행하지 않습니다.
- 적분은 바이어스를 먼저 빼고 이웃 값의 평균을 사용합니다.

## 관련 링크

- [YouTube 공개 강의](https://youtu.be/7qo6uNENz-o)

- [강의](https://www.techrraforming.com/academy/cert-prac-euler-gimbal)
- [252번: 회전행렬 분해](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=252)
- [292번: 자이로 적분](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=292)
