# EP.10 — 해석적 역기구학: 두 해·도달 불가·해 선택

목표 위치와 두 링크 길이로 관절각을 계산하고, 원래 정밀도의 각을 순기구학에 넣어 목표 좌표를 복원합니다. 목표는 위치만 지정하며 집게 방향은 지정하지 않습니다.

## 입력과 출력

`--a1 --a2 --x --y`는 두 길이와 목표 좌표입니다. 기본값은 `2, 1.5, 2.5, 1.5`입니다. `--elbow both|down|up`, `--unit rad|deg`, `--out solutions.csv`를 선택합니다. `--boundary`는 3.5 길이의 팔이 60도 방향으로 완전히 펴진 좌표를 반올림 없이 계산합니다.

CSV 열은 `elbow,theta1,theta2,unit,fk_x,fk_y,status`입니다. 두 각은 정규화 후 표시하고, FK 검산은 반올림 전 값을 사용합니다. 도달 불가에서는 각도를 출력하지 않습니다. 같은 길이·원점 목표의 연속해는 `CONTINUUM`으로 구분합니다.

| 예제 | 입력 | 기대 출력 |
|---|---|---|
| #222 | a=(1,1), T=(1.5,1), down, deg | (8.0312,51.3178), FK=(1.5000,1.0000) |
| #242 | a=(2,1.5), T=(2.5,1.5), both, rad | down=(0.0432,1.1864), up=(1.0376,-1.1864), FK=(2.5000,1.5000) |
| 바깥 | T=(4,0.5) | UNREACHABLE |
| 안쪽 구멍 | T=(0.2,0.1) | UNREACHABLE |
| 바깥 경계 | T=(3.5,0) | 서로 다른 해 1개 |
| 안쪽 경계 | T=(0.5,0) | 서로 다른 해 1개 |
| 같은 길이·원점 | a=(1,1), T=(0,0) | CONTINUUM: theta2=pi, theta1 자유 |
| 소수 경계 | --boundary --unit deg | (60.0000,0.0000) |

소수 리터럴 `(1.75,3.0310889132)`는 반올림된 입력이므로 `c2 > 1`을 재현하지 않습니다. `--boundary`가 `3.5*cos(pi/3), 3.5*sin(pi/3)`로 만든 원래 정밀도의 좌표를 사용합니다. 마지막 자리는 환경에 따라 달라질 수 있습니다.

## Python

Python 3을 설치하고 이 폴더에서 실행합니다. 계산기는 표준 라이브러리만 사용합니다.

```bash
python python/ik2link.py --out solutions.csv
python python/ik2link.py --a1 1 --a2 1 --x 1.5 --y 1 --elbow down --unit deg
python python/ik2link.py --x 4 --y 0.5
python python/ik2link.py --boundary --unit deg
```

그림에는 matplotlib과 numpy가 필요합니다.

```bash
python -m pip install matplotlib numpy
python python/visualize.py
python python/visualize.py --frames
```

정적 PNG와 두 도착 배치를 교대로 보여 주는 PNG 프레임을 만듭니다. 두 자세 사이에서 끝점을 고정한 채 움직이는 물리 궤적을 뜻하지 않습니다.

## C와 C++

GCC 또는 MinGW를 설치해 PATH에 추가합니다. C++11로 컴파일할 수 있습니다.

```bash
gcc c/ik2link.c -o ik2link-c.exe -lm
g++ -std=c++11 cpp/ik2link.cpp -o ik2link-cpp.exe
g++ -std=c++11 cpp/plot_svg.cpp -o plot-svg.exe
./ik2link-c.exe --out solutions-c.csv
./ik2link-cpp.exe --out solutions-cpp.csv
./plot-svg.exe
```

계산 프로그램의 입력 옵션은 Python과 같습니다. C++ `ik2`는 성공 여부를 bool로 반환하고, 참조 인자에 두 각을 저장합니다. `plot_svg`는 기본 #242 두 자세를 SVG로 만듭니다.

## 실험과 채점 체크리스트

- 두 부호에서 첫째·둘째 관절각을 모두 다시 계산합니다.
- 먼저 `abs(c2)>1+eps`를 검사하고, 허용오차 안의 작은 넘침만 클램핑합니다.
- 원래 정밀도의 각으로 FK 오차가 `1e-9`보다 작은지 검사합니다.
- 경계의 중복 해를 제거하고, 연속해를 두 개라고 세지 않습니다.
- 출력 단위·자릿수와 정규화 범위를 확인합니다.
- Python `pick`은 관절 한계 필터와 현재 배치의 래핑 거리로 선택합니다. 특이점 회피까지 구현한 선택기는 아닙니다.
- 현재 (10도,40도), 둘째 관절 한계 0~150도에서는 다운 해가 선택됩니다.

12가지 입력에서 Python·C·C++의 CSV 표준 출력을 실제 실행해 일치 확인했습니다. 경계에서도 `--elbow up`은 합쳐진 해 하나를 출력합니다. 안쪽 경계의 `-pi`와 `pi`는 같은 방향입니다. 비교 기대값은 `tests/expected.csv`입니다. #222 원문 예시의 8.04도는 반올림 오기이며 정확한 둘째 자리 표기는 8.03도입니다.

## 관련 링크

- [강의 페이지](https://www.techrraforming.com/academy/cert-prac-ik-analytic)
- [#222 지정된 한 해](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=222)
- [#242 두 해와 도달 불가](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=242)
- [#465 삼변측량 연결 문제](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=465)
