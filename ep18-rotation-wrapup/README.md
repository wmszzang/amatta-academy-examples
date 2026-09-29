# EP.18 — 이 지문은 어떤 회전 표현을 쓰라는 말인가

회전행렬·오일러각·축각·쿼터니언은 표현이고, 합성·보간·거리·적분은 연산입니다.
공통 규약은 **능동 회전, 열벡터, 쿼터니언 `(w,x,y,z)`**입니다. 계산은 반올림하지 않고 CSV 출력만 소수 넷째 자리로 맞춥니다. 회전행렬은 `Rz(yaw) Ry(pitch) Rx(roll)`이며 내재 ZYX와 고정축 XYZ가 같은 결과입니다.

| 지문 | 주어진 것 | 신호어 | 계산 | 기대 결과·검산 |
|---|---|---|---|---|
| #252 | 반올림 전 행렬 9성분 | 롤·피치·요 | 오일러 분해 | `(0.3000,-0.5000,1.2000)` rad, 재구성 일치 |
| #400 | 축 `(1,2,2)`, 1 rad | 축·각에서 행렬 | 로드리게스 | 첫 행 `(0.5914,-0.4588,0.6631)`, det=1 |
| #244 | `(0.7071,0,0.7071,0)` | 3×3, 직접 구성 | 정규화·원소식 | y축 90도, 직교성·행렬식 검사 |
| #290 | z축 90도 두 개 | 결과 정규화 | 해밀턴 곱 후 정규화 | `(0,0,0,1)` |
| #490 | 항등원·x축 180도 | 적힌 곱 순서 | 대수적 해밀턴 곱 | `(0,1,0,0)`, 항등원 유지 |
| #259 | 항등 자세·z축 90도·비율 | 중간 자세 | SLERP | 절반 `(0.9239,0,0,0.3827)`, 45도 |
| #559 | 자세 두 개 | 최소 회전각 | 측지 거리 | 90·0·0·180도 |
| #292 | 각속도 표본·간격·바이어스 | 시간 누적 | 사다리꼴 적분 | 0.1000 rad, 미보정 0.2500 rad |

```text
요구 산출물
  +-- 9성분 -> 각 3개 ........ 오일러 분해
  +-- 축과 각 -> 행렬 ....... 로드리게스
  +-- 네 성분 -> 행렬 ....... 원소식
  +-- 두 회전 -> 합성 ....... 해밀턴 곱
  +-- 두 회전 + 비율 ........ SLERP
  +-- 두 회전 -> 최소 차이 .. 측지 거리
  +-- 속도 표본 + 시간 ..... 적분
```

`python/rot.py`는 핵심 함수 9개와 보조 함수 `unit`, `clamp`, `mul`을 담습니다. 준비된 회전 변환 라이브러리를 호출하지 않습니다. C와 C++도 같은 공식을 직접 구현합니다.

| 채점 대상 | 제출 전 확인 |
|---|---|
| 오일러 | 축·출력 순서, 양·음 짐벌락 분기, 재구성 |
| 로드리게스 | 축 정규화, 반대칭 부호, K²는 행렬곱 |
| 쿼터니언 변환 | 원소식 9개, 직교성, det=+1 |
| 해밀턴 | wxyz, 부호, 적용 순서와 곱 순서 구별, 지정된 정규화, 항등원 |
| SLERP | 부호 반전, 근접 분기, 사인 가중치, 끝점·단위 길이 |
| 거리 | 절댓값, 정규화 뒤 클램프, 계수 2, 도 변환 |
| 적분 | 바이어스 제거 선행, 간격·초기값, 사다리꼴 누적 |

## 실행

Python 3의 표준 라이브러리로 수치 예제를 실행합니다. 시각화에만 `numpy matplotlib`이 필요합니다.

```sh
python python/run_examples.py --out output-python
python python/p1.py
python -m pip install numpy matplotlib
python python/visualize.py --out output-visual
python python/visualize.py --frames --out output-visual
gcc -std=c11 c/rot.c -o rot-c -lm
g++ -std=c++11 cpp/rot.cpp -o rot-cpp
g++ -std=c++11 cpp/plot_svg.cpp -o plot-svg
```

C/C++ 실행 파일을 각각 별도 출력 디렉터리에서 실행하면 현재 디렉터리에 7개 CSV를 만듭니다. Windows에서는 실행 파일 이름에 `.exe`를 붙입니다. C/C++ 빌드에는 MinGW 또는 동등한 컴파일러를 사용합니다. CSV를 서로 덮어쓰지 않도록 출력 폴더를 분리합니다. `plot-svg`는 현재 폴더에 `slerp.svg`를 만듭니다. 모든 수치 묶음은 재구성 또는 해당 연산의 교차 검산을 실행하며 오차 한계 `1e-10`을 검사합니다.

```text
정밀 입력 -> 정규화/조건 분기 -> 직접 공식 -> 원정밀도 검산
                                          -> CSV 4자리 표시
CSV와 같은 입력 -> visualize.py -> 정적 PNG / 180 PNG 프레임
```

## CSV 규약

| 파일 | 열 |
|---|---|
| euler.csv | case,r00..r22,roll,pitch,yaw,lock |
| rod.csv | kx,ky,kz,theta,r00..r22,det |
| quat.csv | w,x,y,z,r00..r22,orth_err |
| hamilton.csv | order,w,x,y,z,geo_deg |
| slerp.csv | t,w,x,y,z,angle_deg,mode |
| dist.csv | case,deg |
| gyro.csv | i,omega,corrected,theta |

각 행렬은 행 우선입니다. 오일러·로드리게스 입력과 헤딩은 rad, 각속도는 rad/s, 간격은 s, 거리 및 보간 각도는 도입니다. `hamilton.csv`의 order는 문제 번호와 정규화 정책을 포함합니다. 보간의 마지막 `LERP_NEAR` 행은 별도의 근접 입력 예제입니다.

## 실험과 흔한 실점

- `Rz(1.2) Ry(-0.5) Rx(0.3)`에서 구한 원정밀도 행렬을 입력합니다. 화면의 4자리 행렬을 입력하면 롤 표시가 0.2999가 되어 다른 결과입니다.
- 짐벌락에서 yaw=0은 지문이 허용한 대표해입니다. 원래 롤과 요를 각각 복원했다고 주장하지 않습니다. 전치로 역회전하는 능력은 유지됩니다.
- `R_to_q`의 대각합이 작은 분기는 180도 회전에 필요합니다. 쿼터니언 부호가 다를 수 있으므로 마지막 행렬을 비교합니다.
- `hamilton`은 정규화하지 않는 대수적 곱입니다. #290만 호출자가 결과를 정규화합니다. 먼저 A, 다음 B의 적용은 `B*A`입니다.
- SLERP의 등각속도는 t가 시간에 비례할 때입니다. 손끝의 위치가 직선으로 이동한다는 뜻은 아닙니다.
- 오일러 세 성분의 중점은 SLERP 중점과 30.5517도 다릅니다. 두 행렬의 칸별 평균은 det=0.4962로 순수 회전이 아닙니다.
- 자이로 예제는 고정된 한 축만 적분합니다. 대칭 입력에서는 좌측 직사각형도 우연히 0.1이 됩니다. 방법 비교에는 `[0,0.4,0.8,1.2]`를 사용하면 사다리꼴 0.9, 좌측 0.6, 우측 1.2입니다.

### 이중 덮개와 최단호

단위 쿼터니언 q와 -q는 같은 회전을 나타냅니다. 보간할 때 내적이 음수이면 끝점의 네 부호를 모두 뒤집어 같은 자세의 가까운 표기를 선택합니다. 거리 계산도 내적의 절댓값을 취합니다. 이 성질을 실제 로봇의 기계적 특이점 제거로 확대하면 안 됩니다.

## 관련 링크

- [강의](https://www.techrraforming.com/academy/cert-prac-rotation-wrapup)
- [실기 연습장](https://www.techrraforming.com/practical?cert=RobotSoftware)
- 문제: [252](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=252), [400](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=400), [244](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=244), [290](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=290), [490](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=490), [259](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=259), [559](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=559), [292](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=292).
