# EP.12 — 작업공간·조작성 지수·요구사항 분석

손끝이 목표에 닿는지와 그 자세가 요구조건을 만족하는지를 나누어 검사합니다. 길이 2와 1.5인 평면 두 링크 팔을 사용하며, 길이의 단위는 입력과 같은 공통 단위입니다.

[YouTube](https://youtu.be/QQ_1Bz3xvc0) · [강의 페이지](https://www.techrraforming.com/academy/cert-prac-workspace-manipulability) · [실기 연습장](https://www.techrraforming.com/practical?cert=RobotSoftware)

## 입력과 기대값

관절 한계는 첫 관절 -30°~120°, 둘째 관절 15°~150°입니다. 조작성 지수 하한은 1, 조건수 상한은 10, 순기구학 왕복 오차 허용치는 1e-6입니다. 코드 내부 각도는 라디안, CSV 각도는 도입니다.

| 목표점 | 판정 | 관절각(도) | 조작성 w | 조건수 κ |
|---|---|---|---:|---:|
| (2.5, 1.5) | PASS | (2.4773, 67.9757) | 2.7811 | 3.5866 |
| (1.0, 2.6) | PASS | (37.5539, 75.4238) | 2.9034 | 3.1279 |
| (3.4, 0.5) | FAIL_ILL_CONDITIONED | (-1.0771, 22.0790) | 1.1277 | 12.3877 |
| (0.3, 0.2) | FAIL_UNREACHABLE_INNER | 해 없음 | — | — |
| (3.6, 0.2) | FAIL_UNREACHABLE_OUTER | 해 없음 | — | — |
| (-1.0, -2.0) | FAIL_JOINT_LIMIT | 허용 해 없음 | — | — |

검사 순서: 반지름 → 역기구학 두 해 → 관절 한계 → 조작성 기준 해 선택 → 순기구학 왕복 오차 → w와 κ 문턱.

- `w = |det J| = 3 |sin(theta2)|`, `κ(J) = sigma_max / sigma_min`입니다. 손끝 속도 타원의 넓이는 `πw`이며, 작업공간의 넓이와 다릅니다.
- 제한 없는 작업공간은 반지름 0.5~3.5의 고리이며 넓이는 37.6991입니다.
- 관절 한계를 5°로 표본화하면 868점, 볼록껍질은 54개 꼭짓점과 넓이 19.3008528을 갖습니다. 셀 중심을 검사한 0.005 간격 격자 넓이는 14.3884500입니다. 볼록껍질은 오목부를 채워 약 34.1% 과대평가합니다.
- 원 장애물 중심 (1.6, 1.2), 반지름 0.35, 전 회전 범위 5° 격자에서는 충돌 자세 452개, 비충돌 자세 4732개입니다. 링크 두께·자체 충돌은 제외하며 접촉은 충돌입니다. 표본 사이 경로의 안전까지 보장하지 않습니다.
- 길이 (1.0, 0.8, 0.3)인 별도 3링크 예제의 기교 작업공간은 `r ∈ [0,0.1] ∪ [0.5,1.5]`, 넓이 6.3146입니다. 가운데 작은 원판을 빠뜨리지 않습니다.

## Python 설치·계산

Python 3.8 이상을 설치하고 터미널을 이 에피소드 폴더에서 엽니다. 계산 자체는 표준 라이브러리만 사용합니다.
`kin_common.py`가 EP.10의 역기구학·순기구학과 EP.11의 자코비안을 재사용하므로 저장소 전체를 내려받아 에피소드 간 폴더 구조를 유지합니다.

```powershell
python python/workspace.py --out output-python
```

`workspace.csv`, `spec.csv`, `hull.csv`, `cspace.csv`, `area.txt`가 생성됩니다. 콘솔 출력의 판정과 위 표를 대조합니다. `fk_err`는 반올림하면 0이 될 수 있지만 계산이 언제나 정확히 0이라는 뜻은 아닙니다.

## C·C++ 컴파일·실행

MinGW-w64의 gcc와 g++를 설치하고 PATH에 추가합니다. 출력 폴더는 실행 전에 만듭니다. C++판은 `../c/workspace.c`를 포함하므로 폴더 구조를 유지합니다.

```powershell
New-Item -ItemType Directory -Force output-c, output-cpp
gcc c/workspace.c -o output-c/workspace.exe -lm
./output-c/workspace.exe output-c
g++ -std=c++11 cpp/workspace.cpp -o output-cpp/workspace.exe
./output-cpp/workspace.exe output-cpp
```

세 언어의 CSV 네 종류와 `area.txt`는 줄바꿈 형식을 제외하고 동일하게 출력되도록 검증했습니다. 콘솔의 표현 형식은 언어별로 다릅니다.

## 그래프·프레임·SVG

```powershell
python -m pip install numpy matplotlib
python python/visualize.py --out figures
python python/visualize.py --frames --out frames
python python/visualize.py --ellipse --frames --out ellipse-frames
g++ -std=c++11 cpp/plot_svg.cpp -o output-cpp/plot_svg.exe
./output-cpp/plot_svg.exe output-cpp
```

`visualize.py`는 annulus, limits, manip, ellipse, hull, cspace의 여섯 모드를 제공합니다. 정적 PNG는 모드별 한 장, `--frames`는 모드별 48장입니다. 12 fps 재생용이며 영상에 삽입하는 프레임과 같은 코드입니다. 시각화는 기본 입력으로 직접 다시 계산합니다. CSV의 임의 변경을 자동으로 읽지는 않습니다. SVG 생성기는 전달한 폴더의 `workspace.csv`와 `hull.csv`를 읽어 `workspace.svg`를 만들며 브라우저로 열 수 있습니다.

## 실험과 흔한 실점

1. Python의 `spec_check(3.4, .5, k_max=15)`를 호출하면 조건수 문턱 변경으로 판정이 바뀝니다. 요구사항이 바뀌었음을 결과와 함께 기록합니다.
2. 두 링크 길이를 같게 바꾸면 안쪽 구멍이 사라집니다. 이번 기본 예제의 0.5 하한을 다른 팔에 그대로 쓰지 않습니다.
3. 한계 필터를 빼면 수학적으로 존재하지만 실제 관절이 취할 수 없는 해까지 합격시킵니다.
4. 조건수 κ(J)와 κ(JJᵀ), 역비율을 혼용하지 않습니다. 이 코드는 위치 전용 2×2 자코비안 기준입니다.
5. 볼록껍질 넓이를 정확한 작업공간 넓이로 제출하지 않습니다. 격자 넓이는 해상도에 따른 근삿값입니다.

관련 문제: [#257 도달성·조작성](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=257), [#287 신발끈 공식](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=287), [#497 볼록껍질](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=497).
