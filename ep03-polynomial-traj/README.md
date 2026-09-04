# EP.3 — 부드러운 궤적: 3차·5차 다항식과 S-커브

관절을 **시작 각도에서 목표 각도까지 정해진 시간 안에, 덜컹이지 않고 부드럽게** 옮기는 궤적입니다.
영상에서 다룬 예제 그대로, 세 언어의 계산 프로그램이 모두 같은 결과를 만듭니다.

- 입력(코드 안에 상수로): 시작 각도 **0도** → 목표 각도 **90도** · 총 시간 **2초** · 시간 간격 0.01초
- 출력: `cubic.csv`(3차) · `quintic.csv`(5차) — 열은 `t, q, qd, qdd` = 시간, 각도, 각속도, 각가속도
- 기대 결과(검산):
  - 3차: 계수 a2 = 67.5, a3 = −22.5 → 1초(중간)에서 **45도**, 최고 각속도 **초당 67.5도**, 각가속도 135 → −135
  - 5차: 10·15·6 공식 → 1초에서 **45도**, 최고 각속도 **약 초당 84.4도**, 최고 각가속도 약 130(0에서 시작해 0으로 끝남)
- 보너스 `scurve` — S-커브(저크 제한) 프로파일: 목표 90도 · 최고 속도 60 · 최고 가속도 120 · 저크 480 → **총 2.25초**(같은 조건의 사다리꼴 2.0초보다 저크 구간 하나만큼 김)

```
[1] 계산 프로그램 실행 (Python / C / C++ 중 하나) ──→ cubic.csv, quintic.csv (scurve.csv)
[2] python visualize.py ──→ 그래프(정적/애니메이션)   또는   cpp/plot_svg ──→ trajectory.svg
```

---

## 1) Python으로 실행

**준비물**: [Python 3](https://www.python.org/downloads/) (설치 시 "Add python.exe to PATH" 체크)

```
cd python
python polynomial_traj.py      → cubic.csv, quintic.csv
python scurve.py               → scurve.csv
```

콘솔에 `cubic 중간(1.00초) 각도 = 45.00도 | 최고 각속도 = 67.500 …`, `quintic … 최고 각속도 = 84.375 …`,
`총 시간 = 2.25초, 최종 위치 = 90.00도`가 나오면 성공입니다.

## 2) C로 실행

**준비물**: 둘 중 하나
- 시험장과 같은 **Visual Studio**(무료 Community판) — 시작 메뉴에서 *Developer Command Prompt for VS* 실행
- 또는 [MinGW-w64](https://www.mingw-w64.org/)의 gcc

```
cd c

:: Visual Studio 개발자 명령 프롬프트라면
cl polynomial_traj.c
polynomial_traj.exe
cl scurve.c
scurve.exe

:: gcc(MinGW)라면
gcc polynomial_traj.c -o polynomial_traj -lm
polynomial_traj.exe
gcc scurve.c -o scurve -lm
scurve.exe
```

## 3) C++로 실행

```
cd cpp

:: Visual Studio 개발자 명령 프롬프트라면
cl /EHsc polynomial_traj.cpp
polynomial_traj.exe
cl /EHsc scurve.cpp
scurve.exe

:: g++(MinGW)라면
g++ -std=c++11 polynomial_traj.cpp -o polynomial_traj
polynomial_traj.exe
g++ -std=c++11 scurve.cpp -o scurve
scurve.exe
```

## 4) 그래프 그리기 — Python판과 C++판 모두 제공

계산 프로그램이 만든 CSV가 있는 폴더에서, 손에 익은 쪽을 고르세요.

**Python (matplotlib)**:

```
pip install matplotlib
python ..\python\visualize.py                # 정적 3단 그래프(각도·각속도·각가속도, 3차 vs 5차) → trajectory.png
python ..\python\visualize.py --animate      # 그려지는 애니메이션 → trajectory.mp4 (ffmpeg 없으면 .gif)
python ..\python\visualize.py --scurve       # S-커브 3단(속도·가속도·저크) → scurve.png
```

> `python` 폴더에서 실행했다면 `python visualize.py` 처럼 경로 없이 실행하면 됩니다.
> 영상(EP.3)에 삽입된 애니메이션은 정확히 이 스크립트(`--frames` 모드)의 산출물입니다.

**C++ (의존성 없음 — SVG 직접 쓰기)**:

```
cd cpp
cl /EHsc plot_svg.cpp        (또는 g++ -std=c++11 plot_svg.cpp -o plot_svg)
plot_svg.exe                 → trajectory.svg 생성, 브라우저로 열면 3단 그래프가 보입니다
```

> 시험장이라면 같은 CSV를 **Excel로 열어 꺾은선 차트**를 그려도 됩니다(제출물로도 정석).

## 직접 실험해 보기

코드 안의 상수를 바꿔 보세요.

- 총 시간 `T = 1` → 각속도는 2배(초당 135도), 각가속도는 4배(540)로 커집니다 — 계수의 1/T, 1/T² 규칙
- 목표 `QF = 180` → 모든 값이 정확히 2배(다항식 궤적은 목표 각도에 비례)
- `scurve`에서 저크 `J = 240` → 저크 구간이 0.5초로 늘어 총 시간이 2.5초가 됩니다(부드러움의 대가 = 시간)

## 관련 링크

- 🤖 실기 연습장에서 이 유형 문제 풀기: <https://www.techrraforming.com/practical?cert=RobotSoftware>
  (3차 다항식 관절 궤적 · 3차 다항식 관절 궤적 생성 · 5차(퀸틱) 다항식 관절 궤적 · S-커브(저크 제한) 속도 운동계획)
