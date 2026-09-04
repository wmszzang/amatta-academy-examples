# EP.4 — 지문 읽고 계획 고르기: 사다리꼴 · 다항식 · S-커브 끝내기

운동계획 단원의 **끝내기 편**입니다. 영상: <https://youtu.be/fGxWcXUkPFw> 시험 지문을 읽고 **어떤 계획으로 풀지 판별**한 뒤, 네 지문을 끝까지 풉니다.
영상에서 다룬 지문 네 개가 그대로 코드 상수로 들어 있고, 세 언어의 계산 프로그램이 모두 같은 CSV를 만듭니다.

| 지문 | 주어진 것 | 판별 | 기대 결과(검산) |
|---|---|---|---|
| ① 목표 150도 · 최대 속도 60 · 최대 가속도 120 · dt 0.01 | 한계 두 개, 시간 없음 | **사다리꼴** | 가속 0.5 s + 등속 2.0 s + 감속 0.5 s = **총 3.0 s**, 최종 150도 (목표 20도면 삼각형: 꼭짓점 48.99, 0.816 s) |
| ② 30도 → 120도 · 3초 · 시작·끝 정지 · dt 0.1 | 총 시간 + 속도 0 두 개(조건 4) | **3차 다항식** | 계수 30 · −6.667, 1.5 s에 **75도**, 최고 각속도 **45**, 각가속도 60 → −60 |
| ③ −45도 → 45도 · 4초 · 각속도·각가속도 모두 0 · dt 0.2 | 총 시간 + 조건 6 | **5차 다항식** | 2 s에 **0도**, 최고 각속도 **42.19**, 각가속도 0에서 시작·약 32.5·0으로 끝 |
| ④ 목표 180도 · 속도 90 · 가속도 180 · 저크 720 · dt 0.005 | 한계 셋(저크까지) | **S-커브** | 7구간 0.25/0.25/0.25/1.25/0.25/0.25/0.25 = **총 2.75 s** (같은 한계의 사다리꼴 2.5 s + 저크 구간 0.25 s) |

판별 규칙(영상 §갈림길): **총 시간이 주어지면 다항식**(각가속도까지 0이면 5차, 아니면 3차) · **한계가 주어지면 사다리꼴 계열**(저크 한계까지 있으면 S-커브).
Python판의 `choose_plan()` 함수가 이 규칙을 코드로 옮긴 것입니다.

- 출력 파일: `p1_trapezoid.csv` (t, x, v, a) · `p2_cubic.csv` (t, q, qd, qdd) · `p3_quintic.csv` (t, q, qd, qdd) · `p4_scurve.csv` (t, x, v, a, j)
- ⭐ **S-커브 완결판**: 지문 ④의 "도달하지 못하면 해당 구간을 0으로" 조건을 **확인 ①(가속 일정 구간이 있는가)·확인 ②(등속 구간이 있는가)** 두 분기로 처리합니다. 등속이 없으면 목표 거리에 맞는 최고 속도를 근의 공식으로 다시 구하고, 그래도 가속 일정 구간이 없으면 저크 구간만 남깁니다. 구간 끝은 마지막 걸음만 짧게 밟아(`h = min(dt, t_end - t)`) 정확히 맞춥니다. (Python판 `__main__`이 목표 30도 사례로 확인 ②를 실증합니다: 등속 0 s, 최고 속도 54.35, 최종 30.00도.)

```
[1] 계산 프로그램 실행 (Python / C / C++ 중 하나) ──→ p1_trapezoid.csv … p4_scurve.csv
[2] python visualize.py ──→ 그래프(정적/애니메이션)   또는   cpp/plot_svg ──→ motion_wrapup.svg
```

---

## 1) Python으로 실행

**준비물**: [Python 3](https://www.python.org/downloads/) (설치 시 "Add python.exe to PATH" 체크)

```
cd python
python motion_wrapup.py
```

콘솔에 `지문 ① 판별: trapezoid … 총 시간 3.00초`, `지문 ② … 중간 1.5초 각도 75.00 · 최고 각속도 45.00`,
`지문 ③ … 중간 2초 각도 0.00 · 최고 각속도 42.1875`, `지문 ④ … 총 시간 2.750초 · 최종 위치 180.00도`가 나오면 성공입니다.
(한글이 깨지면 `set PYTHONIOENCODING=utf-8` 후 다시 실행하세요.)

## 2) C로 실행

**준비물**: 둘 중 하나
- 시험장과 같은 **Visual Studio**(무료 Community판) — 시작 메뉴에서 *Developer Command Prompt for VS* 실행
- 또는 [MinGW-w64](https://www.mingw-w64.org/)의 gcc

```
cd c

:: Visual Studio 개발자 명령 프롬프트라면
cl motion_wrapup.c
motion_wrapup.exe

:: gcc(MinGW)라면
gcc motion_wrapup.c -o motion_wrapup -lm
motion_wrapup.exe
```

## 3) C++로 실행

```
cd cpp

:: Visual Studio 개발자 명령 프롬프트라면
cl /EHsc motion_wrapup.cpp
motion_wrapup.exe

:: g++(MinGW)라면
g++ -std=c++11 motion_wrapup.cpp -o motion_wrapup
motion_wrapup.exe
```

## 4) 그래프 그리기 — Python판과 C++판 모두 제공

계산 프로그램이 만든 CSV가 있는 폴더에서, 손에 익은 쪽을 고르세요.

**Python (matplotlib)**:

```
pip install matplotlib
python ..\python\visualize.py                # 4행 × 3열(위치·속도·가속도) 정적 그래프 → motion_wrapup.png
python ..\python\visualize.py --profiles     # 속도 프로파일 2×2 비교 → profiles.png
python ..\python\visualize.py --animate      # 네 프로파일이 그려지는 애니메이션 → profiles.mp4 (ffmpeg 없으면 .gif)
```

> `python` 폴더에서 실행했다면 `python visualize.py` 처럼 경로 없이 실행하면 됩니다.
> 영상(EP.4)에 삽입된 애니메이션은 정확히 이 스크립트(`--frames` 모드)의 산출물입니다.

**C++ (의존성 없음 — SVG 직접 쓰기)**:

```
cd cpp
cl /EHsc plot_svg.cpp        (또는 g++ -std=c++11 plot_svg.cpp -o plot_svg)
plot_svg.exe                 → motion_wrapup.svg 생성, 브라우저로 열면 2×2 속도 프로파일이 보입니다
```

> 시험장이라면 같은 CSV를 **Excel로 열어 꺾은선 차트**를 그려도 됩니다(제출물로도 정석).

## 직접 실험해 보기

코드 안의 상수를 바꿔 보세요.

- 지문 ①의 목표 `150 → 20` : 가속·감속 거리(V²/A = 30도)보다 짧아 **삼각형**이 됩니다(꼭짓점 48.99, 총 0.816 s)
- 지문 ②의 시작 각도 `30 → 0` : 상수항이 0이 되고 모든 각도가 30도씩 내려갑니다 — 상수항 = 시작 각도라는 뜻
- 지문 ④의 목표 `180 → 30` : 확인 ②가 작동해 등속 구간이 사라지고 최고 속도가 54.35로 낮아집니다(총 1.104 s, 최종 30.00도)
- 지문 ④의 저크 `720 → 360` : 저크 구간이 0.5 s로 늘어 총 시간이 3.0 s가 됩니다(부드러움의 대가 = 시간)

## 관련 링크

- 🤖 실기 연습장에서 이 유형 문제 풀기: <https://www.techrraforming.com/practical?cert=RobotSoftware>
  (사다리꼴 속도 프로파일 운동계획 · 3차 다항식 관절 궤적 생성 · 5차(퀸틱) 다항식 관절 궤적 · S-커브(저크 제한) 속도 운동계획 · 사다리꼴 — 총 이동 시간과 최고 속도)
