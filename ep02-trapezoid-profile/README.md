# EP.2 — 가속–등속–감속, 사다리꼴 속도 프로파일

로봇 축(관절)을 목표 각도만큼 돌릴 때의 **사다리꼴 속도 프로파일** 운동계획입니다.
영상에서 다룬 예제 그대로, 세 언어의 계산 프로그램이 모두 같은 결과를 만듭니다.

- 입력(코드 안에 상수로): 목표 거리 **L = 180도** · 최고 속도 **V = 초당 90도** · 가속도 **A = 매초 초당 180도**
- 출력: `profile.csv` (시간 `t`, 돌아간 각도 `s`, 그 순간의 속도 `v`)
- 기대 결과: **총 시간 2.50초, 최종 거리 180.0도** (목표를 20도로 줄이면 등속 구간이 사라진 삼각형 프로파일)

```
[1] 계산 프로그램 실행 (Python / C / C++ 중 하나) ──→ profile.csv
[2] python visualize.py ──→ 그래프(정적/애니메이션)
```

---

## 1) Python으로 실행

**준비물**: [Python 3](https://www.python.org/downloads/) (설치 시 "Add python.exe to PATH" 체크)

```
cd python
python trapezoid_profile.py
```

콘솔에 `총 시간 = 2.50초, 최종 거리 = 180.0도`가 나오고 `profile.csv`가 생기면 성공입니다.

## 2) C로 실행

**준비물**: 둘 중 하나
- 시험장과 같은 **Visual Studio**(무료 Community판) — 시작 메뉴에서 *Developer Command Prompt for VS* 실행
- 또는 [MinGW-w64](https://www.mingw-w64.org/)의 gcc

```
cd c

:: Visual Studio 개발자 명령 프롬프트라면
cl trapezoid_profile.c
trapezoid_profile.exe

:: gcc(MinGW)라면
gcc trapezoid_profile.c -o trapezoid -lm
trapezoid.exe
```

## 3) C++로 실행

```
cd cpp

:: Visual Studio 개발자 명령 프롬프트라면
cl /EHsc trapezoid_profile.cpp
trapezoid_profile.exe

:: g++(MinGW)라면
g++ -std=c++11 trapezoid_profile.cpp -o trapezoid
trapezoid.exe
```

## 4) 그래프 그리기 (언어 무관 — 공용 시각화)

계산 프로그램이 만든 `profile.csv`가 있는 폴더에서:

```
pip install matplotlib
python ..\python\visualize.py                # 정적 그래프 → trapezoid.png
python ..\python\visualize.py --animate      # 그려지는 애니메이션 → trapezoid.mp4 (ffmpeg 없으면 .gif)
```

> `python` 폴더에서 실행했다면 `python visualize.py` 처럼 경로 없이 실행하면 됩니다.
> 영상(EP.2)에 삽입된 애니메이션은 정확히 이 스크립트(`--frames` 모드)의 산출물입니다.

## 직접 실험해 보기

코드 안의 상수를 바꿔 보세요.

- `L = 20` → 가속·감속에만 45도가 필요한데 목표가 더 짧으니 **삼각형 프로파일**(꼭짓점 속도 = √(A×L) = 초당 60도)
- `V = 180` → 최고 속도를 2배로 올려도 총 시간은 2.5초 → 2.0초로 줄어들 뿐(절반이 아님)

## 관련 링크

- 🤖 실기 연습장에서 이 유형 문제 풀기: <https://www.techrraforming.com/practical?cert=RobotSoftware>
  (사다리꼴 속도 운동계획 · 사다리꼴 속도 프로파일 운동계획 · 총 이동 시간과 최고 속도 · 엔드이펙터 직선 이동 · 다축 동기화 사다리꼴)
