# EP.17 — 쿼터니언 곱 · SLERP · 측지 거리

네 성분은 **(w,x,y,z)** 순서입니다. 벡터 회전은 `q * (0,v) * conjugate(q)`입니다.
고정 좌표계에서 **먼저 q1, 다음 q2를 적용하면 q2*q1**입니다. 지문이 `q1*q2`를 요구하면 그대로 계산합니다.

| 문제 | 입력 | 계산 · 채점 조건 | 기대값 |
|---|---|---|---|
| #290 | (.7071,0,0,.7071) 두 개 | 해밀턴 곱 뒤 결과 정규화 | (0,0,0,1) |
| #490 | 같은 입력 | 원래 곱, 정규화 없음 | (0,0,0,.99998082) |
| #259 | 항등, z90, t=.25/.5/.75 | 입력 정규화, 음수 내적 부호 반전, 근접 분기 | 22.5 / 45 / 67.5도 |
| #559 | 단위 입력 쌍 | 절댓값 · 클램프 · 두 배 | 항등-z90 90 / q와 -q 0 / 항등-z180 180 / z90-x90 120도 |

소수 네 자리 반올림은 출력할 때만 합니다. #490의 마지막 성분도 출력은 `1.0000`이지만 정규화한 것은 아닙니다.
SLERP와 거리 함수는 단위 입력을 전제로 합니다. `solve_259.py`는 입력을 먼저 정규화합니다.
C/C++ 정규화 함수와 축 생성 함수는 영이 아닌 입력을 전제로 하며, 예제 main은 유효 입력만 사용합니다.
가까운 구간(d>0.9995)의 정규화 LERP는 안정적인 근사이고 정확한 등각속도를 보장하지 않습니다.

## 설치와 실행

Python 3을 설치한 뒤 계산은 외부 라이브러리 없이 실행합니다. 시각화에는 matplotlib와 numpy가 필요합니다.
아래 명령은 에피소드 폴더에서 실행합니다.

```text
python python/main.py
python python/test_check.py
python -m pip install matplotlib numpy
python python/visualize.py
python python/visualize.py --frames
```

문제별 풀이의 표준 입력은 두 줄의 `w x y z`입니다. #259만 마지막 줄에 t가 추가됩니다.
예: `1 0 0 0`, `0.7071 0 0 0.7071`, `0.5`를 `python python/solve_259.py`로 전달하면
`0.9239,0.0000,0.0000,0.3827`입니다. 출력은 쉼표로 구분합니다.

MinGW-w64의 gcc/g++ 또는 Visual Studio 개발자 명령 프롬프트를 사용합니다.

```text
gcc -std=c11 c/main.c c/quat.c c/slerp.c c/distance.c -o quaternion-c.exe -lm
g++ -std=c++11 cpp/main.cpp cpp/quat.cpp cpp/slerp.cpp cpp/distance.cpp -o quaternion-cpp.exe
g++ -std=c++11 cpp/plot_svg.cpp cpp/quat.cpp cpp/slerp.cpp cpp/distance.cpp -o plot-svg.exe
quaternion-c.exe
quaternion-cpp.exe
plot-svg.exe
```

Visual Studio: C는 `cl /std:c11 c/main.c c/quat.c c/slerp.c c/distance.c`,
C++는 `cl /EHsc cpp/main.cpp cpp/quat.cpp cpp/slerp.cpp cpp/distance.cpp`입니다.
세 언어 main의 콘솔 결과와 `angles.csv`를 대조할 수 있습니다.

## 실행 그림과 실험

`visualize.py`는 `arc.png`, `angles.png`를 만들고 `--frames`는 `frames/arc/`, `frames/angles/`에
12fps 프레임을 만듭니다. 구 그림은 쿼터니언의 4차원 단위 구를 설명하기 위한 단면입니다.
별도 좌표축은 실제 3차원 물체 방향입니다. `plot-svg.exe`는 C++ 계산으로 `angles.svg`를 직접 씁니다.

1. 목표 쿼터니언의 네 부호를 뒤집어도 최단 호 보간값이 같은지 확인하세요.
2. 시작과 끝을 같은 자세로 두어 0 나눗셈 없이 반환하는지 확인하세요.
3. 170도 회전을 t=.25와 .5에서 비교하세요. 중간점 하나만 보면 속도 차이를 놓칩니다.
4. qz*qx와 qx*qz를 비교하세요. 끝 자세 차이는 120도입니다.

흔한 실점은 성분 순서 혼동, 곱 순서 임의 반전, #490에도 정규화 적용,
SLERP 두 갈래 누락, 거리 절댓값·계수 2·클램프 누락입니다.
진행률이 시간에 비례할 때 SLERP 일반식은 일정 각속도입니다. 한 구간의 자세 보간이며
관절 특이점 회피나 로봇 전체 가감속을 보장하는 기능은 아닙니다.
회전 표현 판별·회전 벡터 적분·쿼터니언 시간 미분은 이 예제 범위에 넣지 않습니다.

## 관련 링크

- [강의 페이지](https://www.techrraforming.com/academy/cert-prac-quaternion-slerp)
- [EP.16 축-각도와 쿼터니언](../ep16-axis-angle-quaternion/)
- EP.18 회전 표현 판별은 후속 편이며 현재 예제에는 포함하지 않습니다.
- [#290 곱과 정규화](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=290)
- [#490 지정 순서 곱](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=490)
- [#259 SLERP](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=259)
- [#559 측지 거리](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=559)

위 문항은 사이트 자체 연습문제입니다. 공식 기출 복제라고 주장하지 않습니다.
