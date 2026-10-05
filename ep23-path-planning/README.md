# EP.23 격자 길찾기와 안전거리

자체 실기 연습장 예제를 다익스트라·A*·점–선분 거리·선분–원 충돌 검사로 완주합니다. 공식 공개 기출이 아닙니다.

## 입력과 계산 규칙

- P1: `[[1,3,1],[1,5,1],[4,2,1]]`. 상하좌우, 시작·목표 칸을 포함한 지형비용 합계.
- P2: `[[0,0,0,0],[1,1,0,1],[0,0,0,0],[0,1,1,0]]`. 0 통행, 1 장애물, 이동비용 1, 시작비용 0.
- 격자는 `(행, 열)`, 시작 `(0,0)`, 목표 오른쪽 아래. 우선순위 `(f,g,행,열)` 오름차순. 다익스트라는 `h=0`, A*는 맨해튼 거리.
- 기하 좌표는 `(x,y)`. A=(0,0), B=(4,0). 투영 비율을 [0,1]로 제한하고 최근접점까지 거리를 계산합니다. A=B이면 나눗셈 전에 점 거리로 분기합니다.
- 길이 단위는 지문에 없으므로 붙이지 않습니다. 충돌은 반올림 전 거리와 반지름을 비교하며 접촉도 포함합니다.

| 입력 | 기대 결과 |
|---|---|
| P1 | 최소비용 7, 좌표 5개, 이동 4회 |
| P2 | 최소비용 6, 좌표 7개, 이동 6회 |
| 같은 P2, h=0 | 비용 6, 꺼낸 칸 10개(A* 8개) |
| P2의 (1,2)를 장애물로 변경 | NO PATH |
| 한 칸 지형 지도 [[7]] | 비용 7, 이동 0회 |
| 한 칸 통행 지도 [[0]] | 비용 0, 이동 0회 |
| [[1,2],[1,1]] | 비용 3 |
| [[1,9,9],[1,9,1],[1,1,1]] | 비용 5 |
| P=(2,3) | t=0.5, Q=(2,0), 거리 3.0000 |
| P=(6,0) | t=1.5 → 1, Q=(4,0), 거리 2.0000 |
| P=(-1,-1) | t=-0.25 → 0, 거리 1.4142 |
| A=B=(0,0), P=(2,3) | 거리 3.6056 |
| 원 중심 (2,1), 반지름 1 | 거리 1.0000, COLLISION |
| 원 중심 (2,2), 반지름 1 | 거리 2.0000, SAFE |

P1 경로: `(0,0) → (0,1) → (0,2) → (1,2) → (2,2)`.
P2 경로: `(0,0) → (0,1) → (0,2) → (1,2) → (2,2) → (2,3) → (3,3)`.

## 설치·컴파일·실행

각 명령은 이 폴더에서 실행합니다. 답안 자체는 Python 표준 라이브러리 또는 C/C++ 표준 라이브러리만 사용합니다. 예제 입력은 소스에 보관하며 실행 시 자동 재현합니다.

```powershell
python python/path_planning.py
gcc c/path_planning.c -o path_c.exe -lm
./path_c.exe
g++ -std=c++11 cpp/path_planning.cpp -o path_cpp.exe
./path_cpp.exe
python -m pip install matplotlib numpy
python python/visualize.py
python python/visualize.py --frames
g++ -std=c++11 cpp/plot_svg.cpp -o plot_svg.exe
./plot_svg.exe
```

Windows는 GCC/MinGW 설치 경로를 PATH에 추가합니다. CLion 번들 사용 시 `C:\Program Files\JetBrains\CLion 2026.2.0.1\bin\mingw\bin`입니다. Linux/macOS는 해당 시스템의 GCC/Clang과 Python 3을 사용합니다.

답안은 실행 디렉터리에 `paths.csv`, `trace.csv`, `distances.csv`를 생성합니다. 언어마다 별도 작업 폴더에서 실행하면 결과를 보존해 대조할 수 있습니다. 시각화는 `visualization.png`, `--frames`는 `frames/`의 12fps 프레임 시퀀스를 생성합니다. C++ 시각화는 `result.svg`를 직접 씁니다. 생성물·바이너리는 저장소에 넣지 않습니다.

## 실행 검증

Python·C·C++를 실제 실행해 stdout과 세 CSV가 동일함을 확인했습니다. 최종 비용과 시작·목표, 상하좌우 이동, 장애물 회피도 확인했습니다. 동률 규칙은 재현성을 위한 약속이며 다른 입력에서 같은 비용 경로가 여러 개이면 특정 좌표열만 정답으로 강제하지 않습니다. 꺼낸 칸 수 8 대 10은 이 지도·동률 규칙의 결과로 보편적인 속도 비율이 아닙니다.

## 실험과 흔한 실점

1. P2 통로를 막아 부모 복원 없이 NO PATH로 끝나는지 확인합니다.
2. 점을 선분 양 끝 바깥으로 이동해 직선 거리와 선분 거리를 구분합니다.
3. A=B, 시작=목표, 접촉 경계를 유지해 예외 분기가 사라지지 않게 합니다.
4. 목표를 큐에 넣는 순간 종료하지 말고 유효 최소 후보로 꺼낼 때 종료합니다.
5. 비용이 개선될 때 부모도 함께 갱신하고 오래된 큐 기록은 건너뜁니다.
6. 비용이 음수이면 이 다익스트라 확정 논리를 적용하지 않습니다. 대각선 이동·지형 가중치를 바꾸면 휴리스틱 조건도 다시 검토합니다.
7. 몸체 안전거리는 원형 로봇 가정에서 장애물·로봇 반지름·안전 여유를 반영합니다. 이 교육 예제는 실제 장비 제어기가 아닙니다.

## 관련 링크

- [공개 영상](https://youtu.be/qPPd85kKdp0)

- [EP.23 강의](https://www.techrraforming.com/academy/cert-prac-path-planning)
- [P1 지형비용 격자](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=496)
- [P2 A* 경로](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=241)
- [P3 점–선분 거리](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=515)
- [P4 선분–원 충돌](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=263)
