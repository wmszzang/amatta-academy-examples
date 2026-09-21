# EP.16 축 하나와 각도 하나로

축-각도·로드리게스 공식·단위 쿼터니언·회전행렬의 양방향 변환을 C, C++, Python으로 직접 계산합니다. 사이트 자체 출제 실기 연습문제 [#400](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=400), [#244](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=244)에 연결됩니다.

## 계산 규약

- 오른손 회전, 고정 좌표축, 열벡터에 왼쪽에서 곱하는 `v' = Rv`입니다.
- 쿼터니언 순서는 `(w,x,y,z)`입니다. 단위축과 단위 쿼터니언은 입력을 정규화합니다.
- 각도 입력·CSV 각도 출력은 **라디안**입니다. 60도는 `1.0471975511965976`, 90도는 `1.5707963267948966`입니다.
- 내부 계산은 반올림하지 않습니다. CSV와 화면 출력만 소수 네 자리이며 작은 음수 영은 `0.0000`으로 표시합니다.
- 영 길이·비유한 수·잘못된 입력 개수·회전행렬이 아닌 입력은 종료 코드 2로 거부합니다. 행렬의 직교성·행렬식 허용 오차는 `1e-8`입니다. 네 자리로 반올림한 일반 행렬은 이 조건을 깨므로 역변환 검산에는 반올림 전 행렬을 넣습니다.

## 입력과 출력

모드를 명령행 인자로 고르고 표준 입력으로 숫자 한 줄을 줍니다. 기본 모드는 `axis`입니다.

| 모드 | 입력 |
|---|---|
| `axis` | `kx ky kz theta(rad)` |
| `quat` | `w x y z` |
| `matrix` | `r11 r12 r13 r21 r22 r23 r31 r32 r33` (행 우선) |

실행 폴더에 `R.csv`(3행×3열), `quat.csv`(1행×4열), `axis_angle.csv`(1행×5열)를 만듭니다. 마지막 파일은 `kx,ky,kz,theta,axis_defined` 순서입니다. 0도에서는 축이 비유일하므로 앞 세 칸은 0, 마지막 칸은 0입니다. 이때 `(0,0,0)`을 회전축으로 해석하지 않습니다. 그 외에는 마지막 칸이 1입니다.

역변환은 양의 대각합과 최대 대각 원소의 세 분기를 모두 구현합니다. 동률은 앞 성분 우선입니다. 복원 후 정규화하고 `w >= 0`을 택합니다. 정확한 180도 축의 부호는 최대 대각 성분이 양수인 쪽입니다. 180도 부근의 축 복원은 작은 사인으로 나누지 않고 안정적인 쿼터니언 경로를 사용합니다. `q`와 `-q`는 같은 회전으로 비교합니다.

## 설치·컴파일·실행

Python 계산부는 표준 라이브러리만 사용합니다. 시각화만 NumPy·Matplotlib이 필요합니다. 아래는 에피소드 폴더에서 실행하는 PowerShell 명령입니다. C/C++ 컴파일에는 GCC/MinGW 또는 호환 컴파일러가 필요합니다.

```powershell
python python/axis_angle_quat.py --selfcheck
'1 2 2 1.0471975511965976' | python python/axis_angle_quat.py axis
'0.7071 0 0.7071 0' | python python/axis_angle_quat.py quat
'-1 0 0 0 1 0 0 0 -1' | python python/axis_angle_quat.py matrix
gcc c/axis_angle_quat.c -o axis_c.exe -lm
g++ -std=c++11 cpp/axis_angle_quat.cpp -o axis_cpp.exe
'1 2 2 1.0471975511965976' | ./axis_c.exe axis
'1 2 2 1.0471975511965976' | ./axis_cpp.exe axis
```

실행할 때도 MinGW의 `bin`이 PATH에 있어야 런타임 DLL을 찾습니다. 각 언어를 서로 다른 작업 폴더에서 실행하면 CSV를 덮어쓰지 않고 비교할 수 있습니다. Python은 `--out <폴더>`도 지원합니다.

## 기대값과 검산

| 입력 | 기대 결과 |
|---|---|
| 축 `(0,0,1)`, 90도 | `[[0,-1,0],[1,0,0],[0,0,1]]` |
| 축 `(1,2,2)`, 60도 | `[[0.5556,-0.4662,0.6885],[0.6885,0.7222,-0.0665],[-0.4662,0.5109,0.7222]]` |
| 위 행렬에서 복원한 쿼터니언 | `(0.8660,0.1667,0.3333,0.3333)` |
| `(0.7071,0,0.7071,0)` | `[[0,0,1],[0,1,0],[-1,0,0]]` |
| `(1,1,1,1)` 정규화 | 축 `(1,1,1)/sqrt(3)`, 120도 |
| y축 180도 행렬 | `(0,0,1,0)` |

`tests/expected.csv`는 대표 입력별 기대값입니다. Python selfcheck는 30개 왕복과 잘못된 입력 3개를 실제 계산합니다. 제작 검증에서 별도로 50개 입력의 세 언어 CSV 일치와 15개 오류 입력 거부를 확인했습니다.

## 실험·흔한 실점

로드리게스의 `K²`는 원소별 제곱이 아니라 행렬곱입니다. 이를 생략한 비스듬한 축 60도 예제는 행렬식이 `1.75`입니다. 축 `(0,0,2)`를 정규화하지 않고 90도를 계산하면 `13`입니다. 단위 쿼터니언 전용 `1-2(y²+z²)` 원소식에 비단위 `(1,1,1,1)`을 그대로 넣은 오답의 행렬식은 `37`입니다. 이 수치는 일반 비단위 쿼터니언의 동차식과 구별해야 합니다.

반각을 생략하면 같은 입력 60도에서 실제 회전은 120도가 되고 `(1,0,0)`은 `(-0.3333,0.9107,-0.2440)`으로 갑니다. `+k` 둘레 60도와 `-k` 둘레 300도는 같은 회전입니다. `+k` 둘레 300도와 혼동하지 않습니다. #244의 정규화 전후 차이는 네 자리 반올림에 가려지지만 정규화 조건은 여전히 필요합니다. 부호를 모두 뒤집은 반대칭행렬은 길이·부피 검사를 통과하면서 방향만 반대로 돌리므로 요구 방향도 확인합니다.

## 그림과 프레임

```powershell
python -m pip install numpy matplotlib
python python/visualize.py --cone --out cone
python python/visualize.py --sweep --frames --out frames-sweep
python python/visualize.py --doublecover --frames --out frames-doublecover
g++ -std=c++11 cpp/plot_svg.cpp -o plot_svg.exe
./plot_svg.exe projection.svg
```

PNG는 960×540입니다. 양의 축 경로는 0→60도 정지→360도, 반대 축 경로는 0→300도입니다. 이중 덮개 그림은 양의 축 60도 기준축 세 개를 고정하고 반대 축 회전 결과를 겹칩니다. SVG는 수평면 투영이므로 높이 정보가 없습니다. 전체 자세는 3차원 프레임으로 확인합니다.

## 다음 편·관련 링크

해밀턴 곱·SLERP·측지 거리는 EP.17 범위이며 이 예제에는 구현하지 않았습니다.

- [강의 페이지](https://www.techrraforming.com/academy/cert-prac-axis-angle-quaternion)
- [실기 연습장](https://www.techrraforming.com/practical?cert=RobotSoftware)
- [Amatta Academy](https://www.youtube.com/@amatta-techrraforming)
