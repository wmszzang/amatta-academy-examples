# EP.8 DH 파라미터와 행렬 순기구학

문제 #237의 표준 DH 행렬을 직접 곱해 팔 끝 좌표를 구하고, 문제 #223처럼 둘째 관절각을 0도부터 90도까지 15도 간격으로 스윕합니다.

| 검산 항목 | 기대값 |
|---|---|
| #237 끝점 | `(2.1203, 2.4489, 0.0000)` |
| #223 첫/마지막 점 | `(3.0311, 1.7500)` / `(0.9821, 2.2990)` |
| 궤적 점 수 | 7 |
| 팔꿈치-끝점 거리 | `1.5000` |

## 실행

```powershell
python python/dh_fk.py
python python/visualize.py --frames
gcc c/dh_fk.c -o dh-fk-c.exe -lm; .\dh-fk-c.exe
g++ -std=c++11 cpp/dh_fk.cpp -o dh-fk-cpp.exe; .\dh-fk-cpp.exe
g++ -std=c++11 cpp/plot_svg.cpp -o plot-svg.exe; .\plot-svg.exe
```

`visualize.py --frames`는 정적 PNG와 7개 프레임을 만듭니다. `plot_svg.cpp`는 같은 궤적을 외부 그래프 라이브러리 없이 SVG로 작성합니다.

## 흔한 실점

- 20도 간격 반복은 `90도` 끝점을 빠뜨립니다. 횟수를 먼저 계산하고 `0..n`을 포함합니다.
- `0.5236`과 `0.7854`는 라디안입니다. 도 단위로 다시 해석하지 않습니다.
- 좌표만 내지 말고 지문이 요구한 궤적 그래프도 함께 제출합니다.

관련 강의: https://www.techrraforming.com/academy/cert-prac-dh-fk-matrix
