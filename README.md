# Amatta Academy — 실기 예제 답안 모음

[Amatta Academy 유튜브 채널](https://www.youtube.com/@amatta-techrraforming)의
**자격증 · 로봇소프트웨어개발기사 · 실기** 강의 시리즈에서 다룬 문제들의 **실행 가능한 예제 답안**입니다.

- 언어는 실기 시험에서 공식적으로 사용 가능한 **C · C++ · Python** 세 가지만 제공합니다.
  (시험장 제공 환경: Windows + Visual Studio · PyCharm · Excel)
- 에피소드마다 폴더 하나씩, 폴더 안 `README.md`에 **설치부터 실행·시각화까지** 순서대로 적혀 있습니다.
- 계산 프로그램은 어떤 언어로 실행해도 같은 CSV를 만들고,
  그래프는 공용 파이썬 스크립트(`visualize.py`) 또는 C++ SVG 스크립트(`plot_svg.cpp`)로 그립니다 — 영상 속 애니메이션도 `visualize.py`의 산출물입니다.

## 에피소드

| EP | 주제 | 폴더 | 영상 |
|---|---|---|---|
| 2 | 가속–등속–감속, 사다리꼴 속도 프로파일 | [`ep02-trapezoid-profile/`](ep02-trapezoid-profile/) | <https://youtu.be/Qwos-UAw1Kg> |
| 3 | 부드러운 궤적 — 3차·5차 다항식과 S-커브 | [`ep03-polynomial-traj/`](ep03-polynomial-traj/) | <https://youtu.be/3y5KqFdbla4> |
| 4 | 지문 읽고 계획 고르기 — 사다리꼴 · 다항식 · S-커브 끝내기 | [`ep04-motion-wrapup/`](ep04-motion-wrapup/) | <https://youtu.be/fGxWcXUkPFw> |
| 5 | 좌표는 어디서 읽는가 — 프레임 · 2D 회전 · 강체 변환 | [`ep05-frames-2d-rotation/`](ep05-frames-2d-rotation/) | [YouTube](https://youtu.be/Ba1QtkpZPS4) |
| 6 | 회전과 이동을 행렬 하나로 — 동차변환 체인·역변환·4×4 | [`ep06-homogeneous-chain/`](ep06-homogeneous-chain/) | [YouTube](https://youtu.be/aHfGnSfYwoo) |
| 7 | 관절각으로 팔 끝 좌표 구하기 — 2·3링크 순기구학 | [`ep07-fk-2link-3link/`](ep07-fk-2link-3link/) | [YouTube](https://youtu.be/ghgZjtKZz60) |
| 8 | DH 파라미터와 행렬 순기구학 — 팔 끝 좌표·관절각 스윕 | [`ep08-dh-fk-matrix/`](ep08-dh-fk-matrix/) | [YouTube](https://youtu.be/wDLIkh4Rk58) |
| 9 | 지문을 보고 변환이냐 순기구학이냐 — 좌표·FK 끝내기 | [`ep09-transform-fk-wrapup/`](ep09-transform-fk-wrapup/) | [YouTube](https://youtu.be/qdr_7sLRnVU) |

| 10 | 해석적 역기구학 — 두 해·도달 불가·해 선택 | [`ep10-ik-analytic/`](ep10-ik-analytic/) | [YouTube](https://youtu.be/xrqNRSIT354) |
| 11 | 자코비안과 특이점 — 관절속도에서 손끝 속도로 | [`ep11-jacobian-singularity/`](ep11-jacobian-singularity/) | [영상 보기](https://youtu.be/dXPricg-s_k) |

## 함께 보기

- 🤖 실기 연습장 (코딩 문제 99제 · AI 채점 · 기대 그래프 제공): <https://www.techrraforming.com/practical?cert=RobotSoftware>
- 🏠 Techrraforming: <https://www.techrraforming.com>

## 라이선스

MIT — 학습·시험 준비 목적으로 자유롭게 사용하세요.
