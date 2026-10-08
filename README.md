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
| 12 | 작업공간과 조작성 — 닿아도 탈락하는 이유 | [`ep12-workspace-manipulability/`](ep12-workspace-manipulability/) | [YouTube](https://youtu.be/QQ_1Bz3xvc0) |
| 13 | 수치 역기구학과 정역학 — 뉴턴-랩슨 · DLS · τ=JᵀF | [`ep13-numeric-ik-statics/`](ep13-numeric-ik-statics/) | [YouTube](https://youtu.be/bc2OHYpzl6U) |
| 14 | 해석해냐 수치해냐, 속도냐 힘이냐 — 역기구학·자코비안 끝내기 | [`ep14-ik-jacobian-wrapup/`](ep14-ik-jacobian-wrapup/) | [YouTube](https://youtu.be/l2jh9PbPmzo) |
| 15 | 3차원 자세를 각도 셋으로 — 오일러각 ZYX와 짐벌락 | [`ep15-euler-gimbal/`](ep15-euler-gimbal/) | [YouTube](https://youtu.be/7qo6uNENz-o) |
| 16 | 축 하나와 각도 하나로 — 축-각도·로드리게스·쿼터니언 | [`ep16-axis-angle-quaternion/`](ep16-axis-angle-quaternion/) | [YouTube](https://youtu.be/63EIwqBGO4Q) |
| 17 | 두 자세를 곱하고 잇는다 — 해밀턴 곱·SLERP·측지 거리 | [`ep17-quaternion-slerp/`](ep17-quaternion-slerp/) | [YouTube](https://youtu.be/gH4PF2M9ya4) |
| 18 | 이 지문은 어떤 회전 표현을 쓰라는 말인가 — 3D 회전 끝내기 | [`ep18-rotation-wrapup/`](ep18-rotation-wrapup/) | [YouTube](https://youtu.be/q8B9u2OVFHM) |

| 19 | 관절공간 vs 직교공간 — 직선·원호와 매 점 역기구학 | [`ep19-cartesian-path-ik/`](ep19-cartesian-path-ik/) | [영상](https://youtu.be/q_cMPYHZuGA) |

| 20 | 바퀴 로봇 기구학 — 차동구동·메카넘·아커만·바이시클 | [`ep20-mobile-kinematics/`](ep20-mobile-kinematics/) | [YouTube](https://youtu.be/o_wsRncPKvI) |

| 21 | 프레임 트리·기구학 캘리브레이션·카메라 좌표 | [`ep21-frame-tree-calibration/`](ep21-frame-tree-calibration/) | [YouTube](https://youtu.be/RO7qEnu4w-A) · [강의 페이지](https://www.techrraforming.com/academy/cert-prac-frame-tree-calibration) |

| 22 | 기구학 지문 한 줄에서 풀이 고르기 — 단원 종합 | [`ep22-kinematics-wrapup/`](ep22-kinematics-wrapup/) | [YouTube](https://youtu.be/peDViaXbfMg) · [강의 페이지](https://www.techrraforming.com/academy/cert-prac-kinematics-wrapup) |

| 23 | 격자 길찾기와 안전거리 — 다익스트라·A* | [`ep23-path-planning/`](ep23-path-planning/) | [YouTube](https://youtu.be/qPPd85kKdp0) · [강의 페이지](https://www.techrraforming.com/academy/cert-prac-path-planning) |

| 24 | RRT·DWA로 길 찾기와 장애물 회피 | [`ep24-sampling-planning/`](ep24-sampling-planning/) | [YouTube](https://youtu.be/i3GhAJKAkJA) · [강의 페이지](https://www.techrraforming.com/academy/cert-prac-sampling-planning) |
| 25 | 경로계획 끝내기 — 지문으로 풀이 고르기 | [`ep25-path-wrapup/`](ep25-path-wrapup/) | [YouTube](https://youtu.be/ciQo10IPdDc) · [강의 페이지](https://www.techrraforming.com/academy/cert-prac-path-wrapup) |
| 26 | PID 제어와 안티와인드업 | [`ep26-pid/`](ep26-pid/) | [강의 페이지](https://www.techrraforming.com/academy/cert-prac-pid) |

## 함께 보기

- 🤖 실기 연습장 (코딩 문제 99제 · AI 채점 · 기대 그래프 제공): <https://www.techrraforming.com/practical?cert=RobotSoftware>
- 🏠 Techrraforming: <https://www.techrraforming.com>

## 라이선스

MIT — 학습·시험 준비 목적으로 자유롭게 사용하세요.
