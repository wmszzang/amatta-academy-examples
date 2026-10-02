# EP.21 프레임 트리·기구학 캘리브레이션·카메라 좌표

영상: 발행 후 연결 · [강의 노트](https://www.techrraforming.com/academy/cert-prac-frame-tree-calibration)

주어진 변환을 적용하는 문제(체인·핀홀·스테레오)와 관측으로 값을 추정하는 문제(관절 오프셋·강체 정합)를 구분합니다. 점 좌표와 변환은 열벡터 기준이며 `T_a_b`는 b 좌표를 a 좌표로 바꿉니다.

체인 예제는 몸체→팔 밑동 관계 `R=I, t=(0.1,0,0.2)`와 팔 밑동→카메라 관계 `R=R_bc, t=(0.2,0,0.2)`를 합칩니다. 아래 화살표는 부모·자식 연결을 뜻하며, 점을 변환할 때에는 `T_base_arm @ T_arm_cam`의 오른쪽부터 적용합니다. 합친 이동은 `(0.3,0,0.4)`입니다.

|입력|계산|기대 출력|
|---|---|---|
|카메라 점 (0.2,0,2), R=[[0,0,1],[-1,0,0],[0,-1,0]], t=(0.3,0,0.4)|회전 후 이동|p_base=(2.3000,-0.2000,0.4000), 역변환=(0.2000,0.0000,2.0000)|
|#283 Q=[(0,0),(1,0),(1,1),(0,1)], P=[(2,3),(2,4),(1,4),(1,3)]|중심화, H, SVD, 반사 보정, 이동 복원|90.0000 deg, t=(2.0000,3.0000), 네 점 잔차 0|
|#284 X=(1,2,5), R=I,t=(0.5,-0.5,1), f=500,c=(320,240)|Xc=(1.5,1.5,6), 깊이로 나눔|(445.0000,365.0000) px|
|#289 uL=370,uR=345,vL=240, f=500,B=0.1,c=(320,240)|시차 25, 깊이 2|(0.2000,0.0000,2.0000) m|
|길이 0.5m 링크 2개, 자세(30,45),(0,90),(60,-30)도, 참 오프셋(1,-0.5)도|잡음 없는 합성 측정, 최소제곱 1스텝|(0.9962,-0.4907)도, 위치 RMS 11.6300 → 0.0413mm|

## 설치·실행

Python 3와 C/C++ 컴파일러(Visual Studio 또는 MinGW)를 준비합니다. 이 폴더에서 실행합니다.

```sh
python -m pip install -r requirements.txt
python python/test_all.py
python python/frame_chain.py
python python/icp_step.py
python python/pinhole.py
python python/stereo.py
python python/calib_offset.py
gcc -std=c99 c/frame_chain.c -o frame_chain_c -lm
gcc -std=c99 c/pinhole.c -o pinhole_c -lm
gcc -std=c99 c/stereo.c -o stereo_c -lm
g++ -std=c++11 cpp/frame_chain.cpp -o frame_chain_cpp
g++ -std=c++11 cpp/icp_step.cpp -o icp_step_cpp
g++ -std=c++11 cpp/pinhole.cpp -o pinhole_cpp
g++ -std=c++11 cpp/stereo.cpp -o stereo_cpp
g++ -std=c++11 cpp/plot_svg.cpp -o plot_svg
```

Windows에서는 실행 파일 뒤 `.exe`를 붙이고 `./pinhole_c.exe`처럼 실행합니다. Visual Studio에서는 개발자 명령 프롬프트에서 `cl c/pinhole.c`, `cl /EHsc cpp/icp_step.cpp`를 사용합니다. CMake는 `cmake -S cpp -B out/build` 후 `cmake --build out/build`입니다. C의 Makefile은 `make -C c`로 빌드합니다.

## 그림과 실험

```sh
python python/visualize.py chain
python python/visualize.py icp --frames
python python/visualize.py calib
python python/visualize.py stereo
./plot_svg.exe
```

matplotlib 그림은 `out/`, C++ 직접 생성 그림은 실행 위치의 `result.svg`입니다. 점을 바꾸고 정합을 다시 실행하거나, 시차를 50·25·10·5·2로 바꾸면 깊이가 1·2·5·10·25m가 됨을 확인할 수 있습니다. 관절 보정은 잡음 없는 합성 자료이며 실제 장비 정확도 보장이 아닙니다. RMS는 세 자세의 위치 오차 길이를 제곱평균한 뒤 제곱근을 취합니다.

## 채점 형식과 흔한 실점

- #283: 각도 **도**, tx, ty 순서, 소수 네 자리. 중심 차이만 이동으로 쓰지 말고 `t=cP-RcQ`를 씁니다. C++의 2x2 SVD는 영 특이값이 있는 퇴화 입력을 거부합니다.
- #284: u,v 순서, 소수 네 자리. 깊이 나눗셈 또는 외부 이동항 누락에 주의합니다. 깊이 0 이하 입력은 오류입니다.
- #289: X,Y,Z 순서, 미터·소수 네 자리. 정렬 영상, 공통 주점과 fx=fy를 가정합니다. 시차 0 이하를 (0,0,0) 정상 좌표로 반환하지 않습니다. `stereo_c --invalid`, `pinhole_c --invalid`는 오류 메시지와 종료코드 2를 냅니다(C++ 동일).
- 카메라 optical 축은 오른쪽 x·아래 y·전방 z, 몸체 축은 전방 x·왼쪽 y·위 z입니다. 실제 ROS에서는 optical 프레임을 별도로 두는 관행과 예제 이름을 구분합니다.
- 정합은 대응점이 주어진 한 단계이며 ICP 전체 반복이나 여러 자세가 필요한 손–눈 보정과 같지 않습니다.
- 몸체 원점에서 거리 2.3431m의 점을 그 원점 중심으로 1도 회전할 때 최대 작은각 근사가 40.89mm입니다. 카메라 장착축 오차는 카메라 축까지 거리를 사용합니다.

## 연습장

- [#283 SVD 강체변환](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=283)
- [#284 핀홀 투영](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=284)
- [#289 스테레오](https://www.techrraforming.com/practical?cert=RobotSoftware&problem=289)
