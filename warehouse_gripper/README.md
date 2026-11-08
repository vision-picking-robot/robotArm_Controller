# RH-P12-RN-A 그리퍼 ROS 2 제어

두산 E0509 한 대를 연결한 `controller_manager`에 그리퍼 컨트롤러를 로드합니다.
컨트롤러는 두산 하드웨어 인터페이스의 기존 DRFL 연결을 공유하며, 플랜지 시리얼을 통해
Modbus RTU로 통신합니다. 팔 관절 인터페이스는 점유하지 않습니다.
여러 두산 팔이 같은 프로세스에 연결된 구성에서는 사용하지 않습니다.

## 빌드와 로드

기존 두산 드라이버가 빌드된 워크스페이스에서 실행합니다.

```bash
cd ~/ros2_ws
source install/setup.bash
colcon build --packages-select warehouse_gripper
source install/setup.bash
```

이 환경에서 기존 두산 real bringup을 시작한 뒤 다른 터미널에서 아래 명령을 실행합니다.
예시의 로봇 네임스페이스는 `dsr01`입니다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run controller_manager spawner warehouse_gripper \
  -c /dsr01/controller_manager \
  --param-file "$(ros2 pkg prefix --share warehouse_gripper)/config/gripper.yaml"
ros2 service call /dsr01/warehouse_gripper/status std_srvs/srv/Trigger '{}'
```

`status`는 위치(펄스), 전류 레지스터 값, 이동 플래그를 읽습니다. 상태 조회는 그리퍼의
위치·전류·토크 레지스터에 값을 쓰지 않습니다. 기본 포트 번호는 1이며 실물 포트와
일치하는지 확인해야 합니다. 포트 설정은 컨트롤러 구성 시 읽습니다.
시리얼 설정은 57,600 bps, 8N1, Modbus Slave ID 1입니다.

## 열기와 닫기

먼저 상태 조회에서 정상 응답을 확인합니다. 실물 그리퍼만 시험할 수 있는 상태에서
`enable_motion`을 켜고 서비스 한 개씩 호출합니다. 이동 명령은 로봇이 STANDBY일 때만
전송합니다.

```bash
ros2 param set /dsr01/warehouse_gripper enable_motion true
ros2 service call /dsr01/warehouse_gripper/open std_srvs/srv/Trigger '{}'
ros2 service call /dsr01/warehouse_gripper/status std_srvs/srv/Trigger '{}'
ros2 service call /dsr01/warehouse_gripper/close std_srvs/srv/Trigger '{}'
ros2 service call /dsr01/warehouse_gripper/status std_srvs/srv/Trigger '{}'
ros2 param set /dsr01/warehouse_gripper enable_motion false
```

기존 DART 소스에서 확인한 목표 위치 0(열기), 740(닫기), 목표 전류 400 mA를 사용합니다.
서비스 성공은 Modbus 쓰기 응답을 확인했다는 뜻입니다. 손가락 이동 완료와 상품 파지
성공은 위치·전류·이동 플래그 및 실물 상태로 따로 확인합니다. 응답이 끊긴 경우
이미 처리된 쓰기가 있을 수 있으므로 상태를 먼저 조회합니다.
`enable_motion=false`는 이후 서비스 명령을 차단하며 그리퍼 토크를 해제하지 않습니다.
