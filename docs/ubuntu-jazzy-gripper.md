# Ubuntu 24.04 / ROS 2 Jazzy 그리퍼 시험

나은 PC의 Ubuntu 24.04·ROS 2 Jazzy에서 기존 두산 드라이버의 연결을 공유합니다.
`warehouse_gripper`는 ROS 컨트롤러 플러그인이며, 별도 로봇 TCP 연결을 만들지 않습니다.
아래 작업은 그리퍼 단독 시험입니다. 자동 피킹은 이후 교시·파지 피드백 확인 후 진행합니다.

## 1. 코드 갱신과 빌드

기존 드라이버·작업을 정상 종료한 상태에서, 두산 드라이버가 빌드된 작업공간을 사용합니다.
아래 저장소 경로는 실제 clone 위치에 맞춥니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src/robotArm_Controller
git pull --ff-only origin main

cd ~/ros2_ws
source install/setup.bash
rosdep install --from-paths src/robotArm_Controller --ignore-src --rosdistro jazzy -y
colcon build --packages-select warehouse_interfaces warehouse_gripper warehouse_motion
source install/setup.bash
```

`source install/setup.bash`는 기존 `dsr_common2`, `dsr_hardware2`, `dsr_msgs2` 빌드 결과를
사용합니다. 먼저 `ros2 pkg prefix dsr_hardware2`가 성공해야 합니다. 다른 배포판의
워크스페이스를 겹쳐 source하지 않습니다. SDK 헤더·라이브러리는 설치된 Jazzy 두산
드라이버의 `dsr_common2::drfl`을 함께 사용합니다.

팀원 저장소의 Mac 작업자는 GL010117에 고정돼 있습니다. 이 패키지는 기존 Jazzy
드라이버의 SDK를 임의로 바꾸지 않습니다. E0509의 컨트롤러 버전과 현재 드라이버가
연결 가능한지 먼저 확인합니다. 구형/신형 시리얼 함수 서명 호환과 실제 펌웨어 호환은
다른 문제입니다. 연결·상태 조회 실패 상태에서 열기·닫기를 반복하지 않습니다.

빌드 후 이미 실행 중인 드라이버에는 새 공유 라이브러리가 적용되지 않습니다.
기존에 연결이 확인된 두산 real bringup을 다시 시작하고, 다른 터미널에서 진행합니다.
DART Task Writer 작업과 팀원 SDK 작업자는 종료해 제어권·시리얼 충돌을 피합니다.

## 2. 컨트롤러 로드와 상태 읽기

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch warehouse_gripper gripper.launch.py
```

기본 controller manager는 `/dsr01/controller_manager`입니다. 다른 네임스페이스라면
`controller_manager:=/다른이름/controller_manager`를 붙입니다. 로드·활성화만 수행하며
기본 설정의 `enable_motion`은 false입니다. 두산 드라이버나 팔 동작을 시작하지 않습니다.
이미 `warehouse_gripper`가 active라면 spawner를 중복 실행하지 않습니다.

다른 터미널에서 같은 두 setup 파일을 source하고 조회합니다.

```bash
ros2 control list_controllers -c /dsr01/controller_manager
ros2 param get /dsr01/warehouse_gripper enable_motion
ros2 service call /dsr01/system/get_robot_state dsr_msgs2/srv/GetRobotState '{}'
ros2 service call /dsr01/system/get_robot_mode dsr_msgs2/srv/GetRobotMode '{}'
ros2 service call /dsr01/warehouse_gripper/status std_srvs/srv/Trigger '{}'
```

`warehouse_gripper`가 active이고 `status`가 success: true인지 확인합니다.
상태 읽기는 위치·전류·토크를 쓰지 않습니다. 가상 응답 예시는 다음과 같습니다.

```text
position_pulses=0 current=0 moving=0 moving_status=49 input_voltage_tenths=241
```

전압 값 241은 24.1V를 뜻합니다. 예시이며 실물 측정 결과가 아닙니다.
57,600bps·8N1·ID 1을 사용하고, 포트 기본값은 1입니다.
시리얼 개방 후 500ms를 기다리고 첫 전압(40297) 읽기 응답을 최대 1초 기다립니다.
분할·빈 수신은 같은 요청의 수신을 이어갑니다. 전압 읽기가 타임아웃된 경우에만
해당 읽기를 최대 3회 시도합니다. CRC·장치 오류와 구동 쓰기는 재시도하지 않습니다.

## 3. 그리퍼 단독 열기·닫기

그리퍼 주변을 확인하고 로봇이 AUTO·STANDBY인 상태에서 한 명의 작업자가 진행합니다.
팔을 움직이지 않고 그리퍼만 작동합니다. 아래 명령은 하나씩 실행하고 실제 상태를 확인합니다.

```bash
ros2 param set /dsr01/warehouse_gripper enable_motion true
ros2 service call /dsr01/warehouse_gripper/open std_srvs/srv/Trigger '{}'
ros2 service call /dsr01/warehouse_gripper/status std_srvs/srv/Trigger '{}'
```

열림을 확인한 뒤 닫기를 따로 실행합니다.

```bash
ros2 service call /dsr01/warehouse_gripper/close std_srvs/srv/Trigger '{}'
ros2 service call /dsr01/warehouse_gripper/status std_srvs/srv/Trigger '{}'
ros2 param set /dsr01/warehouse_gripper enable_motion false
```

이번 패키지는 사용자가 DART에서 확인한 열기 0·닫기 740·목표 전류 400mA를 유지합니다.
팀원 UI의 닫기 700과 구분합니다. 400mA를 상품별 적정 파지력으로 간주하지 않습니다.
서비스의 success는 명령 응답 확인입니다. 실제 이동 완료·물체 파지는 위치와 전류로
확인합니다. 결과가 불확실하면 자동 재전송·자동 개방·자동 복귀하지 않습니다.

## 4. 오류 확인

| 메시지 | 확인 사항 |
| --- | --- |
| Service unavailable / 서비스 없음 | setup source, manager 경로, 컨트롤러 로드·active 여부 |
| Motion blocked | enable_motion, AUTO·STANDBY, DART 작업 종료 |
| Could not open | 다른 클라이언트의 시리얼 점유, 포트, SDK/펌웨어 호환성 |
| Modbus response timeout | 표시된 레지스터·수신 바이트 수 기록, 전원·배선·ID·포트 확인 |
| Invalid Modbus RTU reply | CRC·ID·길이 오류, 다른 시리얼 요청과 충돌 여부 |
| Hardware Error Status | 장치 오류 확인 후 복구. 반복 닫기로 해결하지 않음 |
| undefined symbol / 라이브러리 미발견 | 같은 Jazzy 작업공간에서 두산 드라이버와 플러그인을 함께 빌드·source |

문의 시 명령 응답, `echo "$ROS_DISTRO"`, `uname -m`, `ros2 control list_controllers` 결과와
두산 드라이버 버전을 함께 확인합니다. SDK 바이너리·인증정보·실제 DB를 저장소에 올리지 않습니다.

## 5. 로봇 없이 검사

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select warehouse_gripper warehouse_motion --return-code-on-test-failure
colcon test-result --verbose
```

GitHub Actions의 `Ubuntu 24.04 / ROS 2 Jazzy`는 Ubuntu에서 Python/C++ 모의 장치 검사,
고정된 공식 Jazzy 드라이버 기반 빌드, 공유 라이브러리 링크와 launch 인자 로드를 확인합니다.
CI는 실제 로봇에 연결하지 않으며 실물 통신·장비별 SDK/펌웨어 호환성을 검증하지 않습니다.
