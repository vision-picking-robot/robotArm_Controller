# RFID 단일 상품 피킹 운영

ROS 2 Jazzy에서 두산 드라이버와 작업공간을 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to warehouse_motion
source install/setup.bash
```

1. 실제 E0509 한 대의 드라이버를 실행하고 자동 모드, TCP, 작업 영역을 확인합니다.
2. DART 작업을 종료한 뒤 [그리퍼 컨트롤러](../warehouse_gripper/README.md)를 별도 spawner로 로드하고 `active` 상태를 확인합니다. `picking.launch.py`만으로는 그리퍼가 로드되지 않습니다. `status` 조회 성공 후 `enable_motion`을 켜서 단독 열기·닫기를 확인합니다. DART에서 검증한 57,600bps·ID 1·400mA·0~740펄스를 사용합니다.
3. 운영 PC에 `poses.yaml`과 `products.yaml`을 준비합니다. 장비마다 홈, 리더기 접근·판독, 경유지, 슬롯, 상품 좌표 및 바닥 높이를 교시합니다. 리더기 접근점과 라벨 판독점은 같은 X·Y에서 수직 접근·후퇴할 수 있어야 합니다. 작업대·박스·주변 상품과 실제 이동 경로의 간섭을 확인합니다.
4. 상품별 실제 파지 시험에서 얻은 위치·전류 판정 범위를 `products.<상품>.real.grasp_feedback`에 설정합니다. 필수 정수 필드는 `min_position_pulses`, `max_position_pulses`, `min_current`입니다. `min_current` 단위는 mA이며 전류 절댓값을 비교합니다. 범위가 없으면 상품 집기를 시작하지 않습니다. 열림 완료는 목표 0에서 ±10펄스이고 이동이 끝난 상태로 판정합니다. 열림·파지 조건은 0.1초 간격으로 연속 2회 확인합니다. `Moving Status`는 별도 필드이며 정지 판정에 합산하지 않습니다.
5. RFID 리더가 판독 중인 UID를 `std_msgs/msg/String`으로 발행하도록 연결합니다. 발행 토픽은 아래에서 설정할 수 있습니다. 판독 자세에서 구독이 시작된 뒤 발행된 UID를 사용하며, 빈 문자열은 판독 실패입니다. 발행자는 이전 작업의 UID를 재전송하지 않아야 합니다. RFID DB에는 UID별 상품과 슬롯 매핑이 필요합니다.

```bash
ros2 launch warehouse_motion picking.launch.py \
  robot_name:=dsr01 \
  config_dir:=/path/to/taught-config \
  db_path:=/path/to/rfid.db \
  rfid_uid_topic:=/warehouse/rfid/uid
```

실물과 설정을 확인한 후 그리퍼 `enable_motion`을 활성화하고 작업 1건을 요청합니다.

```bash
ros2 service call /warehouse/run_pick_cycle std_srvs/srv/Trigger '{}'
```

운영 클라이언트는 실행 응답을 받은 뒤 다음 요청을 보내고, 응답 유실 시 자동 재시도하지 않습니다.
작업 중에는 다른 클라이언트에서 모션·그리퍼 서비스나 펜던트 이동 명령을 보내지 않습니다.
UID 판독 실패 시 라벨 위치에서 수직 후퇴 후 홈으로 복귀합니다. 모션이나 파지 결과가
불확실하면 자동 복귀·자동 해제를 중단합니다. 작업자가 상태를 확인하고 복구한 뒤 작업
조정기를 재시작해야 다음 작업을 받을 수 있습니다.

현재 실행 경로는 교시된 상품 좌표와 단일 상품 UID 매핑을 사용합니다. 실제 YOLO 좌표·폭
입력, 다품목 주문 반복, 상품이 박스 안에 놓였는지 확인하는 센서와 주문 DB 갱신은 별도
연동이 필요합니다. 작업 성공은 로봇 모션과 그리퍼 피드백 완료를 뜻합니다.
