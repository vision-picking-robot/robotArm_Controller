# robotArm_Controller

물류 창고 피킹 시나리오에서 **두산 E0509 협동로봇 + ROBOTIS RH-P12-RN 그리퍼**의
모션 제어를 담당하는 ROS 2 (Jazzy) 패키지 모음입니다.

> YOLO 인식과 RFID 리더기 판독은 외부 모듈에서 담당합니다. 이 저장소의 작업 조정 노드는 판독된 UID를 받아 단일 상품 피킹 순서를 관리합니다.
> 이 저장소는 단일 상품 피킹을 실행하는 **ROS 2 인터페이스**와 오프라인 검증 코드를 제공합니다.

---

실물 단일 상품 작업의 빌드·설정·실행 방법은 [RFID 단일 상품 피킹 운영](docs/operations.md)을 참고하세요.

## 로봇 연결 전 준비

실물 실행 전에는 장비별 자세와 상품별 파지 피드백을 확인하고 교시합니다.
로봇 없이 순서를 확인하려면 [오프라인 실행 방법](docs/offline-preparation.md)을 참고하세요.
요청 형식과 다음 순서는 [연결 초안](docs/integration-contract.md)에 있습니다.
D455를 이용한 물체 폭 추정과 자동 열림 폭은 추후 검토합니다.

## 목차
1. [전체 시나리오](#1-전체-시나리오)
2. [담당 범위](#2-담당-범위)
3. [개발 환경](#3-개발-환경)
4. [설계 결정](#4-설계-결정)
5. [바닥 높이 안전장치 (2중)](#5-바닥-높이-안전장치-2중)
6. [패키지 구조](#6-패키지-구조)
7. [서비스 인터페이스](#7-서비스-인터페이스)
8. [마일스톤](#8-마일스톤)
9. [안전 규칙](#9-안전-규칙)
10. [작업 규칙](#10-작업-규칙)
11. [알려진 이슈](#11-알려진-이슈)

---

## 1. 전체 시나리오

```
① 사람이 박스에 송장 + NFC 라벨 부착 후 지정 위치에 배치 (트리거)
        ↓
② 로봇이 그리퍼 옆 NFC 리더기로 박스 상단 라벨 스캔 → 주문 목록 획득
        ↓
③ 상품 구역 위 고정 카메라 + YOLOv8 로 주문 상품 인식
        ↓
④ 로봇이 상품을 위에서 집어 박스에 넣음 (주문 수량만큼 반복)
        ↓
⑤ 완료 시 홈 자세로 복귀
```

- **상품**: 곽과자, 페트 음료, 스팸/참치캔, 물티슈, 인형 (모두 바닥/테이블 위에 놓임)

## 2. 담당 범위

| 항목 | 내용 |
|---|---|
| TCP | 현재 TCP 확인 및 설정 지원 |
| 그리퍼 | 열기 / 닫기 / 현재 위치 읽기 제어 방법 확보 |
| 교시 | 직접 교시 좌표 기록 도구, 바닥 높이 측정 |
| 안전장치 | 바닥 아래로 내려가지 않도록 하는 소프트웨어 검사 + 펜던트 설정 가이드 |
| 좌표 변환 | 카메라 픽셀 → 로봇 base 좌표 (호모그래피) |
| Pick & Place | 상품 위 접근 → 하강 → 집기 → 상승 → 경유 → 박스 위 → 내려놓기 → 복귀 |
| 라벨 스캔 | 라벨 스캔 자세로 이동 (NFC 읽기 자체는 다른 팀원 담당) |

## 3. 개발 환경

| 항목 | 값 |
|---|---|
| OS / ROS | Ubuntu, ROS 2 **Jazzy** (`/opt/ros/jazzy`) |
| 워크스페이스 | `~/ros2_ws` (소스: `~/ros2_ws/src`) |
| 로봇 | 두산 **E0509**, 드라이버 [`doosan-robot2`](https://github.com/DoosanRobotics/doosan-robot2) (jazzy), 네임스페이스 `dsr01` |
| 그리퍼 | ROBOTIS **RH-P12-RN** (연결·제어 방식 조사 필요) |
| NFC 리더기 | 그리퍼 옆면 부착 (손끝 대비 돌출량 측정 필요) |
| 카메라 | 사용자 보유 Intel RealSense D455, 상품 구역 위 고정 예정 (**Eye-to-Hand**). 보정·폭 추정 작업은 추후 진행 |

### 드라이버 실행

```bash
# 실물
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real host:=<ROBOT_IP> port:=12345 model:=e0509

# 가상 (에뮬레이터) — 새 동작은 여기서 먼저 검증
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=virtual host:=127.0.0.1 model:=e0509
```

- 로봇 모드는 **자동**이어야 합니다: `/dsr01/system/get_robot_mode` → `1`

## 4. 설계 결정

### 로봇 제어
- `DSR_ROBOT2` 파이썬 API 사용 (`movej`, `movel`, `ref=DR_BASE/DR_TOOL`, `mod=DR_MV_MOD_REL` 등)
- 참고 예제: `doosan-robot2/dsr_example2/.../demo/dance_m1013.py`
- 기준 좌표계: **base**
- TCP: **그리퍼 두 손가락 끝 중앙**

### 잡기 방식
- 위에서 수직 접근 (**top-down**)
- 접근 자세 = 집기 자세에서 **z + 100 mm**
- 상품 회전 각도
  - 1차: 상품을 정해진 방향으로 놓는다고 가정
  - 2차: YOLO OBB 각도로 손목(rz) 회전

### 좌표 관리
- 좌표는 **직접 교시**로 따고 YAML에 저장 (`posx`, `posj` 둘 다 기록)
- **YOLO 연동 전**: `products.yaml`의 상품별 고정 좌표로 pick & place 완성
  - 실물 로봇: 실제로 상품을 놓고 **교시한 값만** 사용
  - 가상 모드 테스트: 임의 숫자 허용
- **YOLO 연동 후**: 검증된 Base 파지 좌표를 입력. 테이블 호모그래피를 높이가 다른 상품 윗면에 그대로 적용하지 않으며, 깊이·상품 높이를 반영한 변환을 검증한다.
- 코드는 **"상품 좌표를 어디서 받느냐"만 바꾸면** 되도록 설계 (고정값 ↔ YOLO 변환값)
- YOLO 촬영 시 로봇은 **카메라 시야 밖의 홈 자세**에 있어야 함

## 5. 바닥 높이 안전장치 (2중)

```
                 ┌──────────────────────────────┐
  이동 명령 ───▶ │ 1차: 소프트웨어 (safe_move)    │ ── 목표 z < 하한선 → 명령 차단 + 예외
                 └──────────────┬───────────────┘
                                ▼ 통과
                 ┌──────────────────────────────┐
                 │ 2차: 컨트롤러 (펜던트 안전 영역) │ ── 최후 방어선
                 └──────────────────────────────┘
```

### 바닥 높이 `floor_z` 측정
1. 승인된 측정 도구 또는 수동 교시 절차로 작업대 표면 높이를 측정 (자동 접촉 이동 금지)
2. 테이블 모서리 등 **최소 4곳** 측정
3. 측정값 차이가 **5 mm 이상**이면 보고 (테이블 기울기 확인)
4. 차이가 작으면 **가장 높은 값**을 `floor_z`로 사용

### 하한선 계산

| 항목 | 값 |
|---|---|
| 소프트웨어 하한선 | `floor_z + SAFETY_MARGIN` |
| `SAFETY_MARGIN` | 기본 **10 mm** (설정 파일에서 변경). NFC 리더기 등이 손끝보다 아래로 나와 있으면 **그 돌출량만큼 추가** |
| 펜던트 안전 설정 | 현장 담당자가 툴 형상·정지 거리·작업 영역과 컨트롤러 버전을 검토해 설정. 단순히 소프트웨어 값에서 5mm를 빼서 정하지 않음 |

### 상품 집는 높이
- 절대 z가 아니라 **바닥 기준 높이** `grasp_height_from_floor`로 `products.yaml`에 저장
- 실행 시 `z = floor_z + grasp_height_from_floor`
- 계산된 z가 하한선보다 낮으면 **그 상품은 실행하지 않고 보고**

### ⚠️ 한계
소프트웨어 검사는 **TCP 도착점만** 확인합니다.
이동 **경로**(특히 `movej`)나 그리퍼 옆 **NFC 리더기의 충돌**은 막지 못하므로
**펜던트 안전 영역 설정을 반드시 권장**합니다.

## 6. 패키지 구조

```text
warehouse_interfaces/              # MoveToPose, PickPlace, QueryRfidTag 서비스 정의
warehouse_gripper/                 # 두산 플랜지 시리얼 기반 그리퍼 컨트롤러
warehouse_motion/
├── config/                        # 운영 자세와 상품 설정
├── launch/
│   └── picking.launch.py          # 작업 조정, 모션 서버, RFID DB 노드 실행
├── warehouse_motion/              # 설치되는 운영 Python 모듈
│   ├── picking_coordinator.py     # RFID 판독 대기, 작업 순서, 홈 복귀
│   ├── motion_server.py           # 집기·운반·적치, 자세 이동
│   ├── safe_move.py               # 목표 자세와 속도 검증
│   ├── ros_motion_backend.py      # 두산 ROS 2 이동 서비스 호출
│   └── db/                        # RFID UID와 상품·슬롯 매핑
├── offline_picking/               # 모의 실행 코드와 합성 설정
└── test/                          # 자동 검증용 단위 테스트
docs/                              # 운영, 오프라인 검증, 연동 문서
```

`warehouse_motion`을 빌드하면 `picking_coordinator`, `motion_server`, `rfid_db_node` 세 실행 명령이 설치됩니다.
오프라인 코드는 소스 디렉터리에서 Python 모듈로 실행합니다. [오프라인 실행 방법](docs/offline-preparation.md)을 참고하세요.

### `safe_move` 규칙
- 로봇을 움직이는 모든 코드는 `movel`/`movej`를 **직접 호출하지 않고** 이 래퍼만 사용
- `movel`
  - 절대 좌표 → 목표 z 바로 검사
  - 상대 이동 → 현재 posx를 읽어 도착 위치 계산 후 검사
- `movej` → fkin(정기구학) 서비스로 목표 관절값을 posx로 변환해 z 검사 (없으면 대안 검토)
- 하한선 위반 시 **명령을 보내지 않고** 예외 발생, 목표값과 하한선을 로그로 출력

## 7. 서비스 인터페이스

단일 상품 운영 경로는 `/warehouse/run_pick_cycle`, `/warehouse/move_to_pose`, `/warehouse/pick_place`를 사용합니다.
별도 오프라인 경로의 Python `PickRequest`는 요청/주문/상품/박스/슬롯 ID와 Base 파지 자세를 검증합니다.
픽셀 좌표를 모션에 직접 전달하지 않습니다. [연결 초안과 예시](docs/integration-contract.md)를 참고하세요.

긴 피킹 작업의 진행 보고·취소는 Action을 우선 검토하고, 자세 조회 등 짧은 요청은 Service로 분리합니다.
현재 가짜 실행기의 취소 검사는 실물 이동 도중 정지를 보장하지 않습니다.

### 실패 처리
UID 판독 실패 시 리더기 접근점으로 후퇴한 뒤 홈으로 복귀합니다.
이동·파지 결과가 불확실하면 오류를 반환하고 다음 작업을 막습니다. 작업자가 파지 상태와 경로를 확인한 뒤 복구합니다.

## 8. 마일스톤

각 단계 완료 후 확인을 받고 다음으로 진행합니다.

| # | 단계 | 주요 내용 | 상태 |
|---|---|---|---|
| 1 | 환경 확인 | 현재 TCP 이름·오프셋 확인 방법, 카메라 토픽, 필요한 dsr 서비스 목록 (이동, 현재 좌표, fkin 등) | 🟨 소스 조사 완료, 드라이버 실행 후 재확인 필요 → [조사 결과](#마일스톤-1-환경-확인-결과) |
| 2 | 그리퍼 제어 | 플랜지 시리얼 Modbus RTU 컨트롤러 | 구현·빌드 완료, 실물 통신 검증 필요 |
| 3 | 패키지 생성 | `warehouse_motion` + `poses.yaml`, `products.yaml` | 구현 완료 |
| 4 | 교시 도우미 | 엔터 → 현재 posx/posj 읽기 → 이름 입력 → YAML 저장. 바닥 측정 모드(최대/최소/차이), NFC 돌출 측정 안내 | ⬜ |
| 5 | `safe_move` 래퍼 | z 하한선·유한값·속도 검사 + 단위 테스트 | 오프라인 검증 완료 |
| 6 | 펜던트 안전 영역 가이드 | 하한값 계산, 설정 절차, 승인 필요 여부, 저속 검증 절차, 설정 불가 시 주의사항 | ⬜ |
| 7 | 고정 좌표 pick & place | 교시 좌표 기반 단일 상품 운영 흐름 | 구현 완료, 실물 검증 필요 |
| 8 | 비전 좌표·물체 폭 | D455 깊이·보정과 자동 열림 폭 | 추후 고려사항 |
| 9 | ROS 인터페이스 | 단일 상품 운영 서비스 | 서비스 구현 완료, Action 추후 검토 |
| 10 | 실패 처리 | UID 실패 시 홈 복귀, 이동·파지 오류 시 다음 작업 차단 | 모의 호출 검증, 실물 검증 필요 |

### 마일스톤 1: 환경 확인 결과

> `doosan-robot2` 소스(`dsr_msgs2/srv`, `dsr_controller2.cpp`)에서 확인한 내용입니다.
> 드라이버 실행 후 `ros2 service list` / `ros2 interface show` 로 재확인합니다.

필요한 서비스 (모두 `/dsr01/` 아래, ⚠️ = 로봇 상태를 바꿈):

| 용도 | 서비스 | 요청 → 응답 |
|---|---|---|
| TCP 이름 | `tcp/get_current_tcp` | → `info` (이름만) |
| TCP 변경 ⚠️ | `tcp/set_current_tcp`, `tcp/config_create_tcp` | 이름 / 이름 + `pos[6]` |
| 현재 좌표 | `aux_control/get_current_posx` | `ref` (0=base) → `task_pos_info[0][0:6]` |
| | `aux_control/get_current_posj` | → `pos[6]` |
| | `aux_control/get_current_tool_flange_posx` | `ref` → 플랜지 `pos[6]` |
| 정기구학 | `motion/fkin` | `pos[6]`(posj), `ref` → `conv_posx[6]` |
| 역기구학 | `motion/ikin` | posx, `sol_space`, `ref` → `conv_posj[6]` |
| 이동 ⚠️ | `motion/move_joint`, `motion/move_line`, `motion/move_stop`, `motion/move_wait` | |
| 상태 | `system/get_robot_mode`, `system/get_robot_state`, `system/get_last_alarm` | |
| DRL ⚠️ | `drl/drl_start`, `drl/drl_stop`, `drl/get_drl_state` | `robot_system`, `code` |
| 툴 I/O | `io/get_tool_digital_input`, `io/set_tool_digital_output` ⚠️ | |

- ✅ `motion/fkin` 존재 → `safe_move` 에서 `movej` 목표의 z 검사 가능
- ⚠️ TCP **오프셋**을 읽는 서비스는 없음 → 펜던트 TCP 설정 화면에서 확인.
  TCP·플랜지 자세로 보조 검증하려면 회전을 포함한 좌표 변환이 필요하며 단순 위치 차이를 TCP 오프셋으로 사용하지 않음.
- `modbus/*` 는 Modbus **TCP**(IP·포트) 전용 → 손목 RS-485 장치에는 해당 없음
- 카메라: 개발 PC USB 에 RealSense 미검출, `realsense2_camera` 미설치 → 연결 PC 확인 필요

### 마일스톤 2: 그리퍼 제어

`warehouse_gripper`는 두산 드라이버의 DRFL 연결을 공유하고 툴 플랜지 시리얼로 Modbus RTU를 사용합니다.
`status`, `open`, `close` 서비스와 설정 방법은 [그리퍼 실행 안내](warehouse_gripper/README.md)에 있습니다.
실물에서 플랜지 포트, 상태 응답, 단독 열기·닫기, 상품별 파지 피드백을 확인해야 합니다.

## 9. 안전 규칙

- ✅ **조회 명령은 바로 실행 가능**: `ros2 service list`, `ros2 topic list`, `ros2 interface show`, `get_current_*`, `get_robot_mode` 등
- ⚠️ **로봇/그리퍼를 움직이는 명령은 실행 전 반드시 확인**: 이동 서비스, 그리퍼 열기/닫기, 모드·TCP 변경, 동작 스크립트 실행
- 새 동작은 가능하면 `mode:=virtual`에서 먼저 검증
- 실물 첫 실행 속도는 **vel/acc 30 이하**
- **TCP 변경**은 팀 전체 좌표에 영향 → 명시적 확인 후 진행
- 예시 좌표값을 실제 로봇 좌표처럼 쓰지 않음. **실물에서는 교시값만 사용**
- **`floor_z` 측정 전**에는 실물에서 상품 구역으로 내려가는 동작 금지
- `safe_move` 래퍼를 **우회하는 이동 코드 금지**

## 10. 작업 규칙

- 서비스 이름과 필드는 추측하지 말고 `ros2 service list`, `ros2 interface show`로 확인 후 작성
- 빌드는 항상 `~/ros2_ws`에서 (`src` 안에서 빌드 금지)
  ```bash
  cd ~/ros2_ws
  colcon build --packages-select <패키지>
  source install/setup.bash
  ```
- 기존 `doosan-robot2` 패키지는 수정하지 않고, 코드는 이 저장소 패키지에 둠
- 파일 삭제(`rm`)는 실행 전 확인

## 11. 알려진 이슈

| 이슈 | 대응 |
|---|---|
| 그리퍼 서비스가 보이지 않음 | 두산 bringup 이후 `warehouse_gripper` 컨트롤러 로드 상태 확인 |
| `~/ros2_ws/src` 안에 잘못 생성된 `build/install/log` 폴더가 있을 수 있음 | 발견 시 정리 |
| `--symlink-install` 빌드 시 `PackageNotFoundError: No package metadata` | 해당 패키지 build/install 폴더 삭제 후 옵션 없이 재빌드 |
| `ROS_LOCALHOST_ONLY` / 여러 PC 연결 | 카메라·주문·로봇이 다른 PC면 DDS·ROS_DOMAIN_ID·네트워크 설정 확인 |
| 노드 콜백 안에서 서비스를 동기 호출하면 데드락 가능 | `MultiThreadedExecutor` + 콜백 그룹 분리 또는 비동기 호출로 설계 |
| 가상 모드(에뮬레이터 3.0.1)에서 `move_spiral` 호출 시 응답 없이 컨트롤러가 멈춤 | pick & place 에서는 사용하지 않음. 멈추면 bringup 재시작 |
