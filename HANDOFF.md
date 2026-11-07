# 작업 지시서 (HANDOFF)

> 세션이 끊겼을 때 이어서 작업하기 위한 문서입니다. **단계가 끝날 때마다 갱신**합니다.
> 새 세션에서는 이 파일과 `README.md`를 먼저 읽고 "다음 할 일"부터 진행하세요.
> 전체 요구사항·안전 규칙은 `README.md`에 있습니다 (특히 9. 안전 규칙).

마지막 갱신: 2026-11-07 — 마일스톤 3 완료, 5 진행 전

---

## 이번 세션에서 진행하기로 한 순서
1. [x] 마일스톤 1·2 조사 결과를 README에 반영 → push (`767eb97`)
2. [x] 마일스톤 3: `warehouse_motion` 패키지 뼈대 + `config/poses.yaml`, `config/products.yaml`
3. [ ] 마일스톤 5: `safe_move` 래퍼 + 로봇 없이 도는 단위 테스트

## 다음 할 일
- 마일스톤 5: `warehouse_motion/warehouse_motion/safe_move.py` + `test/test_safe_move.py`
  - 설계: `SafeMover(z_min, backend, max_vel, max_acc)`. backend 는 DSR_ROBOT2 래핑(지연 import) 또는 테스트용 가짜
  - movel: ref 필수(DR_BASE/DR_TOOL), ABS+BASE 는 목표 z 직접 검사, REL 은 현재 posx + 이동량
    (TOOL 상대는 현재 자세 ZYZ 회전행렬로 변환), ABS+TOOL·WORLD·사용자좌표는 거부
  - movej: ABS/REL(현재 posj 더함) → fkin(DR_BASE) → z 검사. fkin 실패 시 차단(fail-closed)
  - 위반 시 명령 미전송 + SafetyLimitError + 목표/하한선 로그
- 사용자 확인 대기: `src/robotArm_Controller/{build,install,log}` 삭제 (실수로 src 안에서 빌드됨, .gitignore 처리됨)

## 현재 상태 요약
- 저장소 위치: `~/ros2_ws/src/robotArm_Controller` (origin: github.com/vision-picking-robot/robotArm_Controller, `main`)
- GitHub push 인증: 사용자가 로그인 완료 (HTTPS)
- 로봇 드라이버: **아직 실행 안 함** (실물 bringup 명령은 README 3장)
- 그리퍼 ROS 모델(`rh_p12_rn_a_description`) 빌드됨. RViz 확인용: `ros2 launch ~/ros2_ws/e0509_gripper_urdf/view.launch.py`
- Claude Code 권한: `~/ros2_ws/.claude/settings.local.json` 에 `colcon build` 허용

## 만든 것
- `warehouse_motion/` (ament_python): `package.xml`, `setup.py`(config/*.yaml 설치), `config_loader.py`
  - `floor_limit(poses, mode)` → `FloorLimit.z_min = floor_z + safety_margin + nfc_protrusion`
    real 모드에서 floor_z / nfc_protrusion 이 null 이면 ConfigError (= 이동 불가)
  - `get_pose / get_box_slot / get_product`: real 모드에서 `taught: false` 면 ConfigError
  - `get_product` 는 grasp_z = floor_z + grasp_height_from_floor 가 z_min 미만이면 ConfigError
- `config/poses.yaml`: floor(floor_z, samples, safety_margin=10, nfc_protrusion), virtual(floor_z=100), motion(approach 100, max vel/acc 30), poses(home, label_scan_approach, label_scan, via_point), box_slots 1~4 — 전부 미교시(null)
- `config/products.yaml`: snack_box, pet_bottle, can, wet_tissue, doll — real 은 null, virtual 은 임의값
- 빌드·검증: `cd ~/ros2_ws && colcon build --packages-select warehouse_motion && source install/setup.bash`

## 확인된 사실 (드라이버 없이 소스에서 확인 → 드라이버 실행 후 `ros2 service list`로 재확인 필요)
- 서비스 (모두 `/dsr01/` 접두):
  - TCP: `tcp/get_current_tcp`(이름만 반환), `tcp/set_current_tcp`, `tcp/config_create_tcp`
  - 좌표: `aux_control/get_current_posx`(ref → task_pos_info[0][0:6]), `aux_control/get_current_posj`, `aux_control/get_current_tool_flange_posx`
  - 기구학: `motion/fkin`(pos[6], ref → conv_posx[6]), `motion/ikin`
  - 이동: `motion/move_joint`, `motion/move_line`, `motion/move_stop`, `motion/move_wait`
  - 상태: `system/get_robot_mode`, `system/get_robot_state`, `system/get_last_alarm`
  - DRL: `drl/drl_start`(robot_system, code), `drl/drl_stop`, `drl/get_drl_state`
  - `modbus/*` 는 Modbus **TCP** 전용 (손목 RS-485 아님). `SerialSendData.srv`는 정의만 있고 서비스 미등록
- DSR_ROBOT2 상수: `DR_BASE=0, DR_TOOL=1, DR_WORLD=2, DR_MV_MOD_ABS=0, DR_MV_MOD_REL=1`
- `movel(pos, vel, acc, ..., ref=None, mod=...)` — ref 생략 시 전역 좌표계 사용 → safe_move 에서는 ref 필수
- `get_current_posx(ref)` → `(posx, sol)`, `fkin(posj, ref)` → posx
- TCP 오프셋 조회 서비스 없음 → 펜던트 확인, 또는 `get_current_posx` vs `get_current_tool_flange_posx` 차이로 검증
- 그리퍼: `/dev/ttyUSB*` 없음 → PC 직결 아님. 유력 경로 = `drl/drl_start` 로 DRL 손목 시리얼 코드 실행
  (DRL 함수명·그리퍼 프로토콜/baud 는 미확인)
- 에뮬레이터 3.0.1 에서 `move_spiral` 호출 시 컨트롤러 멈춤 (사용 금지)

## 사용자에게 받아야 할 정보 (미해결)
- [ ] 펜던트에 그리퍼 플러그인/스킬/예제 DRL 이 있는지
- [ ] 펜던트로 그리퍼를 열고 닫아본 적 있는지, 방법
- [ ] 그리퍼 모델명 (`RH-P12-RN` vs `RH-P12-RN(A)`)
- [ ] 그리퍼 케이블이 연결된 손목 커넥터 단자
- [ ] 펜던트의 TCP 이름·오프셋 값
- [ ] 카메라가 어느 PC 에 연결되는지 (이 PC USB 에는 안 보임, realsense 패키지 미설치)

## 주의 (새 세션도 반드시 지킬 것)
- 로봇·그리퍼를 움직이는 명령, 모드/TCP 변경, 동작 스크립트 실행은 **사용자 확인 후**에만
- 조회 명령(`service list`, `get_current_*`, `get_robot_mode` 등)은 바로 실행 가능
- 빌드는 `~/ros2_ws` 에서 `colcon build --packages-select <pkg>` (워크스페이스 전체 빌드 시 RH-P12-RN-A 의 dynamixel 의존 패키지가 실패할 수 있음)
- Bash 도구는 직전 cd 위치를 유지함 → 빌드는 항상 `cd ~/ros2_ws && colcon build ...` 로
- `pkill -f <패턴>` 은 자기 셸까지 죽일 수 있음 → `pkill -f "[p]attern"` 형태 사용
- 파일 삭제(`rm`)는 사용자 확인 후
