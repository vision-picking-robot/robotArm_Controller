"""모션 서비스 서버 노드 (motion_server).

외부(오케스트레이터, YOLO, RFID)로부터 다음 요청을 받아 안전하게 처리한다:
  1. /warehouse/move_to_pose (warehouse_interfaces/srv/MoveToPose):
     - poses.yaml 에 등록된 자세(home, label_scan_approach, label_scan 등)로 이동
  2. /warehouse/pick_place (warehouse_interfaces/srv/PickPlace):
     - YOLO 검출 좌표와 슬롯 번호로 안전하게 Pick & Place 수행
"""
import os
import rclpy
from rclpy.node import Node

from warehouse_interfaces.srv import MoveToPose, PickPlace
from warehouse_motion import config_loader as c
from warehouse_motion.safe_move import (
    SafeMover, DsrBackend, SafetyLimitError, MotionError, DR_BASE, DR_MV_MOD_ABS
)


class MotionServer(Node):
    def __init__(self):
        super().__init__('warehouse_motion_server')

        # 파라미터 선언 (기본값 real)
        self.declare_parameter('mode', 'real')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.get_logger().info(f'Starting MotionServer in [{self.mode}] mode')

        # 설정 파일 로드
        config_dir = c.default_config_dir()
        self.poses_cfg = c.load_yaml(os.path.join(config_dir, 'poses.yaml'))
        self.products_cfg = c.load_yaml(os.path.join(config_dir, 'products.yaml'))

        # 안전 하한선 계산
        self.floor_limit = c.floor_limit(self.poses_cfg, self.mode)
        self.get_logger().info(
            f'Floor Limit: floor_z={self.floor_limit.floor_z:.1f}mm, '
            f'z_min={self.floor_limit.z_min:.1f}mm'
        )

        # 모션 및 속도 파라미터
        motion_cfg = self.poses_cfg.get('motion', {})
        self.approach_offset_z = float(motion_cfg.get('approach_offset_z', 100.0))
        self.max_vel = float(motion_cfg.get('max_vel', 30.0))
        self.max_acc = float(motion_cfg.get('max_acc', 30.0))

        # SafeMover 초기화
        # 실제 DsrBackend 연결 (노드 기반)
        try:
            backend = DsrBackend()
        except Exception as e:
            self.get_logger().warn(f'DsrBackend init failed ({e}), using mock/direct service fallback')
            backend = None

        self.mover = SafeMover(
            z_min=self.floor_limit.z_min,
            backend=backend,
            max_vel=self.max_vel,
            max_acc=self.max_acc,
            logger=self.get_logger()
        )

        # 서비스 등록
        self.srv_move_to_pose = self.create_service(
            MoveToPose, '/warehouse/move_to_pose', self.handle_move_to_pose
        )
        self.srv_pick_place = self.create_service(
            PickPlace, '/warehouse/pick_place', self.handle_pick_place
        )

        self.get_logger().info('Ready for MoveToPose and PickPlace services.')

    def handle_move_to_pose(self, request, response):
        pose_name = request.pose_name
        self.get_logger().info(f'[MoveToPose] Request: {pose_name}')

        try:
            pose = c.get_pose(self.poses_cfg, pose_name, self.mode)
            posj = pose.get('posj')
            posx = pose.get('posx')

            # posj 가 있으면 관절 공간 이동 (경로 케이블 꼬임 방지)
            if posj is not None:
                self.mover.movej(posj, vel=self.max_vel, acc=self.max_acc)
            elif posx is not None:
                self.mover.movel(posx, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)
            else:
                response.success = False
                response.message = f'자세 {pose_name} 의 좌표가 비어 있습니다'
                return response

            response.success = True
            response.message = f'Successfully moved to {pose_name}'
        except (c.ConfigError, SafetyLimitError, MotionError) as e:
            self.get_logger().error(f'[MoveToPose] Error: {e}')
            response.success = False
            response.message = str(e)

        return response

    def handle_pick_place(self, request, response):
        prod_name = request.product_name
        box_slot = request.box_slot
        self.get_logger().info(f'[PickPlace] Request: {prod_name} -> Box Slot {box_slot}')

        try:
            # 1. 대상 상품 및 박스 슬롯 유효성 검사
            target = c.get_product(self.products_cfg, prod_name, self.mode, self.floor_limit)
            slot_pose = c.get_box_slot(self.poses_cfg, box_slot, self.mode)

            # 2. 목표 피킹 좌표 결정 (YOLO 픽셀/캘리브레이션 또는 사전 정의 좌표)
            # request.u, request.v 가 0 이상이면 캘리브레이션 변환값 사용 가능
            # 현재 1차 버전은 캘리브레이션 전 등록 좌표 사용
            pick_x = target.x
            pick_y = target.y
            pick_z = target.grasp_z

            # 짧은 쪽 파지 각도 반영
            rz = request.angle if request.angle != 0.0 else target.rz

            # 3. Pick 시퀀스 실행
            # 3-1. 접근 (Approach)
            app_pos = [pick_x, pick_y, pick_z + self.approach_offset_z, 0.0, 180.0, rz]
            self.mover.movel(app_pos, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)

            # 3-2. 하강 및 파지 (Grasp)
            pick_pos = [pick_x, pick_y, pick_z, 0.0, 180.0, rz]
            self.mover.movel(pick_pos, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)
            # TODO: 그리퍼 닫기 (gripper.close())

            # 3-3. 상승 (Retract)
            self.mover.movel(app_pos, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)

            # 4. 경유 또는 박스 슬롯 이동 (Place)
            slot_posx = slot_pose['posx']
            slot_posj = slot_pose.get('posj')
            slot_app_pos = [slot_posx[0], slot_posx[1], slot_posx[2] + self.approach_offset_z] + slot_posx[3:6]

            # 박스 상공 접근
            self.mover.movel(slot_app_pos, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)
            # 박스 하강
            self.mover.movel(slot_posx, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)
            # TODO: 그리퍼 열기 (gripper.open())
            # 박스 상공 복귀
            self.mover.movel(slot_app_pos, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)

            response.success = True
            response.message = f'Successfully picked {prod_name} and placed into box {box_slot}'

        except (c.ConfigError, SafetyLimitError, MotionError) as e:
            self.get_logger().error(f'[PickPlace] Error: {e}')
            # 안전 자세(Home) 복귀 시도
            try:
                home_pose = c.get_pose(self.poses_cfg, 'home', self.mode)
                if home_pose.get('posj'):
                    self.mover.movej(home_pose['posj'], vel=self.max_vel, acc=self.max_acc)
            except Exception:
                pass
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
