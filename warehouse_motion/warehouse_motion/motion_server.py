"""Serve taught robot poses and execute a single physical pick/place."""
import os

import rclpy
from dsr_msgs2.srv import GetRobotSystem
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from warehouse_interfaces.srv import MoveToPose, PickPlace
from warehouse_motion import config_loader as c
from warehouse_motion.gripper_feedback import wait_for_feedback
from warehouse_motion.ros_calls import call_service, spin_node
from warehouse_motion.ros_motion_backend import RosMotionBackend
from warehouse_motion.safe_move import DR_BASE, MotionError, SafeMover, SafetyLimitError
from warehouse_motion.validation import pose6


class MotionServer(Node):
    def __init__(self):
        super().__init__('warehouse_motion_server')
        self.mode = self.declare_parameter('mode', 'real').value
        robot_name = self.declare_parameter('robot_name', 'dsr01').value.strip('/')
        config_dir = self.declare_parameter('config_dir', '').value or c.default_config_dir()
        self.poses_cfg = c.load_yaml(os.path.join(config_dir, 'poses.yaml'))
        self.products_cfg = c.load_yaml(os.path.join(config_dir, 'products.yaml'))
        self.floor_limit = c.floor_limit(self.poses_cfg, self.mode)
        motion = self.poses_cfg.get('motion') or {}
        self.max_vel = motion.get('max_vel', 30.0)
        self.max_acc = motion.get('max_acc', 30.0)

        self.io = Node('warehouse_motion_io', context=self.context)
        self.io_executor = SingleThreadedExecutor(context=self.context)
        self.io_executor.add_node(self.io)
        self.mover = SafeMover(
            self.floor_limit.z_min, RosMotionBackend(self.io, self.io_executor, robot_name),
            self.max_vel, self.max_acc, self.get_logger(),
        )
        self.robot_system = self.io.create_client(
            GetRobotSystem, f'/{robot_name}/system/get_robot_system'
        )
        gripper_prefix = self.declare_parameter(
            'gripper_service_prefix', f'/{robot_name}/warehouse_gripper'
        ).value.rstrip('/')
        self.gripper = {
            name: self.io.create_client(Trigger, f'{gripper_prefix}/{name}')
            for name in ('status', 'open', 'close')
        }
        self.create_service(MoveToPose, '/warehouse/move_to_pose', self.handle_move_to_pose)
        self.create_service(PickPlace, '/warehouse/pick_place', self.handle_pick_place)

    def check_robot_system(self):
        result = call_service(self.io_executor, self.robot_system, GetRobotSystem.Request(), 3.0)
        if not result.success or result.robot_system != {'real': 0, 'virtual': 1}[self.mode]:
            raise MotionError(f'Connected robot does not match configured mode {self.mode}')

    def move_line(self, posx):
        self.mover.movel(posx, vel=self.max_vel, acc=self.max_acc, ref=DR_BASE)

    def move_to_pose(self, name):
        if name in ('label_scan', 'label_scan_retract'):
            scan = c.get_pose(self.poses_cfg, 'label_scan', self.mode)['posx']
            if name == 'label_scan_retract':
                approach = c.get_pose(self.poses_cfg, 'label_scan_approach', self.mode)['posx']
                scan = [*scan[:2], approach[2], *scan[3:]]
            self.move_line(scan)
            return

        pose = c.get_pose(self.poses_cfg, name, self.mode)
        if pose.get('posj') is not None:
            self.mover.movej(pose['posj'], vel=self.max_vel, acc=self.max_acc)
        else:
            self.move_line(pose['posx'])

    def handle_move_to_pose(self, request, response):
        try:
            self.check_robot_system()
            self.move_to_pose(request.pose_name)
            response.success = True
            response.message = f'Moved to {request.pose_name}'
        except (c.ConfigError, SafetyLimitError, MotionError, RuntimeError, TimeoutError,
                ValueError, TypeError) as error:
            response.message = str(error)
        return response

    def gripper_call(self, command, timeout=12.0):
        result = call_service(self.io_executor, self.gripper[command], Trigger.Request(), timeout)
        if not result.success:
            raise MotionError(f'Gripper {command}: {result.message}')
        return result.message

    def wait_for_gripper(self, condition, description):
        wait_for_feedback(
            lambda remaining: self.gripper_call('status', timeout=remaining),
            condition, description,
        )

    def grasp_feedback(self, product_name):
        feedback = self.products_cfg['products'][product_name]['real'].get('grasp_feedback') or {}
        low, high, current = (
            feedback.get(key) for key in ('min_position_pulses', 'max_position_pulses', 'min_current')
        )
        if not all(type(value) is int for value in (low, high, current)):
            raise c.ConfigError(f'{product_name}: calibrated grasp_feedback is required')
        if not 0 < low <= high < 740 or current <= 0:
            raise c.ConfigError(f'{product_name}: invalid grasp_feedback range')
        return low, high, current

    def pick_place(self, request):
        if self.mode != 'real':
            raise MotionError('Physical PickPlace requires real mode')
        product = c.get_product(
            self.products_cfg, request.product_name, self.mode, self.floor_limit
        )
        slot = c.get_box_slot(self.poses_cfg, request.box_slot, self.mode)['posx']
        home = c.get_pose(self.poses_cfg, 'home', self.mode)['posx']
        via = c.get_pose(self.poses_cfg, 'via_point', self.mode)['posx']
        if any(pose is None for pose in (slot, home, via)):
            raise c.ConfigError('PickPlace requires taught Cartesian home, via and slot poses')

        low, high, min_current = self.grasp_feedback(request.product_name)

        def grasped(position, current):
            return low <= position <= high and current >= min_current

        def opened(position, _current):
            return abs(position) <= 10

        home_z = home[2]
        carry_z = home_z + 150.0
        place_z = product.grasp_z + 10.0
        if home_z <= max(product.grasp_z, place_z):
            raise c.ConfigError('Home Z must clear pick and place Z')

        angle = request.angle if request.angle != 0.0 else product.rz
        pick_orientation = [0.0, 180.0, angle]
        approach = [product.x, product.y, home_z, *pick_orientation]
        pick = [product.x, product.y, product.grasp_z, *pick_orientation]
        lift = [product.x, product.y, carry_z, *pick_orientation]
        carry_via = [*via[:2], carry_z, *pick_orientation]
        carry_box = [*slot[:2], carry_z, *slot[3:]]
        place = [*slot[:2], place_z, *slot[3:]]
        retreat = [*slot[:2], home_z, *slot[3:]]

        # Check every Cartesian target before opening the gripper or moving the arm.
        for pose in (approach, pick, lift, carry_via, carry_box, place, retreat, via):
            pose6(pose, 'pick/place pose')
            if pose[2] < self.floor_limit.z_min:
                raise SafetyLimitError('Pick/place target below Z limit')

        self.gripper_call('status')
        self.move_line(approach)
        self.gripper_call('open')
        self.wait_for_gripper(opened, 'open')
        self.move_line(pick)
        self.gripper_call('close')
        self.wait_for_gripper(grasped, 'grasp')
        self.move_line(lift)
        self.wait_for_gripper(grasped, 'grasp after lift')
        self.move_line(carry_via)
        self.move_line(carry_box)
        self.move_line(place)
        self.gripper_call('open')
        self.wait_for_gripper(opened, 'release position')
        self.move_line(retreat)
        self.move_line(via)

    def handle_pick_place(self, request, response):
        try:
            self.check_robot_system()
            self.pick_place(request)
            response.success = True
            response.message = 'Pick/place motion and gripper feedback completed'
        except (c.ConfigError, SafetyLimitError, MotionError, RuntimeError, TimeoutError,
                ValueError, TypeError) as error:
            response.message = str(error)
            self.get_logger().error(f'Pick/place stopped: {error}')
        return response

    def destroy_node(self):
        self.io_executor.shutdown()
        self.io.destroy_node()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionServer()
        spin_node(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
