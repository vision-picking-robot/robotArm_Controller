"""Run one RFID-driven pick cycle through the warehouse motion services."""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from warehouse_interfaces.srv import MoveToPose, PickPlace, QueryRfidTag
from warehouse_motion.db.rfid_db_bridge import normalize_uid
from warehouse_motion.ros_calls import call_service, spin_node


class PickingCoordinator(Node):
    def __init__(self):
        super().__init__('picking_coordinator')
        self.uid_topic = self.declare_parameter('rfid_uid_topic', '/warehouse/rfid/uid').value
        self.rfid_timeout = self.declare_parameter('rfid_timeout_sec', 2.0).value
        self.motion_timeout = self.declare_parameter('motion_timeout_sec', 180.0).value
        if any(not math.isfinite(t) or t <= 0 for t in (self.rfid_timeout, self.motion_timeout)):
            raise ValueError('Timeouts must be finite and positive')

        self.io = Node('picking_coordinator_io', context=self.context)
        self.io_executor = SingleThreadedExecutor(context=self.context)
        self.io_executor.add_node(self.io)
        self.move = self.io.create_client(MoveToPose, '/warehouse/move_to_pose')
        self.lookup = self.io.create_client(QueryRfidTag, '/warehouse/query_rfid_tag')
        self.pick = self.io.create_client(PickPlace, '/warehouse/pick_place')
        self.requires_recovery = False
        self.cycle = self.create_service(Trigger, '/warehouse/run_pick_cycle', self.run_cycle)

    def read_uid(self):
        message = None

        def receive(msg):
            nonlocal message
            message = msg

        subscription = self.io.create_subscription(String, self.uid_topic, receive, 1)
        try:
            deadline = time.monotonic() + self.rfid_timeout
            while message is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('RFID UID timeout')
                self.io_executor.spin_once(timeout_sec=remaining)
            return normalize_uid(message.data)
        finally:
            self.io.destroy_subscription(subscription)

    def move_to(self, pose_name):
        result = call_service(
            self.io_executor, self.move, MoveToPose.Request(pose_name=pose_name), self.motion_timeout
        )
        if not result.success:
            raise RuntimeError(f'{pose_name}: {result.message}')

    def run_cycle(self, _request, response):
        if self.requires_recovery:
            response.message = 'Previous motion outcome requires operator review before restart'
            return response
        for client in (self.move, self.lookup, self.pick):
            if not client.wait_for_service(timeout_sec=2.0):
                response.message = f'Service unavailable: {client.srv_name}'
                return response
        try:
            self.move_to('home')
            self.move_to('label_scan_approach')
            self.move_to('label_scan')
            try:
                uid = self.read_uid()
                product = call_service(
                    self.io_executor, self.lookup, QueryRfidTag.Request(tag_uid=uid), 5.0
                )
                if not product.found:
                    raise ValueError(product.message)
            except (TimeoutError, ValueError, RuntimeError) as error:
                self.move_to('label_scan_retract')
                self.move_to('home')
                response.message = f'RFID failed, returned home: {error}'
                return response

            self.move_to('label_scan_retract')
            self.move_to('via_point')
            result = call_service(
                self.io_executor, self.pick,
                PickPlace.Request(product_name=product.yolo_class_name, box_slot=product.target_box_slot),
                self.motion_timeout,
            )
            if not result.success:
                raise RuntimeError(result.message)
            self.move_to('home')
            response.success = True
            response.message = f'{uid}: {result.message}; returned home'
        except (RuntimeError, TimeoutError, ValueError) as error:
            self.requires_recovery = True
            response.message = f'{error}; operator review required'
            self.get_logger().error(response.message)
        return response

    def destroy_node(self):
        self.io_executor.shutdown()
        self.io.destroy_node()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PickingCoordinator()
        spin_node(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
