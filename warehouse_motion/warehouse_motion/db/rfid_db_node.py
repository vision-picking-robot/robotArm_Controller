"""RFID DB 서비스 노드 (rfid_db_node).

RFID 스캐너가 태그를 읽어 /warehouse/query_rfid_tag 서비스를 호출하면
DB(product_tag_mappings)에서 조회하여 다음 정보를 응답한다:
  - product_name (한글명)
  - yolo_class_name (YOLO가 인식할 이름, 예: snack_box, can 등)
  - target_class_id
  - target_box_slot (목적지 박스 슬롯 1~4)
"""
import rclpy
from rclpy.node import Node

from warehouse_interfaces.srv import QueryRfidTag
from warehouse_motion.db.rfid_db_bridge import RfidDatabaseBridge


class RfidDbNode(Node):
    def __init__(self):
        super().__init__('rfid_db_node')

        self.declare_parameter('db_path', '')
        param_db_path = self.get_parameter('db_path').get_parameter_value().string_value
        db_path = param_db_path if param_db_path else None

        self.bridge = RfidDatabaseBridge(db_path=db_path)
        self.get_logger().info(f'RfidDatabaseBridge initialized with DB: {self.bridge.db_path}')

        self.srv = self.create_service(
            QueryRfidTag,
            '/warehouse/query_rfid_tag',
            self.handle_query_tag
        )
        self.get_logger().info('Service [/warehouse/query_rfid_tag] ready.')

    def handle_query_tag(self, request, response):
        uid = request.tag_uid
        self.get_logger().info(f'[QueryRfidTag] Received UID: {uid}')

        try:
            prod = self.bridge.get_product_by_uid(uid)
            if prod:
                response.found = True
                response.product_name = prod.product_name
                response.yolo_class_name = prod.yolo_class_name
                response.target_class_id = prod.target_class_id
                response.target_box_slot = prod.box_slot
                response.message = f'Found product: {prod.product_name} ({prod.yolo_class_name}) -> Box Slot {prod.box_slot}'
                self.get_logger().info(f'[QueryRfidTag] {response.message}')
            else:
                response.found = False
                response.message = f'Tag UID {uid} not found in database'
                self.get_logger().warn(f'[QueryRfidTag] {response.message}')
        except Exception as e:
            response.found = False
            response.message = f'Error querying DB: {str(e)}'
            self.get_logger().error(f'[QueryRfidTag] Exception: {e}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RfidDbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
