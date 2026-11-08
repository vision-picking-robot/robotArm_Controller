"""Proposed motion boundary v1. Not a deployed ROS/network API."""
from dataclasses import dataclass

from warehouse_motion.validation import pose6


def identifier(value, label):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ValueError(f'{label}: 1~128자의 식별자가 필요합니다')
    return value


@dataclass(frozen=True)
class PickRequest:
    request_id: str
    order_id: str
    product_id: str
    box_id: str
    slot_id: int
    grasp_posx: tuple
    frame_id: str = 'base'
    position_unit: str = 'mm'
    orientation: str = 'zyz_deg'
    simulation: bool = True

    def __post_init__(self):
        for name in ('request_id', 'order_id', 'product_id', 'box_id'):
            identifier(getattr(self, name), name)
        if type(self.slot_id) is not int or self.slot_id < 1:
            raise ValueError('slot_id는 양의 정수여야 합니다')
        if self.frame_id != 'base' or self.position_unit != 'mm' or self.orientation != 'zyz_deg':
            raise ValueError('요청 좌표는 Base / mm / 두산 ZYZ degree여야 합니다')
        if type(self.simulation) is not bool:
            raise ValueError('simulation은 bool이어야 합니다')
        object.__setattr__(self, 'grasp_posx', tuple(pose6(self.grasp_posx, 'grasp_posx')))

    def require_mock(self):
        if self.simulation is not True:
            raise ValueError('현재 실행기는 모의 요청만 처리합니다')
