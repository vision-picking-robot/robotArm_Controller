"""ROS/robot-free demonstration. No hardware backend is imported or constructed."""
import argparse
import json
import os

from .config_loader import default_config_dir, floor_limit, get_product, load_yaml
from .gripper import GripController, GripFeedback, get_grip_profile
from .pick_place import execute_pick_place
from .safe_move import SafeMover


class SimClock:
    def __init__(self): self.value = 0.0
    def now(self): return self.value
    def sleep(self, seconds): self.value += seconds


class MockGripper:
    simulated = True

    def __init__(self, clock, profile, outcome='ok'):
        self.clock, self.profile, self.outcome = clock, profile, outcome
        self.width = profile.open_width_mm
        self.current = 0.0
        self.moving = False
        self.commands = []
        self.halted = False
        self.closing = False

    def set_opening(self, width, speed, current_limit):
        self.commands.append((width, speed, current_limit))
        self.closing = width < self.width
        self.moving = False
        if not self.closing:
            self.width, self.current = width, 0.0
        elif self.outcome == 'empty':
            self.width, self.current = width, 0.0
        elif self.outcome == 'jam':
            self.width = self.profile.open_width_mm - 2
            self.current = self.profile.contact_current_ma * 1.1
        else:
            self.width = (self.profile.min_width_mm + self.profile.max_width_mm) / 2
            self.current = self.profile.contact_current_ma * 1.1
            if self.outcome == 'overcurrent': self.current = current_limit * 2
            if self.outcome == 'timeout': self.moving = True

    def read(self):
        age = 5 if self.outcome == 'stale' and self.closing else 0
        return GripFeedback(self.width, self.current, self.moving, self.clock.now() - age)

    def halt(self): self.halted = True


class MockMotion:
    simulated = True
    def __init__(self): self.commands = []; self.stopped = False
    def movel(self, pos, vel, acc, ref, mod, radius):
        self.commands.append(list(pos))
        return 0
    def stop(self): self.stopped = True


def mock_plan(grasp, slot=1):
    """Synthetic Base/mm poses; never use these as measured robot poses."""
    if type(slot) is not int or not 1 <= slot <= 5:
        raise ValueError('모의 슬롯은 1~5입니다')
    approach = list(grasp); approach[2] += 100
    x, y = 300 + 60 * ((slot - 1) % 3), -200 + 60 * ((slot - 1) // 3)
    return dict(approach=approach, grasp=grasp, lift=approach,
                via=[350, 0, 500, 0, 180, 0],
                place_approach=[x, y, 400, 0, 180, 0],
                place=[x, y, 200, 0, 180, 0], retract=[x, y, 400, 0, 180, 0])


def run_mock_order(names, config_dir=None):
    directory = config_dir or default_config_dir()
    poses_cfg = load_yaml(os.path.join(directory, 'poses.yaml'))
    products = load_yaml(os.path.join(directory, 'products.yaml'))
    grips = load_yaml(os.path.join(directory, 'gripper_profiles.yaml'))
    limit = floor_limit(poses_cfg, 'virtual')
    motion, clock = MockMotion(), SimClock()
    mover = SafeMover(limit.z_min, motion, 30, 30)
    # Synthetic scene coordinates only. No measured poses are read here.
    home = [350, 0, 500, 0, 180, 0]
    mover.movel(home, 10, 10, ref=0)
    result = []
    for name in names:
        target = get_product(products, name, 'virtual', limit)
        profile = get_grip_profile(grips, name, 'mock')
        grip = GripController(MockGripper(clock, profile), clock.now, clock.sleep)
        grasp = [target.x, target.y, target.grasp_z, 0, 180, target.rz]
        plan = mock_plan(grasp)
        steps = execute_pick_place(mover, grip, profile, plan, motion.stop, vel=10, acc=10)
        result.append(dict(product=name, steps=steps, simulated=True))
    mover.movel(home, 10, 10, ref=0)
    return dict(mode='mock', cycles=len(result), results=result,
                motion_commands=len(motion.commands), hardware_connected=False)


def main():
    parser = argparse.ArgumentParser(description='가짜 장비만 사용하는 피킹 순서 검사')
    parser.add_argument('--products', nargs='+', default=['snack_box', 'pet_bottle', 'wet_tissue'])
    parser.add_argument('--config-dir')
    args = parser.parse_args()
    print(json.dumps(run_mock_order(args.products, args.config_dir), ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
