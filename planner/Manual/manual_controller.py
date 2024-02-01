import keyboard
import numpy as np
from planner.plannerBase import PlannerBase

class ManualController(PlannerBase):
    def __init__(self):
        self.acc_status = None
        self.lane_change_status = None
        self.speed_limit = 40

    def init(self, scenario_info: dict):
        print("--------------------------------------------------------------Manual INIT--------------------------------------------------------------")

    def _pressed_key(self):
        up_or_down = None
        left_or_right = None

        if keyboard.is_pressed('up') and not keyboard.is_pressed('down'):
            up_or_down = 'up'
        if keyboard.is_pressed('down') and not keyboard.is_pressed('up'):
            up_or_down = 'down'

        if keyboard.is_pressed('left') and not keyboard.is_pressed('right'):
            left_or_right = 'left'
        if keyboard.is_pressed('right') and not keyboard.is_pressed('left'):
            left_or_right = 'right'

        return (up_or_down, left_or_right)
    
    def act(self, observation):
        acc = 0
        rotate = 0

        up_or_down, left_or_right = self._pressed_key()

        if up_or_down == 'up' and observation.vehicle_info['ego']['v'] < self.speed_limit:
            acc = 3
        if up_or_down == 'down' and observation.vehicle_info['ego']['v'] >= 0.2:
            acc = -3

        if left_or_right == 'left':
            rotate = 10 * np.pi / 180
        elif left_or_right == 'right':
            rotate = -10 * np.pi / 180
        
        return acc, rotate
    

# controller = ManualController()
# while True:
#     print(f"Key Pressed: {controller.act('1')}")
#     time.sleep(0.5)