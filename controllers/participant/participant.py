"""
Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).
"""

from controller import Robot
import sys
sys.path.append('..')
# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera

'''
Progress:
1. Fatima added
2. docker updated
'''


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 0.1
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 200  # 8000 ms / 40 ms
    ATTACK_DISTANCE_THRESHOLD = 0.2

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        self.counter = 0
        self.opponent_detected = False
        self.attacking = False
        self.attack_counter = 0

    def run(self):
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            self.gait_manager.update_theta()
            if 0.3 < t < 2:
                self.start_sequence()
            elif t > 2:
                self.fall_detector.check()
                if not self.opponent_detected:
                    self.detect_opponent()
                if self.opponent_detected:
                    if not self.attacking:
                        self.evade_or_attack()
                    else:
                        self.attack()

    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)

    def detect_opponent(self):
        """Locate the opponent in the image and set the opponent_detected flag."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            self.opponent_detected = False
        else:
            self.opponent_detected = True

    def evade_or_attack(self):
        """Decide whether to evade or attack the opponent."""
        normalized_x = self._get_normalized_opponent_x()
        if abs(normalized_x) < self.ATTACK_DISTANCE_THRESHOLD:
            self.attacking = True
        else:
            if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
                self.heading_angle = - self.heading_angle
                self.counter = 0
            self.counter += 1
            self.gait_manager.command_to_motors(desired_radius=self.SMALLEST_TURNING_RADIUS / normalized_x, heading_angle=self.heading_angle)

    def attack(self):
        """Attack the opponent by moving towards it and performing a punch."""
        normalized_x = self._get_normalized_opponent_x()
        self.gait_manager.command_to_motors(desired_radius=self.SMALLEST_TURNING_RADIUS / normalized_x, heading_angle=0)
        if self.attack_counter == 0:
            self.gait_manager.punch()
        self.attack_counter += 1
        if self.attack_counter > self.gait_manager.PUNCH_TIME:
            self.attacking = False
            self.attack_counter = 0

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1


# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()