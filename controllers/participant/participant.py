# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).
"""

from controller import Robot, Motion
import sys
sys.path.append('..')
# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera
from utils.current_motion_manager import CurrentMotionManager
import numpy as np
import cv2
import os

'''
Progress:
1. Fatima added
2. docker updated
'''


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 0.2
    # SMALLEST_TURNING_RADIUS = 1.0
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 200  # 8000 ms / 40 ms

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.camera2 = Camera(self, 'CameraBottom')
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.current_motion = CurrentMotionManager()
        self.heading_angle = 3.14 / 2
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0
        # load motion files
        self.motions = {
            'SideStepLeftLoop': Motion('../motions/SideStepLeftLoop.motion'),
            'SideStepRightLoop': Motion('../motions/SideStepRightLoop.motion'),
            'SideStepLeft': Motion('../motions/SideStepLeft.motion'),
            'SideStepRight': Motion('../motions/SideStepRight.motion'),
            'TurnRight': Motion('../motions/TurnRight20.motion'),
            'TurnLeft': Motion('../motions/TurnLeft20.motion'),
            'TurnLeft180': Motion('../motions/TurnLeft180.motion'),
            'TurnLeft60': Motion('../motions/TurnLeft60.motion'),
            'TurnRight60': Motion('../motions/TurnRight60.motion'),
            'TurnLeft20': Motion('../motions/TurnLeft20.motion'),
            'TurnRight20': Motion('../motions/TurnRight20.motion'),
            'TurnRight40': Motion('../motions/TurnRight40.motion'),
            'TurnLeft40': Motion('../motions/TurnLeft40.motion'),
            'TaiChi': Motion('../motions/TaiChi.motion'),
            'TaiChi_fast': Motion('../motions/TaiChi_fast.motion'),
            'Shoot': Motion('../motions/Shoot.motion'),
            'Backwards': Motion('../motions/Backwards.motion'),
            'Forwards': Motion('../motions/Forwards.motion'),
        }


    def run(self):
        self.running = 0
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            self.gait_manager.update_theta()

            # Imgage analysis start
            # if 1:
            threshold = 0.02
            # threshold = 0.3
            img = self.camera.get_image()
            # img = self.camera2.get_image()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            l, w = img.shape   
            img[0:int(l*0.75), :] = 0
            img[img < 180] = 0
            img[img >= 180] = 1
            white_px = img.sum()
            total_px = np.prod(img.shape)
            ratio = white_px / total_px

            left_img = img[:, :int(w/2)]
            right_img = img[:, int(w/2):]
            left_white_px = left_img.sum()
            right_white_px = right_img.sum()
            left_ratio = left_white_px / np.prod(left_img.shape)
            right_ratio = right_white_px / np.prod(right_img.shape)

            l_bad = left_ratio < threshold
            r_bad = right_ratio < threshold

            # cv2.imwrite(f'img/{t}.png', img)
            # print(os.getcwd())
            # Imgage analysis ends
        
            # if 0.3 < t < 2:
            if 0.3 < t < 2.3:
                self.start_sequence()
                self.init_motion = self.current_motion.get()
            elif t > 2:
            # if 1:
                self.fall_detector.check()

                if t < self.running:
                    continue
                else: 
                    if self.running != 0: print(t, "Motion end")
                    self.running = 0
                    self.walk(t)
                
                # if ratio < threshold:
                if l_bad and r_bad:
                    self.current_motion.set(self.motions['Backwards'])
                    self.running = t + 2.6
                    print(t, 'Backwards')
                    continue
                elif r_bad:
                    # self.current_motion.set(self.motions['SideStepLeft'])
                #     # self.running = t + 1.536
                #     self.running = t + 4.92
                #     print(t, 'SideStepLeft')
                    # Turn left 60
                    # self.current_motion.set(self.motions['TurnLeft60'])
                    # self.running = t + 4.52
                    # print(t, 'TurnLeft60')
                    # Turn left 20
                    # self.current_motion.set(self.motions['TurnLeft20'])
                    # self.running = t + 0.852
                    # print(t, 'TurnLeft20')
                    # Turn left 40
                    self.current_motion.set(self.motions['TurnLeft40'])
                    self.running = t + 2.880
                    print(t, 'TurnLeft40')

                    continue
                elif l_bad:
                #     self.current_motion.set(self.motions['SideStepRight'])
                #     # self.running = t + 1.536
                #     self.running = t + 5.76
                #     print(t, 'SideStepRight')
                    # Turn right 60
                    # self.current_motion.set(self.motions['TurnRight60'])
                    # self.running = t + 4.52
                    # print(t, 'TurnRight60')
                    # Turn right 20
                    # self.current_motion.set(self.motions['TurnRight20'])
                    # self.running = t + 0.852
                    # print(t, 'TurnRight20')
                    # Turn right 40
                    self.current_motion.set(self.motions['TurnRight40'])
                    self.running = t + 0.2272
                    print(t, 'TurnRight40')
                    continue
                
                
                test = np.random.uniform()
                # if test > 0.99:
                #     print(t, "TaiChi start")
                #     self.current_motion.set(self.motions['TaiChi_fast'])
                #     self.running = t + 1.3
                # elif test > 0.98:
                # if test > 0.997:
                if test > 0.999:
                    print(t, "Shoot start")
                    self.current_motion.set(self.motions['Shoot'])
                    self.running = t + 4.8
                # elif test > 0.995:
                elif test > 0.998:
                    # print(t, "TurnLeft180 start")
                    # self.current_motion.set(self.motions['TurnLeft180'])
                    # self.running = t + 9.0
                    print(t, "TurnLeft60 start")
                    self.current_motion.set(self.motions['TurnLeft60'])
                    self.running = t + 4.52
                # elif test > 0.993:
                elif test > 0.997:
                    print(t, "TurnRight start")
                    self.current_motion.set(self.motions['TurnRight60'])
                    self.running = t + 4.52
                # elif test > 0.990:
                elif test > 0.996:
                    print(t, 'Forward start')
                    self.current_motion.set(self.motions['Forwards'])
                    self.running = t + 2.6
                
                
    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)

    def walk(self, t):
        """Walk towards the opponent like a homing missile."""
        contour_area, normalized_x = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight.
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        # TODO: position estimation so that if the robot is close to the edge, it switches dodging direction
        # if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
        #     self.heading_angle = - self.heading_angle
        #     self.counter = 0
        # self.counter += 1

        print('Walking', contour_area)
        if contour_area <= 100 and np.random.uniform() > 0.99:
            # print(t, "Shoot start")
            self.current_motion.set(self.motions['Shoot'])
            self.running = t + 4.8

        
        self.gait_manager.command_to_motors(desired_radius=desired_radius, 
            # heading_angle=self.heading_angle
            heading_angle=0
        )

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        contour, _, horizontal_coordinate = IP.locate_opponent(img)
        try:
            contour_area = cv2.contourArea(contour)
        except:
            contour_area = 1000
            
        if horizontal_coordinate is None:
            return contour_area, 0
        return contour_area, horizontal_coordinate * 2 / img.shape[1] - 1


# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
