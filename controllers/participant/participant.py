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
      'TurnLeft60': (Motion('../motions/TurnLeft60.motion'), 4.52),
      'TurnRight60': Motion('../motions/TurnRight60.motion'),
      'TurnLeft20': (Motion('../motions/TurnLeft20.motion'), 0.852),
      'TurnRight20': (Motion('../motions/TurnRight20.motion'), 0.852),
      'TurnRight40': Motion('../motions/TurnRight40.motion'),
      'TurnLeft40': Motion('../motions/TurnLeft40.motion'),
      'TaiChi': Motion('../motions/TaiChi.motion'),
      'TaiChi_fast': Motion('../motions/TaiChi_fast.motion'),
      'Shoot': Motion('../motions/Shoot.motion'),
      'Backwards': Motion('../motions/Backwards.motion'),
      'Forwards': (Motion('../motions/Forwards.motion'), 2.60),
      'ForwardLoop': (Motion('../motions/ForwardLoop.motion'), 0.960),
      'ForwardLoop_fast': (Motion('../motions/ForwardLoop_fast.motion'), 0.480 + 0.2),
      # 'ForwardLoop_fast2': (Motion('../motions/ForwardLoop_fast2.motion'), 0.240 + 0.2),
    }


  def run(self):
    # self.sonar = self.getDevice('Sonar/Right')
    # self.sonar.enable(100000)
    self.running = 0
    init_tr = 0
    fw_cnt = 0
    bw_tr = False
    # print(dir(self))
    # print(self.devices)
    while self.step(self.time_step) != -1:
      # We need to update the internal theta value of the gait manager at every step:
      t = self.getTime()
      self.gait_manager.update_theta()

      threshold = 0.05
      img = self.camera2.get_image()
      img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      pix = 150
      img[img < pix] = 0
      img[img >= pix] = 1
      l, w = img.shape
      img = img[int(l*0.5):, :]

      left_img = img[:, :int(w/2)]
      right_img = img[:, int(w/2):]
      # cv2.imwrite('tmp.png', img*255)

      left_white_px = left_img.sum()
      right_white_px = right_img.sum()
      left_ratio = left_white_px / np.prod(left_img.shape)
      right_ratio = right_white_px / np.prod(right_img.shape)

      l_bad = left_ratio < threshold
      r_bad = right_ratio < threshold

      # sonar_right_val = self.sonar.getValue()
      # print("Sonar right:", sonar_right_val)
    
      # if 0.3 < t < 2:
      if 0.3 < t < 2.3:
        self.start_sequence()
        pass
      elif t > 2:
      # if 1:
        self.fall_detector.check()

        if t < self.running:
          continue
        else: 
          if self.running != 0: print(t, "Motion end")
          self.running = 0
          # self.walk(t)
        
        if bw_tr:
          bw_tr = False
          # self.current_motion.set(self.motions['TurnRight40'])
          # self.running = t + 0.2272
          # print(t, 'TurnRight40')
          self.current_motion.set(self.motions['TurnLeft40'])
          self.running = t + 2.880
          print(t, 'TurnLeft40')
          continue

        # sonar_bad = (sonar_right_val < 0.30)
        sonar_bad = False
        if (l_bad and r_bad) or sonar_bad:
          if sonar_bad: print("Too near !!!")
          self.current_motion.set(self.motions['Backwards'])
          self.running = t + 2.6
          print(t, 'Backwards')
          bw_tr = True
          continue
        elif r_bad:
          self.current_motion.set(self.motions['TurnLeft40'])
          self.running = t + 2.880
          print(t, 'TurnLeft40')
          continue
        elif l_bad:
          self.current_motion.set(self.motions['TurnRight40'])
          self.running = t + 0.2272
          print(t, 'TurnRight40')
          continue
          
        if init_tr < 4:
          tr = self.motions['TurnRight20']
          self.current_motion.set(tr[0])
          self.running = t + tr[1]
          init_tr += 1

        else:
        # if 1:
          print('Forwards start')
          # tr = self.motions['Forwards']
          # tr = self.motions['ForwardLoop']
          tr = self.motions['ForwardLoop_fast']
          # tr = self.motions['ForwardLoop_fast2']
          self.current_motion.set(tr[0])
          self.running = t + tr[1]
          # fw_cnt += 1
          # if fw_cnt == 10:
          #   print('TurnLeft20 start')
          #   tr = self.motions['TurnLeft20']
          #   self.current_motion.set(tr[0])
          #   self.running = t + tr[1]
          #   fw_cnt = 0


        
        # else:
        #   tr = self.motions['TurnRight20']
        #   self.current_motion.set(tr[0])
        #   self.running = t + tr[1]
        #   contour_area, normalized_x = self._get_normalized_opponent_x()
        #   print("Contour area: ", contour_area)
          
          

        
        
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
