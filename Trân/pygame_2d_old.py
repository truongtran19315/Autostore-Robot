import cv2 
import numpy as np
import math
import random
from obstacles import Obstacles
from const import *
from utils import *
import time

ENV_HEIGHT = 720
ENV_WIDTH = 1280
BLUE = (255, 0, 0)
RED = (0, 0, 255)
GREEN = (0, 255, 0)

ZERO = 0
INFINITY = 99999

FORWARD_ACC = 0 # accelerate forward
BACKWARD_ACC = 1 # 
LEFT_ACC = 2
RIGHT_ACC = 3
STOP = 4

class Car():
  def __init__(self, initX, initY, maxForwardVelocity, minRotationVelocity, maxRotationVelocity, accelerationForward, accelerationRotate, radiusObject) -> None:
    self.xPos, self.yPos = initX, initY
    self.maxForwardVelocity = maxForwardVelocity
    self.maxRotationVelocity = maxRotationVelocity #rotate to the right
    self.minRotationVelocity = minRotationVelocity #      //      left

    self.currentForwardVelocity = 0  # always >= 0
    self.currRotationVelocity = 0  # rotate left < 0, rotate right > 0

    self.currAngle = math.pi / 2
    self.currAngle = math.pi 
    self.accelerationForward = accelerationForward
    self.accelerationRotate = accelerationRotate
    
    
    self.xStep = 0
    self.yStep = 0

    self.radiusObject = radiusObject
    
  def move(self, action): # t = 1
    if action == ACTIONS.FORWARD_ACCELERATION:
      if self.currentForwardVelocity != self.maxForwardVelocity:
        self.currentForwardVelocity = self.currentForwardVelocity + self.accelerationForward
      if self.currentForwardVelocity >= self.maxForwardVelocity:
        self.currentForwardVelocity = self.maxForwardVelocity
    elif action == ACTIONS.BACKWARD_ACCELERATION:
      if self.currentForwardVelocity != 0:
        self.currentForwardVelocity = self.currentForwardVelocity + self.accelerationForward
      if self.currentForwardVelocity <= 0:
        self.currentForwardVelocity = 0
    elif action == ACTIONS.TURN_LEFT_ACCELERATION:
      if self.currRotationVelocity != self.minRotationVelocity:
        self.currRotationVelocity = self.currRotationVelocity - self.accelerationRotate
      if self.currRotationVelocity <= self.minRotationVelocity:
        self.currRotationVelocity = self.minRotationVelocity
    elif action == ACTIONS.TURN_RIGHT_ACCELERATION:
      if self.currRotationVelocity != self.maxRotationVelocity:
        self.currRotationVelocity = self.currRotationVelocity + self.accelerationRotate
      if self.currRotationVelocity >= self.maxRotationVelocity:
        self.currRotationVelocity = self.maxRotationVelocity
    elif action == ACTIONS.STOP:
      self.currentForwardVelocity = 0
      self.currRotationVelocity = 0
    else:
      pass
    
    # Calculate the position base on velocity per frame
    dt = float(1/GAME_SETTING.FPS)
    # dt = 1

    self.currAngle += (self.currRotationVelocity*dt)

    # Prevent car go to the opposite way
    if (self.currAngle < 0):
      self.currAngle = 2*math.pi - abs(self.currAngle)
    elif (self.currAngle > 2*math.pi):
      self.currAngle = abs(self.currAngle - 2*math.pi)

    # Update the new position based on the velocity
    self.yStep += math.cos(self.currAngle) * \
      self.currentForwardVelocity * dt
    self.xStep += -math.sin(self.currAngle) * \
      self.currentForwardVelocity * dt
    
    if self.yStep >= 1 or self.yStep <= -1:
      self.yPos += int(self.yStep)
      self.yStep = 0
    if self.xStep >= 1 or self.xStep <= -1:
      self.xPos += int(self.xStep)
      self.xStep = 0
    
    print("current position: ", self.xPos, self.yPos, " action: ", action, " current angle: ", self.currAngle, math.cos(self.currAngle))
    # print("action: ", action)
    # print("current forward velocity: ", self.currentForwardVelocity)
    # print("current rotate velocity: ", self.currRotationVelocity)
    # print("current angle: ", self.currAngle)
      
    
class Robot(Car):
  def __init__(self) -> None:
    super().__init__(
      initX = PLAYER_SETTING.INITIAL_X,
      initY = PLAYER_SETTING.INITIAL_Y,
      maxForwardVelocity = PLAYER_SETTING.MAX_FORWARD_VELO,
      minRotationVelocity = PLAYER_SETTING.MIN_ROTATION_VELO,
      maxRotationVelocity = PLAYER_SETTING.MAX_ROTATION_VELO,
      accelerationForward = PLAYER_SETTING.ACCELERATION_FORWARD,
      accelerationRotate = PLAYER_SETTING.ACCELERATION_ROTATE,
      radiusObject = PLAYER_SETTING.RADIUS_OBJECT
    )
    
    # 360 lidar signal around robot 
    self.lidarSignals = [INFINITY]*PLAYER_SETTING.CASTED_RAYS # lidar return distance from robot to obstacles in range - return infinity if not have obstacle
    self.lidarVisualize = [{"source": {"x": self.xPos, "y": self.yPos},
                            "target": {"x": self.xPos, "y": self.yPos},
                            "color": COLOR.WHITE
                            } for x in range(PLAYER_SETTING.CASTED_RAYS)]
  
  def scanLidar(self, obstacles):
    obstaclesInRange = []  #to save obstacles in lidar range
    
    for obstacle in obstacles.obstacles:
      distance = Utils.distanceBetweenTwoPoints(
          self.xPos, self.yPos, obstacle.xCenter, obstacle.yCenter)
      if obstacle.shape == 'circle':
        isInRageLidar = distance < obstacle.radius + \
            PLAYER_SETTING.RADIUS_LIDAR
      else: 
        radius = math.sqrt(obstacle.height**2 + obstacle.width**2) / 2  # distance from the center to the point in the corner
        isInRageLidar = distance < radius + \
            PLAYER_SETTING.RADIUS_LIDAR
      if (isInRageLidar == True):
          obstaclesInRange.append(obstacle)
    startAngle = 0
      
    if (len(obstaclesInRange) == 0):
      for ray in range(PLAYER_SETTING.CASTED_RAYS):
        target_x = int(self.xPos - \
            math.sin(startAngle) * PLAYER_SETTING.RADIUS_LIDAR)
        target_y = int(self.yPos + \
            math.cos(startAngle) * PLAYER_SETTING.RADIUS_LIDAR)
        self.lidarVisualize[ray]["target"] = {
            "x": target_x,
            "y": target_y
        }
        self.lidarVisualize[ray]["source"] = {
            "x": self.xPos,
            "y": self.yPos
        }
        self.lidarVisualize[ray]["color"] = COLOR.CYAN
        self.lidarSignals[ray] = INT_INFINITY
        startAngle += PLAYER_SETTING.STEP_ANGLE
    else:
      for ray in range(PLAYER_SETTING.CASTED_RAYS):
        # if ray == 0:
        #   a = 1
        target_x = int(self.xPos - \
          math.sin(startAngle) * PLAYER_SETTING.RADIUS_LIDAR)
        target_y = int(self.yPos + \
          math.cos(startAngle) * PLAYER_SETTING.RADIUS_LIDAR)
        
        # self.lidarVisualize[ray]["target"] = {
        #     "x": target_x,
        #     "y": target_y
        # }
        self.lidarVisualize[ray]["source"] = {
            "x": self.xPos,
            "y": self.yPos
        }
        
        distance = INT_INFINITY
        
        for obstacle in obstaclesInRange:
          distance = min(distance, Utils.getDistanceFromObstacle(obstacle, self.xPos, self.yPos, target_x, target_y))
          # if ray == 0:
          #   print("distance 0: ", distance)
          # elif ray == 180:
          #   print("distance 180: ", distance)
          
        if distance <= PLAYER_SETTING.RADIUS_LIDAR:
          target_x = int(self.xPos - math.sin(startAngle) * distance)
          target_y = int(self.yPos + math.cos(startAngle) * distance)
          self.lidarSignals[ray] = distance
          self.lidarVisualize[ray]["color"] = COLOR.RED
          self.lidarVisualize[ray]["target"] = {
              "x": target_x,
              "y": target_y
          }
        else:
          self.lidarSignals[ray] = INT_INFINITY
          self.lidarVisualize[ray]["color"] = COLOR.CYAN
          self.lidarVisualize[ray]["target"] = {
              "x": target_x,
              "y": target_y
          }
          
        startAngle += PLAYER_SETTING.STEP_ANGLE
    
  def checkCollision(self, collisions):
    pass
  
  def checkAchieveGoal(self):
    pass
  
  def draw(self, screen):
    # cv2.circle(screen, (self.xPos, self.yPos), 370, COLOR.BLUE, -1)
    
    for lidarItemVisualize in self.lidarVisualize:
      color = lidarItemVisualize["color"]
      srcX = lidarItemVisualize["source"]["x"]
      srcY = lidarItemVisualize["source"]["y"]
      targetX = lidarItemVisualize["target"]["x"]
      targetY = lidarItemVisualize["target"]["y"]
      cv2.line(screen, (srcX, srcY), (targetX, targetY), color, 1)
      
      
    target_x = int(self.xPos - \
        math.sin(self.currAngle) * PLAYER_SETTING.RADIUS_LIDAR / 3)
    target_y = int(self.yPos + \
        math.cos(self.currAngle) * PLAYER_SETTING.RADIUS_LIDAR / 3)
    cv2.line(screen, (self.xPos, self.yPos), (target_x, target_y), COLOR.WHITE, 5)
    cv2.circle(screen, (self.xPos, self.yPos), self.radiusObject, COLOR.BLUE, -1)
    
      
class PyGame2D():
  def __init__(self, screen) -> None:
    self.screen = screen
    self.obstacles = self._initObstacle()
    self.robot = Robot()
  
  def _initObstacle(self):
    return Obstacles(self.screen)
    
  def _obstacleMoves(self):
    pass   
  
  def action(self, action):
    self.robot.move(action=action)
    # self._obstacleMoves()
    # self.robot.checkCollision(collisions=self.obstacles, lane=self.lane)
    
    start_time = time.time()
    
    self.robot.scanLidar(obstacles=self.obstacles)
    
    end_time = time.time()
    
    elapsed_time = end_time - start_time
    print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

    # self.robot.checkAchieveGoal()
  
  def evaluate(self):
    pass    
  
  def is_done(self):
    pass
  
  def observe(self): 
    pass   
  
  def update(self): 
    pass
  
  def view(self, screen):  
    self.obstacles.generateObstacles(screen)
    
    self.robot.draw(screen)
    # print(self.obstacles.get())
    # cv2.circle(self.screen, (564, 96), 10, COLOR.CYAN, -1)
    print(self.robot.xPos, self.robot.yPos)
    
    cv2.imshow('Enviroment', screen)
    # cv2.waitKey(0)
    
    
screen = np.zeros((720, 1280, 3), dtype = np.uint8)      
game = PyGame2D(screen)
# game.view()
while True:
    screen = np.zeros((720, 1280, 3), dtype = np.uint8)  
    Utils.inputUser(game)   
    game.view(screen)
    pass

# Utils.inputUser(game)
# game.view()