import cv2 
import numpy as np
import math
import random

ENV_HEIGHT = 700
ENV_WIDTH = 1250
BLUE = (255, 0, 0)
RED = (0, 0, 255)
GREEN = (0, 255, 0)

FORWARD_ACC = 0 # accelerate forward
BACKWARD_ACC = 1 # 
LEFT_ACC = 2
RIGHT_ACC = 3
STOP = 4

class PLAYER_SETTING:
    RADIUS_OBJECT = 10
    RADIUS_LIDAR = 100  # From the border of the circle

    INITIAL_X = ENV_WIDTH//2
    INITIAL_Y = ENV_HEIGHT - 20

    MAX_FORWARD_VELO = 100
    MAX_ROTATION_VELO = 1
    MIN_ROTATION_VELO = -MAX_ROTATION_VELO

    ACCELERATION_FORWARD = 5
    ACCELERATION_ROTATE = 0.05

    CASTED_RAYS = 45
    CASTED_RAYS = 90
    FOV = math.pi 
    HALF_FOV = FOV/2 # pi/2
    STEP_ANGLE = FOV / CASTED_RAYS 

    Y_GOAL_POSITION = 10

    # MAX_STEP_PER_EPOCH = 5000

class Car():
  def __init__(self, initX, initY, maxForwardVelocity, minRotationVelocity, maxRotationVelocity, accelerationForward, accelerationRotate, radiusObject) -> None:
    self.xPos, self.yPos = initX, initY
    self.maxForwardVelocity = maxForwardVelocity
    self.maxRotationVelocity = maxRotationVelocity #rotate to the right
    self.minRotationVelocity = minRotationVelocity #      //      left

    self.currentForwardVelocity = 0  # always >= 0
    self.currRotationVelocity = 0  # rotate left < 0, rotate right > 0

    self.currAngle = math.pi / 2
    self.accelerationForward = accelerationForward
    self.accelerationRotate = accelerationRotate

    self.radiusObject = radiusObject
    
  def move(self, action): # t = 1
    if action == FORWARD_ACC:
      if self.currentForwardVelocity != self.maxForwardVelocity:
        self.currentForwardVelocity = self.currentForwardVelocity + self.accelerationForward
      if self.currentForwardVelocity >= self.maxForwardVelocity:
        self.currentForwardVelocity = self.maxForwardVelocity
    elif action == BACKWARD_ACC:
      if self.currentForwardVelocity != 0:
        self.currentForwardVelocity = self.currentForwardVelocity + self.accelerationForward
      if self.currentForwardVelocity <= 0:
        self.currentForwardVelocity = 0
    elif action == LEFT_ACC:
      if self.currRotationVelocity != self.minRotationVelocity:
        self.currRotationVelocity = self.currRotationVelocity + self.accelerationRotate
      if self.currRotationVelocity <= self.minRotationVelocity:
        self.currRotationVelocity = self.minRotationVelocity
    elif action == RIGHT_ACC:
      if self.currRotationVelocity != self.maxRotationVelocity:
        self.currRotationVelocity = self.currRotationVelocity + self.accelerationRotate
      if self.currRotationVelocity >= self.maxRotationVelocity:
        self.currRotationVelocity = self.maxRotationVelocity
    elif action == STOP:
      self.currentForwardVelocity = 0
      self.currRotationVelocity = 0
      
    
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
  
  def scanLidar(self, obstacles):
    pass
    
  def checkCollision(self, collisions, lane):
      pass
  def checkAchieveGoal(self):
    pass
  def draw(self, screen):
    pass
      
class Obstacles(Car):
  def __init__(self, initX, initY):
    self.xPos = initX
    self.yPos = initY
    
  def draw(self, screen):
    radius = random.randint(10, 50)
    # cv2.circle(image, center_coordinates, radius, color, thickness)
    cv2.circle(screen, (self.xPos, self.yPos), radius, RED, -1)
    
class ObstaclesStatic():
  def __init__(self, initX, initY) -> None:
    self.xPos = initX
    self.yPos = initY
  
  def draw(self, screen):
    radius = random.randint(10, 50)
    shape = random.randint(1, 2)
    if shape == 1:
      cv2.circle(screen, (self.xPos, self.yPos), radius, GREEN, -1)
    else:
      cv2.rectangle(screen, (self.xPos - radius, self.yPos - radius), (self.xPos + radius, self.yPos + radius), GREEN, -1)
      
    
      
class PyGame2D():
  def __init__(self):
    # create a black image
    self.img = np.zeros((ENV_HEIGHT, ENV_WIDTH, 3), dtype = np.uint8)
    
    # Draw border
    # cv2.rectangle(self.img, start_point, end_point, color, thickness)
    cv2.rectangle(self.img, (0 + 5, 0 + 5), (ENV_WIDTH - 5, ENV_HEIGHT - 5), BLUE, 1)
    
    obstacles = Obstacles(random.randint(5, 100), random.randint(200, 500))
    obstaclesStatis = ObstaclesStatic(random.randint(50, 150), random.randint(200, 250))
    
    obstacles.draw(screen = self.img)
    obstaclesStatis.draw(screen = self.img)

    # display the image using opencv
    cv2.imshow('WINDOW', self.img)
    cv2.waitKey(0)
    
    
    
    
PyGame2D()