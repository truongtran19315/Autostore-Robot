import cv2 
import numpy as np
import math
import random
from obstacles import Obstacles, Goal
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
    self.currAngle = math.pi  #! Truong: vague
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
        self.currentForwardVelocity = self.currentForwardVelocity - self.accelerationForward
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
    
    # print(self.xPos, self.yPos, " current angle: ", self.currAngle, self.currentForwardVelocity)
      
    
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
    
    
    self.isAlive = True
    self.achieveGoal = False
    # 360 lidar signal around robot 
    self.lidarSignals = [INFINITY]*PLAYER_SETTING.CASTED_RAYS # khởi tạo list (360) chứa độ dài tia lidar tới vật cản
    # khởi tạo 360 đối tượng tia lidar tới vật cản, màu trắng 
    self.lidarVisualize = [{"source": {"x": self.xPos, "y": self.yPos},
                            "target": {"x": self.xPos, "y": self.yPos},
                            "color": COLOR.WHITE
                            } for x in range(PLAYER_SETTING.CASTED_RAYS)]
  
  def scanLidar(self, obstacles):
    obstaclesInRange = []  # Tạo một danh sách rỗng để lưu trữ các vật cản nằm trong phạm vi quét của lidar.
    minDistance = INT_INFINITY  # lưu khoảng cách tới vật cản gần nhất
    
    # Duyệt qua ds các vật cản và kiểm trả xem khoảng cách có nằm trong phạm vi quét của lidar hay không 
    for obstacle in obstacles.obstacles:
      distance = Utils.distanceBetweenTwoPoints(
          self.xPos, self.yPos, obstacle.xCenter, obstacle.yCenter)
      isInRageLidar = distance < obstacle.radius + \
          PLAYER_SETTING.RADIUS_LIDAR
      if (isInRageLidar == True):
          obstaclesInRange.append(obstacle)   # nếu có thì thêm vật cản này vào danh sách 'obstaclesInRange'
          
    startAngle = 0  # Khởi tạo góc bắt đầu cho tín hiệu lidar là 0 độ
    
    # Tính toán vị trí đích của tia quét (target_x và target_y) khi "chạm" vật cản (không phải trung tâm)
    for ray in range(PLAYER_SETTING.CASTED_RAYS):
      target_x = int(self.xPos + \
        math.cos(startAngle) * PLAYER_SETTING.RADIUS_LIDAR)
      target_y = int(self.yPos - \
        math.sin(startAngle) * PLAYER_SETTING.RADIUS_LIDAR)
      
      self.lidarVisualize[ray]["source"] = {
          "x": self.xPos,
          "y": self.yPos
      }
      
      distance = INT_INFINITY
      x = INT_INFINITY
      y = INT_INFINITY
      
      for obstacle in obstaclesInRange:
        (d, x, y) = Utils.getDistanceFromObstacle(obstacle, self.xPos, self.yPos, target_x, target_y)
        if d < distance:
          distance = d
          target_x = x
          target_y = y
          
      minDistance = min(distance, minDistance)
        
      if distance <= PLAYER_SETTING.RADIUS_LIDAR:
        self.lidarSignals[ray] = distance
        self.lidarVisualize[ray]["color"] = COLOR.RED
      else:
        self.lidarSignals[ray] = INT_INFINITY
        self.lidarVisualize[ray]["color"] = COLOR.CYAN
      
      self.lidarVisualize[ray]["target"] = {
          "x": target_x,
          "y": target_y
      }
        
      startAngle += PLAYER_SETTING.STEP_ANGLE
    
    return minDistance
    
  def checkCollision(self, distance):
    if distance < self.radiusObject:
      self.isAlive = False
  
  def checkAchieveGoal(self, goal):
    distance = Utils.distanceBetweenTwoPoints(self.xPos, self.yPos, goal.xCenter, goal.yCenter)
    if distance < self.radiusObject + goal.radius:  
      self.achieveGoal = True
  
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
    self.goal = Goal()
    self.robot = Robot()
    self.n = 0  # Đếm số lần thực hiện hành động 
    self.totalTime = 0
    self.minTime = INT_INFINITY
    self.maxTime = 0
    # Lưu trữ thông tin về trạng thái màn hình, hành động và trạng thái của robot khi thời gian thực hiện hành động tối thiểu và tối đa.
    self.saveMin = {"screen": self.screen,
                    "action": -1,
                    "robot": self.robot}
    self.saveMax = {"screen": self.screen,
                    "action": -1,
                    "robot": self.robot}
  
  def _initObstacle(self):
    return Obstacles(self.screen)
    
  def _obstacleMoves(self):
    pass   
  
  def action(self, action):
    self.robot.move(action=action)
    # self._obstacleMoves()
    self.n += 1
    start_time = time.time()
    
    distance = self.robot.scanLidar(obstacles=self.obstacles)
    
    end_time = time.time()
    
    elapsed_time = end_time - start_time
    
    self.totalTime += elapsed_time
    mediumTime = self.totalTime / self.n
    if elapsed_time < self.minTime:
      self.minTime = elapsed_time
      self.saveMin["screen"] = self.screen
      self.saveMin["action"] = action
      self.saveMin["robot"] = self.robot
      
    if elapsed_time > self.maxTime:
      self.maxTime = elapsed_time
      self.saveMax["screen"] = self.screen
      self.saveMax["action"] = action
      self.saveMax["robot"] = self.robot
    
    print (elapsed_time, mediumTime, self.minTime, self.maxTime, self.n)

    self.robot.checkCollision(distance)
    self.robot.checkAchieveGoal(self.goal)
  
  def evaluate(self):
    pass    
  
  def is_done(self):
    if ((not self.robot.isAlive) or self.robot.achieveGoal):
      return True
    return False
  
  def observe(self): 
    ratioLeft = (self.robot.xPos)/(GAME_SETTING.SCREEN_WIDTH)
    alpha = self.robot.currAngle
    fwVelo = self.robot.currentForwardVelocity
    rVelo = self.robot.currRotationVelocity
    lidars = self.robot.lidarSignals

    infoStateVector = np.array([ratioLeft, alpha, fwVelo, rVelo])
    lidarStateVector = np.array(lidars)
    return np.concatenate((infoStateVector, lidarStateVector)) 
  
  def update(self): 
    pass
  
  def view(self, screen):  
    self.obstacles.generateObstacles(screen)
    self.goal.draw(screen)
    self.robot.draw(screen)
    # print(self.obstacles.get())
    # cv2.circle(self.screen, (564, 96), 10, COLOR.CYAN, -1)
    
    cv2.imshow('Enviroment', screen)
    # cv2.waitKey(0)
    
  def convert_lenLidar(self):
    self.robot.lidarSignals = np.array(self.robot.lidarSignals)
    self.robot.lidarSignals[self.robot.lidarSignals <= 120] = 2
    self.robot.lidarSignals[(self.robot.lidarSignals > 120) & (self.robot.lidarSignals <= 240)] = 1
    self.robot.lidarSignals[(self.robot.lidarSignals > 240) & (self.robot.lidarSignals <= 360)] = 0
    self.robot.lidarSignals[self.robot.lidarSignals > 360] = INT_INFINITY
    
  def convert_ObstacleDetectionArea(self):
    self.convert_lenLidar()
    
    
    
    
# screen = np.zeros((720, 1280, 3), dtype = np.uint8)      
# game = PyGame2D(screen)
# while True:
#     screen = np.zeros((720, 1280, 3), dtype = np.uint8)  
#     a = Utils.inputUser(game)   
#     game.view(screen)
#     if not game.robot.isAlive:
#       print("Oops!!!!!!!!!!")
#       break
#     elif game.robot.achieveGoal:
#       print("Great!!!!!!!!!")
#       break
#     if a == False:
#       cv2.imshow('min', game.saveMin["screen"])
#       cv2.waitKey(0)
#       break
#     pass
