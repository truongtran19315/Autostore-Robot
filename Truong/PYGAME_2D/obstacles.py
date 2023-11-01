import cv2
from const import *
import numpy as np
import math

class StaticObstacles():
  def __init__(self, xCenter, yCenter, shape) -> None:
    self.xCenter = xCenter
    self.yCenter = yCenter
    self.shape = shape
    
class CircleObstacles(StaticObstacles):
  def __init__(self, xCenter, yCenter, radius) -> None:
    super().__init__(xCenter, yCenter, 'circle')
    
    self.radius = radius
    
  def draw(self, screen):
    cv2.circle(screen, (self.xCenter, self.yCenter), self.radius, COLOR.GREEN, -1)
    
class RectangleObstacles(StaticObstacles):
  def __init__(self, xCenter, yCenter, height, width) -> None:
    super().__init__(xCenter, yCenter, 'rectangle')
    
    self.height = height
    self.width = width 
    self.radius = math.sqrt(self.height**2 + self.width**2) / 2  # distance from the center to the point in the corner
    
  def draw(self, screen, color):
    
    xStart = self.xCenter - self.width//2
    yStart = self.yCenter - self.height//2
    
    xEnd = self.xCenter + self.width//2
    yEnd = self.yCenter + self.height//2
    cv2.rectangle(screen, (xStart, yStart), (xEnd, yEnd), color, -1)
    
class Goal(StaticObstacles):
  def __init__(self) -> None:
    super().__init__(PLAYER_SETTING.GOAL_POSITION["x"], PLAYER_SETTING.GOAL_POSITION["y"], "goal")
    
    self.radius = PLAYER_SETTING.GOAL_RADIUS
    
  def draw(self, screen):
    cv2.circle(screen, (self.xCenter, self.yCenter), self.radius, COLOR.PURPLE, -1)
    
class Obstacles():
  def __init__(self, screen) -> None:
    
    self.obstacles = [] # to save all obstacles 
    
    self.numberOfCircleObstacles = 10
    # x, y, radius - 1280x720
    self.circleObstaclesArr = [[1025, 94, 87],     
                            [468, 347, 97],
                            [164, 226, 60],
                            [715, 75, 50],
                            [771, 503, 59],
                            [227, 675, 7],
                            [297, 647, 59],
                            [680, 407, 7],
                            [427, 608, 13],
                            [1222, 442, 95]]
    
    self.circleObstacles = [] # to save all circle obstacles
    for obstacle in self.circleObstaclesArr:
      circle = CircleObstacles(obstacle[0], obstacle[1], obstacle[2])
      self.circleObstacles.append(circle)
      self.obstacles.append(circle)
      
    self.numberOfRectangleObstacles = 10
    # x, y height, width - 1280x720
    self.rectangleObstaclesArr = [[1224, 246, 23, 15],
                              [438, 115, 38, 26],
                              [1272, 295, 32, 17],
                              [920, 612, 48, 5],
                              [351, 496, 14, 42],
                              [606, 520, 28, 16],
                              [729, 178, 34, 29],
                              [100, 104, 41, 26],
                              [101, 712, 35, 37],
                              [867, 471, 40, 22]]
    self.rectangleObstacles = [] # to save all rectangle obstacles
    for obstacle in self.rectangleObstaclesArr:
      rectangle = RectangleObstacles(obstacle[0], obstacle[1], obstacle[2], obstacle[3])
      self.rectangleObstacles.append(rectangle)
      self.obstacles.append(rectangle)
    
    # x, y height, width - 1280x720
    self.wallArr = [[12, 360, 700, 4],
                  [640, 708, 4, 1260],
                  [1268, 360, 700, 4],
                  [640, 8, 4, 1260]]
    self.wall = []
    for obstacle in self.wallArr:
      rectangle = RectangleObstacles(obstacle[0], obstacle[1], obstacle[2], obstacle[3])
      self.wall.append(rectangle)
      self.obstacles.append(rectangle)
      
  
  def generateObstacles(self, screen):
    for circle in self.circleObstacles:
      circle.draw(screen)
      
    for rectangle in self.rectangleObstacles:
      rectangle.draw(screen, COLOR.GREEN)
      
    for rectangle in self.wall:
      rectangle.draw(screen, COLOR.RED)
      

# img = np.zeros((720, 1280, 3), dtype = np.uint8)      

# obstacle = Obstacles(img)
# obstacle.generateObstacles()

# cv2.imshow('WINDOW', img)
# cv2.waitKey(0)