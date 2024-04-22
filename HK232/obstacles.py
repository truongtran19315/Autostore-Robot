import cv2
from consts import *
import numpy as np
import math
import random


class StaticObstacles():
    def __init__(self, xCenter, yCenter, shape, height=None, width=None, radius=None) -> None:
        self.xCenter = xCenter
        self.yCenter = yCenter
        self.height = height
        self.width = width
        self.shape = shape
        # distance from the center to the point in the corner
        if self.shape == 'goal':
            self.radius = radius
        else:
            self.radius = math.sqrt(self.height**2 + self.width**2) / 2

    def draw(self, screen):
        xStart = self.xCenter - self.width//2
        yStart = self.yCenter - self.height//2

        xEnd = self.xCenter + self.width//2
        yEnd = self.yCenter + self.height//2
        if self.shape == 'rectangle':
            cv2.rectangle(screen, (xStart, yStart),
                          (xEnd, yEnd), COLOR.GREEN, -1)
        else:
            cv2.rectangle(screen, (xStart, yStart),
                          (xEnd, yEnd), COLOR.RED, -1)


class RectangleObstacles(StaticObstacles):
    def __init__(self, xCenter, yCenter, height, width) -> None:
        super().__init__(xCenter, yCenter, 'rectangle')

        self.height = height
        self.width = width
        # distance from the center to the point in the corner
        self.radius = math.sqrt(self.height**2 + self.width**2) / 2

    def draw(self, screen, color):

        xStart = self.xCenter - self.width//2
        yStart = self.yCenter - self.height//2

        xEnd = self.xCenter + self.width//2
        yEnd = self.yCenter + self.height//2
        cv2.rectangle(screen, (xStart, yStart), (xEnd, yEnd), color, -1)


class Goal():
    def __init__(self):
        # self.randomGoal()
        self.radius = GAME_SETTING.GOAL_WIDTH / 2 * math.sqrt(2)
        self.width = GAME_SETTING.GOAL_WIDTH
        self.height = GAME_SETTING.GOAL_WIDTH

    def randomGoal(self, obstacleArr):
        rand1 = random.randint(1, 15)
        rand2 = random.randint(1, 8)
        self.x = obstacleArr[rand1 - 1][0] + \
            (rand2 % 2 - 1) * GAME_SETTING.GRID_WIDTH
        self.y = obstacleArr[rand1 - 1][1] + \
            (rand2 % 4 - 2) * GAME_SETTING.GRID_WIDTH
        self.xCenter = self.x + GAME_SETTING.GOAL_WIDTH / 2
        self.yCenter = self.y + GAME_SETTING.GOAL_WIDTH / 2

        #! added to no Random
        # self.x = 300
        # self.y = 200
        # self.xCenter = self.x + GAME_SETTING.GOAL_WIDTH/2
        # self.yCenter = self.y + GAME_SETTING.GOAL_WIDTH/2

    def draw(self, screen):
        cv2.rectangle(screen, (self.x, self.y),
                      (self.x + GAME_SETTING.GOAL_WIDTH, self.y + GAME_SETTING.GOAL_WIDTH), COLOR.PURPLE, -1)


class Obstacles():
    def __init__(self, numOfRect, map) -> None:

        self.goal = Goal()

        self.obstacles = []  # to save all obstacles

        self.numberOfRectangleObstacles = numOfRect

        # define wall
        # x, y height, width - 1100x700
        # left, bot, right, top
        self.wallArr = [[25, 350, 700, 8],
                        [550, 700, 8, 1050],
                        [1075, 350, 700, 8],
                        [550, 0, 8, 1050]]
        # self.wallArr = []

        # x, y height, width - 1100x700
        self.rectangleObstaclesArr = [[950, 605, 90, 100],
                                      [800, 605, 90, 100],
                                      [650, 605, 90, 100],
                                      [500, 605, 90, 100],
                                      [350, 605, 90, 100],
                                      [200, 605, 90, 100],

                                      [950, 455, 90, 100],
                                      [800, 455, 90, 100],
                                      [650, 455, 90, 100],
                                      [500, 455, 90, 100],
                                      [350, 455, 90, 100],
                                      [200, 455, 90, 100],

                                      [950, 255, 90, 100],
                                      [800, 255, 90, 100],
                                      [650, 255, 90, 100],
                                      [500, 255, 90, 100],
                                      [350, 255, 90, 100],
                                      [200, 255, 90, 100],
                                      
                                      [950, 105, 90, 100],
                                      [800, 105, 90, 100],
                                      [650, 105, 90, 100],
                                      [500, 105, 90, 100],
                                      [350, 105, 90, 100],
                                      [200, 105, 90, 100],]
        # self.rectangleObstaclesArr = []

        # add each of obstacleArr to a list of obstacle object
        self.listObstacle()

        self.goal.randomGoal(self.rectangleObstaclesArr)

    def listObstacle(self):
        self.rectangleObstacles = []  # to save all rectangle obstacles
        for obstacle in self.rectangleObstaclesArr:
            rectangle = StaticObstacles(
                xCenter=obstacle[0], yCenter=obstacle[1], height=obstacle[2], width=obstacle[3], shape='rectangle')
            self.rectangleObstacles.append(rectangle)
            self.obstacles.append(rectangle)

        self.wall = []
        for obstacle in self.wallArr:
            rectangle = StaticObstacles(
                xCenter=obstacle[0], yCenter=obstacle[1], height=obstacle[2], width=obstacle[3], shape='wall')
            self.wall.append(rectangle)
            self.obstacles.append(rectangle)       

    def generateObstacles(self, screen):
        for obstacle in self.obstacles:
            obstacle.draw(screen)


# img = np.zeros((500, 500, 3), dtype = np.uint8)

# obstacle = Obstacles(5, 5, 0)
# obstacle.generateObstacles(img)
# obstacle.goal.draw(img)

# cv2.putText(img, str(len(obstacle.obstacles)), (50, 450),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)

# cv2.imshow('WINDOW', img)
# cv2.waitKey(0)
