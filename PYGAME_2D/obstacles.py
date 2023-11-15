import cv2
from consts import *
import numpy as np
import math


class StaticObstacles():
    def __init__(self, xCenter, yCenter, shape, height=None, width=None, radius=None) -> None:
        self.xCenter = xCenter
        self.yCenter = yCenter
        self.height = height
        self.width = width
        self.shape = shape
        if self.shape == 'circle' or self.shape == 'goal':
            self.radius = radius
        else:
            # distance from the center to the point in the corner
            self.radius = math.sqrt(self.height**2 + self.width**2) / 2

    def draw(self, screen):
        if self.shape == 'circle':
            cv2.circle(screen, (self.xCenter, self.yCenter),
                       self.radius, COLOR.GREEN, -1)
        else:
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


class CircleObstacles(StaticObstacles):
    def __init__(self, xCenter, yCenter, radius) -> None:
        super().__init__(xCenter, yCenter, 'circle')

        self.radius = radius

    def draw(self, screen):
        cv2.circle(screen, (self.xCenter, self.yCenter),
                   self.radius, COLOR.GREEN, -1)


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


class Goal(StaticObstacles):
    def __init__(self) -> None:
        super().__init__(
            xCenter=PLAYER_SETTING.GOAL_POSITION["x"], yCenter=PLAYER_SETTING.GOAL_POSITION["y"], shape="goal", radius=PLAYER_SETTING.GOAL_RADIUS)

    def draw(self, screen):
        cv2.circle(screen, (self.xCenter, self.yCenter),
                   self.radius, COLOR.PURPLE, -1)


class Obstacles():
    def __init__(self) -> None:

        self.obstacles = []  # to save all obstacles

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
        self.circleObstaclesArr = []

        self.circleObstacles = []  # to save all circle obstacles
        for obstacle in self.circleObstaclesArr:
            circle = StaticObstacles(
                xCenter=obstacle[0], yCenter=obstacle[1], radius=obstacle[2], shape='circle')
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
        self.rectangleObstaclesArr = []

        self.rectangleObstacles = []  # to save all rectangle obstacles
        for obstacle in self.rectangleObstaclesArr:
            rectangle = StaticObstacles(
                xCenter=obstacle[0], yCenter=obstacle[1], height=obstacle[2], width=obstacle[3], shape='rectangle')
            self.rectangleObstacles.append(rectangle)
            self.obstacles.append(rectangle)

        # x, y height, width - 1280x720
        self.wallArr = [[12, 360, 700, 4],
                        [640, 708, 4, 1260],
                        [1268, 360, 700, 4],
                        [640, 8, 4, 1260]]
        # self.wallArr = []

        self.wall = []
        for obstacle in self.wallArr:
            rectangle = StaticObstacles(
                xCenter=obstacle[0], yCenter=obstacle[1], height=obstacle[2], width=obstacle[3], shape='wall')
            self.wall.append(rectangle)
            self.obstacles.append(rectangle)

    def generateObstacles(self, screen):
        for obstacle in self.obstacles:
            obstacle.draw(screen)


# img = np.zeros((720, 1280, 3), dtype = np.uint8)

# obstacle = Obstacles(img)
# obstacle.generateObstacles()

# cv2.imshow('WINDOW', img)
# cv2.waitKey(0)
