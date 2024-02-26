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
        
    def randomGoal(self):
        self.xCenter = random.randint(50, GAME_SETTING.SCREEN_WIDTH - 50)
        self.yCenter = random.randint(50, GAME_SETTING.SCREEN_HEIGHT - 50)

    def draw(self, screen):
        cv2.circle(screen, (self.xCenter, self.yCenter),
                   self.radius, COLOR.PURPLE, -1)


class Obstacles():
    def __init__(self, numOfCircle, numOfRect, map) -> None:

        self.obstacles = []  # to save all obstacles

        self.numberOfCircleObstacles = numOfCircle
        self.numberOfRectangleObstacles = numOfRect

        self.goal = Goal()
        
        # define wall 
        # x, y height, width - 1280x720
        self.wallArr = [[12, 360, 700, 4],
                        [640, 708, 4, 1260],
                        [1268, 360, 700, 4],
                        [640, 8, 4, 1260]]
        # self.wallArr = []
        
        ##### random obstacle
        if map == MAP_SETTING.RANDOM_MAP_ON:
            self.randomObstacle()
        elif map == MAP_SETTING.MAP_DEFAULT:
            # x, y, radius - 1280x720
            self.circleObstaclesArr = [[1025, 94, 87],
                                    [468, 347, 97],
                                    [297, 647, 59],
                                    [680, 407, 7],
                                    [1222, 442, 95]]
            # self.circleObstaclesArr = []

            # x, y height, width - 1280x720
            self.rectangleObstaclesArr = [[380, 100, 38, 26],
                                        [920, 612, 48, 5],
                                        [729, 178, 34, 29],
                                        [100, 104, 41, 26],
                                        [867, 471, 40, 22]]
            # self.rectangleObstaclesArr = []
        
        # add each of obstacleArr to a list of obstacle object
        self.listObstacle()
    
    def listObstacle(self):
        self.circleObstacles = []  # to save all circle obstacles
        for obstacle in self.circleObstaclesArr:
            circle = StaticObstacles(
                xCenter=obstacle[0], yCenter=obstacle[1], radius=obstacle[2], shape='circle')
            self.circleObstacles.append(circle)
            self.obstacles.append(circle)

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
            
    def randomObstacle(self):
        self.circleObstaclesArr = []
        for i in range(self.numberOfCircleObstacles):
            x = random.randint(0, GAME_SETTING.SCREEN_WIDTH)
            y = random.randint(0, GAME_SETTING.SCREEN_HEIGHT)
            r = random.randint(OBSTACLE_SETTING.MIN_RADIUS, OBSTACLE_SETTING.MAX_RADIUS)
            self.circleObstaclesArr.append([x, y, r])
        
        self.rectangleObstaclesArr = []
        for i in range(self.numberOfRectangleObstacles):
            x = random.randint(0, GAME_SETTING.SCREEN_WIDTH)
            y = random.randint(0, GAME_SETTING.SCREEN_HEIGHT)
            h = random.randint(OBSTACLE_SETTING.MIN_HEIGHT, OBSTACLE_SETTING.MAX_HEIGHT)
            w = random.randint(OBSTACLE_SETTING.MIN_WIDTH, OBSTACLE_SETTING.MAX_WIDTH)
            self.rectangleObstaclesArr.append([x, y, h, w])
            
        check = False    
        while not check:
            self.goal.randomGoal()
            # print(self.goal.xCenter, self.goal.yCenter)
            check = True
            if check:
                for circle in self.circleObstaclesArr:
                    distance = math.sqrt((circle[0] - self.goal.xCenter)**2 + (circle[1] - self.goal.yCenter)**2)
                    # print('circle: ', circle)
                    if distance <= circle[2] + self.goal.radius + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS:
                        check = False
                        break
                    else:
                        check = True
                        
            if check:
                for rect in self.rectangleObstaclesArr:
                    # print('rect: ', rect)
                    if (self.goal.xCenter >= rect[0] - rect[3]//2 - self.goal.radius - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.xCenter <= rect[0] + rect[3]//2 + self.goal.radius + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter >= rect[1] - rect[2]//2 - self.goal.radius - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter <= rect[1] + rect[2]//2 + self.goal.radius + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS):
                        check = False
                        break
                    else:
                        check = True    
                        
            if check:
                for wall in self.wallArr:
                    # print('wall: ', wall)
                    if (self.goal.xCenter >= wall[0] - wall[3]//2 - self.goal.radius - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.xCenter <= wall[0] + wall[3]//2 + self.goal.radius + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter >= wall[1] - wall[2]//2 - self.goal.radius - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter <= wall[1] + wall[2]//2 + self.goal.radius + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS):
                        check = False
                        break
                    else:
                        check = True 
            

    def generateObstacles(self, screen):
        for obstacle in self.obstacles:
            obstacle.draw(screen)


# img = np.zeros((720, 1280, 3), dtype = np.uint8)

# obstacle = Obstacles(5, 5, random=True)
# obstacle.generateObstacles(img)
# obstacle.goal.draw(img)

# cv2.putText(img, str(len(obstacle.obstacles)), (50, 450),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)

# cv2.imshow('WINDOW', img)
# cv2.waitKey(0)
