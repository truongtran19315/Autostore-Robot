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
        # goal is next to the obstacle
        # rand1 to choose the obstacle
        # rand2 to choose the pair of edge (left + bot vs right + top)
        # rand3 to choose the height edge or width edge
        rand1 = random.randint(1, len(obstacleArr))
        rand2 = random.randint(0, 1)
        rand3 = random.randint(0, 1)
        
        xObstacle, yObstacle, height, width = obstacleArr[rand1 - 1] 
        h = height // GAME_SETTING.GOAL_WIDTH
        w = width // GAME_SETTING.GOAL_WIDTH
        if rand2: # rand2 = 1 right + top
            if rand3: # rand3 = 1 top
                self.x = xObstacle - width // 2 + random.randint(0, w - 1) * GAME_SETTING.GOAL_WIDTH
                self.y = yObstacle - height // 2 - GAME_SETTING.GOAL_WIDTH
            else: # rand3 = 0 right
                self.x = xObstacle + width // 2
                self.y = yObstacle - height // 2 + random.randint(0, h - 1) * GAME_SETTING.GOAL_WIDTH
        else: # rand2 = 0 left + bot
            if rand3: # rand3 = 1 bot  
                self.x = xObstacle - width // 2 + random.randint(0, w - 1) * GAME_SETTING.GOAL_WIDTH
                self.y = yObstacle + height // 2
            else: # rand3 = 0 left
                self.x = xObstacle - width // 2 - GAME_SETTING.GOAL_WIDTH
                self.y = yObstacle - height // 2 + random.randint(0, h - 1) * GAME_SETTING.GOAL_WIDTH
        self.xCenter = self.x + GAME_SETTING.GOAL_WIDTH / 2
        self.yCenter = self.y + GAME_SETTING.GOAL_WIDTH / 2

        #! added to no Random
        # self.x = 300
        # self.y = 200
        # self.xCenter = self.x + GAME_SETTING.GOAL_WIDTH/2
        # self.yCenter = self.y + GAME_SETTING.GOAL_WIDTH/2
        #! print(xObstacle, yObstacle, height, width)
        #! print(self.x, self.y, rand1, rand2, rand3)

    def draw(self, screen):
        cv2.rectangle(screen, (self.x, self.y),
                      (self.x + GAME_SETTING.GOAL_WIDTH, self.y + GAME_SETTING.GOAL_WIDTH), COLOR.PURPLE, -1)


class Obstacles():
    def __init__(self, map) -> None:

        self.goal = Goal()

        self.obstacles = []  # to save all obstacles

        # define wall
        # x, y height, width - 1100x700
        # left, bot, right, top
        self.wallArr = [[25, 350, 700, 8],
                        [550, 700, 8, 1050],
                        [1075, 350, 700, 8],
                        [550, 0, 8, 1050]]
        # self.wallArr = []
        
        if map == MAP_SETTING.RANDOM_MAP:
            self.numberOfRectangleObstacles = OBSTACLE_SETTING.NUM_OBSTACLE
            self.randomObstacle()
        elif map == MAP_SETTING.MAP_DEFAULT:
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
            self.numberOfRectangleObstacles = len(self.rectangleObstaclesArr)
            self.goal.randomGoal(self.rectangleObstaclesArr)

        elif map == MAP_SETTING.MAP_DEMO:
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
            self.numberOfRectangleObstacles = len(self.rectangleObstaclesArr)
            #! Trái: [row_number][0]-60  Phải: [row_number][0]+50  
            #! Trên: [row_number][1]-55  Dưới: [row_number][1]+45
            row_number = 7
            self.goal.x, self.goal.y = self.rectangleObstaclesArr[row_number][0]-40, self.rectangleObstaclesArr[row_number][1]+45
            self.goal.xCenter = self.goal.x + GAME_SETTING.GOAL_WIDTH/2 
            self.goal.yCenter = self.goal.y + GAME_SETTING.GOAL_WIDTH/2
        # self.rectangleObstaclesArr = []

        # add each of obstacleArr to a list of obstacle object
        self.listObstacle()

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
            
    def randomObstacle(self):
        self.rectangleObstaclesArr = []
        for i in range(self.numberOfRectangleObstacles):
            x = random.randint(0, GAME_SETTING.SCREEN_WIDTH // 5) * 5
            y = random.randint(0, GAME_SETTING.SCREEN_HEIGHT // 5) * 5
            h = random.randint(2, OBSTACLE_SETTING.MAX_HEIGHT // OBSTACLE_SETTING.MIN_HEIGHT) * OBSTACLE_SETTING.MIN_HEIGHT
            w = random.randint(2, OBSTACLE_SETTING.MAX_WIDTH // OBSTACLE_SETTING.MIN_WIDTH) * OBSTACLE_SETTING.MIN_WIDTH
            self.rectangleObstaclesArr.append([x, y, h, w])
            
        check = False    
        while not check:
            self.goal.randomGoal(self.rectangleObstaclesArr)
            # print(self.goal.xCenter, self.goal.yCenter)
            check = True
            
            if self.goal.xCenter <= 25 or self.goal.xCenter >= 1075:
                check = False
                continue
            if check:
                for rect in self.rectangleObstaclesArr:
                    # print('rect: ', rect)
                    if (self.goal.xCenter > rect[0] - rect[3]//2 - self.goal.width // 2 - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.xCenter < rect[0] + rect[3]//2 + self.goal.width // 2 + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter > rect[1] - rect[2]//2 - self.goal.height // 2 - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter < rect[1] + rect[2]//2 + self.goal.height // 2 + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS):
                        check = False
                        break
                    else:
                        check = True    
                        
            if check:
                for wall in self.wallArr:
                    # print('wall: ', wall)
                    if (self.goal.xCenter > wall[0] - wall[3]//2 - self.goal.width // 2 - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.xCenter < wall[0] + wall[3]//2 + self.goal.width // 2 + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter > wall[1] - wall[2]//2 - self.goal.height // 2 - OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS) \
                        and (self.goal.yCenter < wall[1] + wall[2]//2 + self.goal.height // 2 + OBSTACLE_SETTING.MIN_DISTANCE_GOAL_VS_OBS):
                        check = False
                        break
                    else:
                        check = True    

    def generateObstacles(self, screen):
        for obstacle in self.obstacles:
            obstacle.draw(screen)


# img = np.ones((GAME_SETTING.SCREEN_HEIGHT, GAME_SETTING.SCREEN_WIDTH, 3), dtype=np.uint8) * 255

# obstacle = Obstacles(1)
# obstacle.generateObstacles(img)
# obstacle.goal.draw(img)

# cv2.putText(img, str(len(obstacle.obstacles)), (50, 450),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)

# cv2.imshow('WINDOW', img)
# cv2.waitKey(0)
