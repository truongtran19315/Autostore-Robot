import cv2
import numpy as np
import math
import random
from obstacles import Obstacles, Goal
from consts import *
from utils import Utils
import time
from logVersion import *


class Car():
    def __init__(self, initX, initY, initAngleIndex, radiusObject) -> None:
        self.xPos, self.yPos = initX, initY

        self.currAngleIndex = initAngleIndex
        self.currAngle = PLAYER_SETTING.CURR_ANGLE[self.currAngleIndex]

        self.radiusObject = radiusObject

    def move(self, action):  # t = 1
        if action == ACTIONS.FORWARD:
            self.xPos += math.cos(self.currAngle) * GAME_SETTING.GRID_WIDTH
            self.yPos += -math.sin(self.currAngle) * GAME_SETTING.GRID_WIDTH
        elif action == ACTIONS.TURN_LEFT:
            self.currAngleIndex = (self.currAngleIndex + 1) % 4
        elif action == ACTIONS.TURN_RIGHT:
            self.currAngleIndex = (self.currAngleIndex - 1) % 4
        elif action == ACTIONS.TURN_BACK:
            self.currAngleIndex = (self.currAngleIndex + 2) % 4
        else:
            pass

        self.currAngle = PLAYER_SETTING.CURR_ANGLE[self.currAngleIndex]

class Robot(Car):
    def __init__(self) -> None:
        super().__init__(
            initX=PLAYER_SETTING.INITIAL_X,
            initY=PLAYER_SETTING.INITIAL_Y,
            initAngleIndex=PLAYER_SETTING.INITIAL_ANGLE_INDEX,
            radiusObject=PLAYER_SETTING.RADIUS_OBJECT
        )

        self.isAlive = True
        self.achieveGoal = False
        # self.bug = 0
        # 360 lidar signal around robot
        # lidar return distance from robot to obstacles in range - return infinity if not have obstacle
        self.lidarSignals = [INT_INFINITY]*PLAYER_SETTING.CASTED_RAYS
        self.lidarVisualize = [{"source": {"x": self.xPos, "y": self.yPos},
                                "target": {"x": self.xPos, "y": self.yPos},
                                "color": COLOR.LIDAR_DEF_COLOR
                                } for x in range(PLAYER_SETTING.CASTED_RAYS)]

    def scanLidar(self, obstacles):
        obstaclesInRange = []  # to save obstacles in lidar range
        minDistance = INT_INFINITY

        for obstacle in obstacles.obstacles:
            check = Utils.isRobotWithinObstacle(obstacle, self.xPos, self.yPos)
            if check:
                minDistance = -1
                return minDistance
            distance = Utils.distanceBetweenTwoPoints(
                self.xPos, self.yPos, obstacle.xCenter, obstacle.yCenter)
            isInRangeLidar = distance < obstacle.radius + \
                PLAYER_SETTING.RADIUS_LIDAR
            if (isInRangeLidar == True):
                obstaclesInRange.append(obstacle)

        startAngle = self.currAngle - PLAYER_SETTING.HALF_PI

        for ray in range(PLAYER_SETTING.CASTED_RAYS):
            target_x = self.xPos + \
                math.cos(startAngle) * PLAYER_SETTING.RADIUS_LIDAR
            target_y = self.yPos - \
                math.sin(startAngle) * PLAYER_SETTING.RADIUS_LIDAR

            self.lidarVisualize[ray]["source"] = {
                "x": self.xPos,
                "y": self.yPos
            }

            distance = INT_INFINITY
            x = INT_INFINITY
            y = INT_INFINITY

            for obstacle in obstaclesInRange:
                d, x, y = Utils.getDistanceFromObject(
                    obstacle, self.xPos, self.yPos, target_x, target_y)

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
                self.lidarVisualize[ray]["color"] = COLOR.LIDAR_DEF_COLOR

            self.lidarVisualize[ray]["target"] = {
                "x": target_x,
                "y": target_y
            }

            startAngle += PLAYER_SETTING.STEP_ANGLE
            if startAngle > 2*PLAYER_SETTING.PI:
                startAngle = startAngle - 2*PLAYER_SETTING.PI

        # print(minDistance, minRay)
        return minDistance

    def checkCollision(self, distance):
        if distance <= self.radiusObject \
                or self.xPos < 0 or self.xPos > GAME_SETTING.SCREEN_WIDTH \
                or self.yPos < 0 or self.yPos > GAME_SETTING.SCREEN_HEIGHT:
            self.isAlive = False

    def checkAchieveGoal(self, goal):
        check = Utils.isRobotWithinObstacle(goal, self.xPos, self.yPos)
        if check:
            self.achieveGoal = True
            return -1
        distance = Utils.getDistanceFromObject(
            goal, self.xPos, self.yPos, goal.xCenter, goal.yCenter)
        if distance[0] <= self.radiusObject:
            self.achieveGoal = True

        return distance[0]

    def draw(self, screen):
        # cv2.circle(screen, (self.xPos, self.yPos), 370, COLOR.BLUE, -1)

        for lidarItemVisualize in self.lidarVisualize:
            color = lidarItemVisualize["color"]
            srcX = int(lidarItemVisualize["source"]["x"])
            srcY = int(lidarItemVisualize["source"]["y"])
            targetX = int(lidarItemVisualize["target"]["x"])
            targetY = int(lidarItemVisualize["target"]["y"])
            cv2.line(screen, (srcX, srcY), (targetX, targetY), color, 1)

        target_x = int(self.xPos +
                       math.cos(self.currAngle) * PLAYER_SETTING.RADIUS_LIDAR / 3 * 2)
        target_y = int(self.yPos -
                       math.sin(self.currAngle) * PLAYER_SETTING.RADIUS_LIDAR / 3 * 2)
        xSource = int(self.xPos)
        ySource = int(self.yPos)
        cv2.line(screen, (xSource, ySource),
                 (target_x, target_y), COLOR.BLACK, 2)
        print("xSource: ", xSource, "ySource: ", ySource, "target_x: ", target_x, "target_y: ", target_y)
        cv2.circle(screen, (xSource, ySource),
                   self.radiusObject, COLOR.BLUE, -1)


class PyGame2D():
    def __init__(self, screen, map) -> None:
        self.env = screen

        # init obstacles
        self.obstacles = self._initObstacle(numOfRect=OBSTACLE_SETTING.RECT_OBSTACLE,
                                            map=map)

        self.goal = self.obstacles.goal
        self.robot = Robot()
        self.generateEnvironment()

        # self.videoFile_path = getlogVideo_path(getlogVersion(base_path))
        # self.recordVideo = cv2.VideoWriter(self.videoFile_path,
        #                                    cv2.VideoWriter_fourcc(*'MJPG'),
        #                                    GAME_SETTING.FPS,
        #                                    (GAME_SETTING.SCREEN_WIDTH, GAME_SETTING.SCREEN_HEIGHT))
        # self.n = 0
        # self.elapsed_time = 0
        # self.totalTime = 0
        # self.minTime = INT_INFINITY
        # self.maxTime = 0

    def _initObstacle(self, numOfRect, map):
        return Obstacles(numOfRect, map)

    # def randomObstacle(self):
    #     x = Obstacles()
    #     x.randomObstacle()
    #     return x

    def _obstacleMoves(self):
        pass

    def action(self, action):
        self.robot.move(action=action)
        distance = self.robot.scanLidar(obstacles=self.obstacles)
        self.distanceGoal = self.robot.checkAchieveGoal(self.goal)
        self.robot.checkCollision(distance)

    def evaluate(self):
        reward = 0

        if not self.robot.isAlive:
            reward -= 1000000
        if self.robot.achieveGoal:
            reward += 100000

        observe = self.observe()
        goal_distance = observe[0]
        reward -= goal_distance*500

        if observe[2] == 0:
            reward -= 1000
        elif observe[2] == 1:
            reward += 10

        # if observe[1] == 0:
        #     reward -= 50
        # if observe[3] == 0:
        #     reward -= 50

        return reward

    def observe(self):
        a = self.robot.currAngle
        b = Utils.angleBetweenTwoPoints(
            self.robot.xPos, self.robot.yPos, self.goal.xCenter, self.goal.yCenter)
        # alpha = abs(a - b)
        alpha = a - b
        if alpha > PLAYER_SETTING.PI:
            alpha += -2*PLAYER_SETTING.PI
        elif alpha < -PLAYER_SETTING.PI:
            alpha += 2*PLAYER_SETTING.PI

        lidars = self.robot.lidarSignals
        lidars_NumberSelected = [lidars[0], lidars[90], lidars[180]]

        lidarLength_bin = [40]
        lidarLength_digitized = np.digitize(
            lidars_NumberSelected, lidarLength_bin)

        distanceGoal_space = PLAYER_SETTING.DISTANCEGOAL_MIN, PLAYER_SETTING.DISTANCEGOAL_MAX
        distanceGoal_bin = np.linspace(
            distanceGoal_space[0], distanceGoal_space[1], num=DISTANCE_SPACE, endpoint=False)
        distanceGoal_bin = np.delete(distanceGoal_bin, 0)
        infoStateVector = []
        infoStateVector.append(np.digitize(
            self.distanceGoal, distanceGoal_bin))

        infoStateVector = np.array(infoStateVector)
        lidarStateVector = np.array(lidarLength_digitized)
        # distance, lidar 0, 90, 180
        return np.concatenate((infoStateVector, lidarStateVector))

    def is_done(self):
        if self.robot.achieveGoal:
            return PLAYER_SETTING.GOAL
        elif not self.robot.isAlive:
            return PLAYER_SETTING.GONE
        return PLAYER_SETTING.ALIVE

    def record(self, screen, input):
        if input == 27:
            self.recordVideo.release()
        else:
            self.recordVideo.write(screen)

    def generateGrid(self, env):
        start_x = 0
        start_y = 0
        end_x = 0
        end_y = GAME_SETTING.SCREEN_HEIGHT

        while start_x < GAME_SETTING.SCREEN_WIDTH:
            cv2.line(env, (start_x, start_y), (end_x, end_y), COLOR.GRID_COLOR, 1)
            start_x = start_x + GAME_SETTING.GRID_WIDTH
            end_x = end_x + GAME_SETTING.GRID_WIDTH

        start_x = 0
        start_y = 0
        end_x = GAME_SETTING.SCREEN_WIDTH
        end_y = 0

        while start_y < GAME_SETTING.SCREEN_HEIGHT:
            cv2.line(env, (start_x, start_y), (end_x, end_y), COLOR.GRID_COLOR, 1)
            start_y = start_y + GAME_SETTING.GRID_WIDTH
            end_y = end_y + GAME_SETTING.GRID_WIDTH

    def generateEnvironment(self):
        self.generateGrid(self.env)
        self.obstacles.generateObstacles(self.env)
        self.goal.draw(self.env)

    def getEnv(self):
        return self.env

    def view(self):
        screen = self.env.copy()
        self.robot.draw(screen)
        # action = str(action)
        # cv2.putText(screen, action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        # self.record(screen, input)

        # return screen
        cv2.imshow('Enviroment', screen)
        # cv2.waitKey(1)

    def convert_lenLidar(self):
        self.robot.lidarSignals = np.array(self.robot.lidarSignals)
        self.robot.lidarSignals[self.robot.lidarSignals <= 120] = 2
        self.robot.lidarSignals[(self.robot.lidarSignals > 120) & (
            self.robot.lidarSignals <= 240)] = 1
        self.robot.lidarSignals[(self.robot.lidarSignals > 240) & (
            self.robot.lidarSignals <= 360)] = 0
        self.robot.lidarSignals[self.robot.lidarSignals > 360] = INT_INFINITY

    def convert_ObstacleDetectionArea(self):
        self.convert_lenLidar()


screen = np.ones((GAME_SETTING.SCREEN_HEIGHT,
                 GAME_SETTING.SCREEN_WIDTH, 3), dtype=np.uint8) * 255
game = PyGame2D(screen, MAP_SETTING.MAP_DEFAULT)
# game.view()
while True:
    input = Utils.inputUser()
    game.action(input)
    if game.robot.achieveGoal:
        print("Great!!!!!!!!!")
        input = 27
    elif not game.robot.isAlive:
        print("Oops!!!!!!!!!!")
        input = 27
    game.view()
    if input == 27:
        cv2.destroyAllWindows()
        break
    pass
