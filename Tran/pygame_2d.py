import cv2
import numpy as np
import math
import random
from obstacles import Obstacles, Goal
from consts import *
from utils import Utils
import cythonUtils
import time
from logVersion import *


class Car():
    def __init__(self, initX, initY, maxForwardVelocity, minRotationVelocity, maxRotationVelocity, accelerationForward, accelerationRotate, radiusObject) -> None:
        self.xPos, self.yPos = initX, initY
        self.maxForwardVelocity = maxForwardVelocity
        self.maxRotationVelocity = maxRotationVelocity  # rotate to the right
        self.minRotationVelocity = minRotationVelocity  # //      left

        self.currentForwardVelocity = 0  # always >= 0
        self.currRotationVelocity = 0  # rotate left < 0, rotate right > 0

        self.currAngle = math.pi / 2
        # self.currAngle = math.pi
        self.accelerationForward = accelerationForward
        self.accelerationRotate = accelerationRotate

        self.radiusObject = radiusObject

    def move(self, action):  # t = 1
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
            if self.currRotationVelocity != self.maxRotationVelocity:
                self.currRotationVelocity = round(
                    self.currRotationVelocity + self.accelerationRotate, 2)
            if self.currRotationVelocity >= self.maxRotationVelocity:
                self.currRotationVelocity = self.maxRotationVelocity
        elif action == ACTIONS.TURN_RIGHT_ACCELERATION:
            if self.currRotationVelocity != self.minRotationVelocity:
                self.currRotationVelocity = round(
                    self.currRotationVelocity - self.accelerationRotate, 2)
            if self.currRotationVelocity <= self.minRotationVelocity:
                self.currRotationVelocity = self.minRotationVelocity
        elif action == ACTIONS.STOP:
            self.currentForwardVelocity = 0
            self.currRotationVelocity = 0
        else:
            pass

        # Calculate the position base on velocity per frame
        # dt = float(1/GAME_SETTING.FPS)
        dt = 1

        self.currAngle += (self.currRotationVelocity*dt)

        # Prevent car go to the opposite way
        if (self.currAngle < 0):
            self.currAngle = 2*math.pi - abs(self.currAngle)
        elif (self.currAngle > 2*math.pi):
            self.currAngle = abs(self.currAngle - 2*math.pi)

        # Update the new position based on the velocity
        self.xPos += math.cos(self.currAngle) * \
            self.currentForwardVelocity * dt
        self.yPos += -math.sin(self.currAngle) * \
            self.currentForwardVelocity * dt

        # print(self.xPos, self.yPos, " current angle: ", self.currAngle, self.currentForwardVelocity, self.currRotationVelocity)


class Robot(Car):
    def __init__(self) -> None:
        super().__init__(
            # initX=random.randint(900, 1265),
            # initY=random.randint(15, 705),
            initX=PLAYER_SETTING.INITIAL_X,
            initY=PLAYER_SETTING.INITIAL_Y,
            maxForwardVelocity=PLAYER_SETTING.MAX_FORWARD_VELO,
            minRotationVelocity=PLAYER_SETTING.MIN_ROTATION_VELO,
            maxRotationVelocity=PLAYER_SETTING.MAX_ROTATION_VELO,
            accelerationForward=PLAYER_SETTING.ACCELERATION_FORWARD,
            accelerationRotate=PLAYER_SETTING.ACCELERATION_ROTATE,
            radiusObject=PLAYER_SETTING.RADIUS_OBJECT
        )

        self.isAlive = True
        self.achieveGoal = False
        # 360 lidar signal around robot
        # lidar return distance from robot to obstacles in range - return infinity if not have obstacle
        self.lidarSignals = [INT_INFINITY]*PLAYER_SETTING.CASTED_RAYS
        self.lidarVisualize = [{"source": {"x": self.xPos, "y": self.yPos},
                                "target": {"x": self.xPos, "y": self.yPos},
                                "color": COLOR.WHITE
                                } for x in range(PLAYER_SETTING.CASTED_RAYS)]

    def scanLidar(self, obstacles):
        obstaclesInRange = []  # to save obstacles in lidar range
        minDistance = INT_INFINITY

        for obstacle in obstacles.obstacles:
            distance = Utils.distanceBetweenTwoPoints(
                self.xPos, self.yPos, obstacle.xCenter, obstacle.yCenter)
            isInRageLidar = distance < obstacle.radius + \
                PLAYER_SETTING.RADIUS_LIDAR
            if (isInRageLidar == True):
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
                d, x, y = cythonUtils.getDistanceFromObstacle(
                    obstacle, self.xPos, self.yPos, target_x, target_y)
                # d, x, y = Utils.getDistanceFromObstacle(obstacle, self.xPos, self.yPos, target_x, target_y)

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
            if startAngle > 2*PLAYER_SETTING.PI:
                startAngle = startAngle - 2*PLAYER_SETTING.PI

        # print(minDistance, minRay)
        return minDistance

    def checkCollision(self, distance):
        if distance <= self.radiusObject \
                or self.xPos < 12 or self.xPos > 1268 \
                or self.yPos < 8 or self.yPos > 708:
            self.isAlive = False

    def checkAchieveGoal(self, goal):
        distance = Utils.distanceBetweenTwoPoints(
            self.xPos, self.yPos, goal.xCenter, goal.yCenter)
        if distance < self.radiusObject + goal.radius:
            self.achieveGoal = True

        return distance

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
                       math.cos(self.currAngle) * PLAYER_SETTING.RADIUS_LIDAR / 3)
        target_y = int(self.yPos -
                       math.sin(self.currAngle) * PLAYER_SETTING.RADIUS_LIDAR / 3)
        xSource = int(self.xPos)
        ySource = int(self.yPos)
        cv2.line(screen, (xSource, ySource),
                 (target_x, target_y), COLOR.WHITE, 5)
        cv2.circle(screen, (xSource, ySource),
                   self.radiusObject, COLOR.BLUE, -1)


class PyGame2D():
    def __init__(self, screen) -> None:
        self.env = screen
        self.obstacles = self._initObstacle()
        self.goal = Goal()
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

    def _initObstacle(self):
        return Obstacles()

    def _obstacleMoves(self):
        pass

    def action(self, action):
        self.robot.move(action=action)
        # self._obstacleMoves()
        # self.n += 1
        # start_time = time.time()

        distance = self.robot.scanLidar(obstacles=self.obstacles)

        # end_time = time.time()

        # elapsed_time = end_time - start_time

        # self.totalTime += elapsed_time
        # mediumTime = self.totalTime / self.n

        # print(elapsed_time, mediumTime, self.minTime, self.maxTime, self.n)

        self.robot.checkCollision(distance)
        self.robot.checkAchieveGoal(self.goal)

    def evaluate(self):
        reward = 0
        if not self.robot.isAlive:
            reward -= 10000000

        if self.robot.achieveGoal:
            reward += 10000000

        # distance = self.observe()[-4:]
        observe = self.observe()
        distance = observe[-4:]
        direction = observe[0]
        if distance[1] == 0:
            reward -= 500
            # print('-150 huong thang co do dai = 0')
        elif distance[1] == 1:
            reward -= 300
            # print('-70 huong thang co do dai = 1')
        elif distance[1] == 2:
            reward -= 100
            # print('-20 huong thang co do dai = 2')
        elif distance[1] > 2:
            # reward += 200
            pass
            # print('+70 huong thang khong co vat can')

        if distance[0] == 0:
            reward -= 8
            # print('-80 huong 2 ben co do dai = 0')
        elif distance[0] == 1:
            reward -= 4
            # print('-40 huong 2 ben co do dai = 1')
        elif distance[0] == 2:
            reward -= 1
            # print('-15 huong 2 ben co do dai = 2')
        elif distance[0] > 2:
            # reward += 18
            pass
            # print('+180 huong 2 ben khong co vat can')
            
        if distance[2] == 0:
            reward -= 8
            # print('-80 huong 2 ben co do dai = 0')
        elif distance[2] == 1:
            reward -= 4
            # print('-40 huong 2 ben co do dai = 1')
        elif distance[2] == 2:
            reward -= 1
            # print('-15 huong 2 ben co do dai = 2')
        elif distance[2] > 2:
            # reward += 18
            pass
            # print('+180 huong 2 ben khong co vat can')
            
        if self.robot.currentForwardVelocity == 0:
            reward -= 500
            
        # direction = self.observe()[0] 
        reward -= (abs(direction - int(ALPHA_SPACE/2)) * 100)    
            
        far_from_goal = self.robot.checkAchieveGoal(goal=self.goal)
        reward -= (int(far_from_goal) + 500)
        # print(direction, observe[1], reward)

        return reward

    def observe(self):
        # ratioLeft = (self.robot.xPos)/(GAME_SETTING.SCREEN_WIDTH)
        a = self.robot.currAngle
        b = Utils.angleBetweenTwoPoints(self.robot.xPos, self.robot.yPos, self.goal.xCenter, self.goal.yCenter)
        alpha = a - b
        # print(a, b, alpha)
        fwVelo = self.robot.currentForwardVelocity
        rVelo = self.robot.currRotationVelocity
        lidars = self.robot.lidarSignals

        # TODO
        #! chuyen gia tri tia lidar
        bin_lidarsignal = np.linspace(
            0, 360, num=LENGTH_LIDARSIGNAL, endpoint=True)
        bin_lidarsignal = np.delete(bin_lidarsignal, 0)
        lidars_digitized = np.digitize(lidars, bin_lidarsignal)

        bin_lidarspace = np.array([60, 120, 180])
        lidars_sections = np.array_split(lidars_digitized, bin_lidarspace)
        section_lidars_min = [np.amin(section) for section in lidars_sections]

        #! chuyen doi state vector
        high, low = 1, 0
        alpha_space = -PLAYER_SETTING.PI, PLAYER_SETTING.PI
        fwVelo_space = 0, PLAYER_SETTING.MAX_FORWARD_VELO
        rVelo_space = PLAYER_SETTING.MIN_ROTATION_VELO, PLAYER_SETTING.MAX_ROTATION_VELO

        lowState = np.array(
            [alpha_space[0], fwVelo_space[0], rVelo_space[0]], dtype=float)
        upState = np.array(
            [alpha_space[1], fwVelo_space[1], rVelo_space[1]], dtype=float)

        infoState_shape = (ALPHA_SPACE, FWVELO_SPACE, RVELO_SPACE)

        bins = []
        for i in range(3):
            item = np.linspace(
                lowState[i],
                upState[i],
                num=infoState_shape[i],
                endpoint=False)
            item = np.delete(item, 0)
            bins.append(item)

        s = (alpha, fwVelo, rVelo)
        infoStateVector = []
        for i in range(3):
            infoStateVector.append(np.digitize(s[i], bins[i]))

        infoStateVector = np.array(infoStateVector)
        lidarStateVector = np.array(section_lidars_min)
        return np.concatenate((infoStateVector, lidarStateVector))

    def is_done(self):
        if not self.robot.isAlive:
            return PLAYER_SETTING.GONE
        elif self.robot.achieveGoal:
            return PLAYER_SETTING.GOAL
        return PLAYER_SETTING.ALIVE

    def record(self, screen, input):
        if input == 27:
            self.recordVideo.release()
        else:
            self.recordVideo.write(screen)

    def generateEnvironment(self):
        self.obstacles.generateObstacles(self.env)
        self.goal.draw(self.env)
        
    def getEnv(self):
        return self.env

    def view(self, counter, action):
        screen = self.env.copy()
        self.robot.draw(screen)
        action = str(action)
        cv2.putText(screen, action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        # self.record(screen, input)
        
        return screen
        # cv2.imshow('Enviroment', screen)
        # cv2.waitKey(0)

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


# screen = np.zeros((720, 1280, 3), dtype=np.uint8)
# game = PyGame2D(screen)
# # game.view()
# while True:
#     input = Utils.inputUser()
#     game.action(input)
#     if not game.robot.isAlive:
#         print("Oops!!!!!!!!!!")
#         input = 27
#     elif game.robot.achieveGoal:
#         print("Great!!!!!!!!!")
#         input = 27
#     game.view(input)
#     if input == 27:
#         cv2.destroyAllWindows()
#         break
#     pass
