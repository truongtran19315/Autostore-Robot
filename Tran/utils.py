import pygame
import math
from consts import *
import cv2


class Utils:
    @staticmethod
    def blit_rotate_center(win, image, top_left, angle):
        rotated_image = pygame.transform.rotate(image, angle)
        new_rect = rotated_image.get_rect(
            center=image.get_rect(topleft=top_left).center)
        win.blit(rotated_image, new_rect.topleft)
        
    @staticmethod
    def angleBetweenTwoPoints(xPointA, yPointA, xPointB, yPointB):
        delta_x = xPointB - xPointA
        delta_y = yPointB - yPointA
        # print('a', delta_y, delta_x)
        radian_angle = math.atan2(delta_y, delta_x)
        if radian_angle < 0:
            radian_angle = -radian_angle
        else: radian_angle = PLAYER_SETTING.PI * 2 - radian_angle
        # degree_angle = math.degrees(radian_angle)
        return radian_angle

    @staticmethod
    def distanceBetweenTwoPoints(xPointA, yPointA, xPointB, yPointB):
        return math.sqrt((xPointA - xPointB)**2 + (yPointA - yPointB)**2)
    
    def isPointInObstacle(obstacle, xPointA, yPointA):
        if obstacle.shape == 'circle':
            distance = Utils.distanceBetweenTwoPoints(
                    xPointA, yPointA, obstacle.xCenter, obstacle.yCenter)
            if distance <= PLAYER_SETTING.RADIUS_OBJECT + obstacle.radius:
                return True
        else:
            topLeft = [obstacle.xCenter - obstacle.width // 2 - PLAYER_SETTING.RADIUS_OBJECT, \
                obstacle.yCenter - obstacle.height//2 - PLAYER_SETTING.RADIUS_OBJECT]
            topRight = obstacle.xCenter + obstacle.width // 2 + PLAYER_SETTING.RADIUS_OBJECT
            botLeft = obstacle.yCenter + obstacle.height//2 + PLAYER_SETTING.RADIUS_OBJECT
            if xPointA > topLeft[0] and xPointA < topRight \
                and yPointA > topLeft[1] and yPointA < botLeft:
                    return True
        return False
            

    @staticmethod
    def findLinePassTwoPoints(xPointA, yPointA, xPointB, yPointB):
        # y = ax + b
        c = False  # c = true khi đường thẳng đứng (góc = +- 90)
        if (xPointB - xPointA) == 0:
            xPointB += 0.00001
            c = True
        a = (yPointB - yPointA) / (xPointB - xPointA)
        if a > 5730 or a < -5730:  # tan of 89.99 and 90.01
            c = True
        b = yPointA - a * xPointA
        # print("find line ", a, b, c, maxA)
        return a, b, c

    @staticmethod
    def findSolOfEquation(a, b, c):
        # aX^2 + bX + c = 0
        delta = b**2 - 4*a*c
        # print("delta: ", delta)
        if delta < 0:
            # print("0 nghiem ", delta)
            return EQUATION.NO_SOLUTION, 0, 0
        if delta == 0:
            # print("1 nghiem ", delta, (-b)/(2*a), (-b)/(2*a))
            return EQUATION.ONE_SOLUTION, -b/(2*a), -b/(2*a)
        else:
            # print("2 nghiem ", delta, (-b + math.sqrt(delta))/(2*a), (-b - math.sqrt(delta))/(2*a))
            return EQUATION.TWO_SOLUTION, (-b + math.sqrt(delta))/(2*a), (-b - math.sqrt(delta))/(2*a)

    @staticmethod
    def getDistanceFromObstacle(obstacle, xSource, ySource, xTarget, yTarget):

        distance = INT_INFINITY
        xPoint = INT_INFINITY
        yPoint = INT_INFINITY

        # xSource, ySource is the source of the lidar, xTarget, yTarget is the target of the lidar
        #     # if abs(xTarget - xCenter) < 0.0001:
        # x = xCenter
        # (xCenter - xObstacle)^2 + (y - yObstacle)^2 = r^2
        # Pt đường thẳng lidar y = ax + b
        a, b, c = Utils.findLinePassTwoPoints(
            xSource, ySource, xTarget, yTarget)
        # print("42: ", Utils.verifyLine(a, b, xCenter, yCenter))
        # print("41: ", Utils.verifyLine(a, b, xTarget, yTarget))
        # print("y = {}x + {}".format(a, b))

        if obstacle.shape == "circle":
            if c == True:
                x = xSource

                a_temp = 1
                b_temp = -2*obstacle.yCenter
                c_temp = (x - obstacle.xCenter)**2 + obstacle.yCenter**2 - \
                    obstacle.radius**2  # (PLAYER_SETTING.RADIUS_OBJECT)**2
                # print("a_temp = {}, b_temp = {}, c_temp = {}".format(a_temp, b_temp, c_temp))
                numberOfSolution, y1, y2 = Utils.findSolOfEquation(
                    a_temp, b_temp, c_temp)

                if numberOfSolution == EQUATION.NO_SOLUTION:
                    # print('NO_SOLUTION')
                    pass
                elif numberOfSolution == EQUATION.ONE_SOLUTION:
                    if (y1 >= ySource and y1 <= yTarget) or (y1 <= ySource and y1 >= yTarget):
                        distance = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x, y1)
                        xPoint = x
                        yPoint = y1
                    # print('ONE_SOLUTION')
                    # print("---> ", x1, a*x1 + b)

                else:
                    if ((y1 >= ySource and y1 <= yTarget) or (y1 <= ySource and y1 >= yTarget)) \
                            or ((y2 >= ySource and y2 <= yTarget) or (y2 <= ySource and y2 >= yTarget)):
                        d1 = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x, y1)
                        d2 = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x, y2)
                        if d1 < d2:
                            distance = d1
                            xPoint = x
                            yPoint = y1
                        else:
                            distance = d2
                            xPoint = x
                            yPoint = y2
            else:
                # (x - xObstacle)^2 + (y - yObstacle)^2 = r^2
                # (x - xObstacle)^2 + (a*x + b - yObstacle)^2 = r^2
                # x^2 - 2*x*xObstacle + xObstacle^2 + a^2*x^2 + 2*a*x*(b - yObstacle) + (b - yObstacle)^2 = r^2
                # Pt đường thẳng cắt hình tròn (a^2 + 1)x^2 - 2*(xObstacle - a*b + a*yObstacle)x + (b - yObstacle)**2 + xObstacle**2 - RADIUS_LIDAR**2 = 0
                a_temp = a**2 + 1
                b_temp = -2*obstacle.xCenter + 2*a*(b - obstacle.yCenter)
                c_temp = (b - obstacle.yCenter)**2 + obstacle.xCenter**2 - \
                    obstacle.radius**2  # (PLAYER_SETTING.RADIUS_OBJECT)**2
                # print("a_temp = {}, b_temp = {}, c_temp = {}".format(a_temp, b_temp, c_temp))
                numberOfSolution, x1, x2 = Utils.findSolOfEquation(
                    a_temp, b_temp, c_temp)

                if numberOfSolution == EQUATION.NO_SOLUTION:
                    # print('NO_SOLUTION')
                    pass
                elif numberOfSolution == EQUATION.ONE_SOLUTION:
                    y1 = a*x1 + b
                    if (x1 >= xSource and x1 <= xTarget) or (x1 <= xSource and x1 >= xTarget):
                        distance = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x1, y1)
                        xPoint = x1
                        yPoint = y1
                    # print('ONE_SOLUTION')
                    # print("---> ", x1, a*x1 + b)

                else:
                    # print('TWO_SOLUTION')
                    y1 = a*x1 + b
                    y2 = a*x2 + b
                    if ((x1 >= xSource and x1 <= xTarget) or (x1 <= xSource and x1 >= xTarget)) \
                            or ((x2 >= xSource and x2 <= xTarget) or (x2 <= xSource and x2 >= xTarget)):
                        d1 = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x1, y1)
                        d2 = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x2, y2)
                        if d1 < d2:
                            distance = d1
                            xPoint = x1
                            yPoint = y1
                        else:
                            distance = d2
                            xPoint = x2
                            yPoint = y2
                    # print("---> ", x1, a*x1 + b)
                    # print("---> ", x2, a*x2 + b)
                    # print(Utils.distanceBetweenTwoPoints(x1, a*x1 + b, x2, a*x2 + b))
                    # print('---check---')
                    # print(Utils.verifyLine(a, b, x1, a*x1 + b))
                    # print(Utils.verifyCircle(xObstacle, yObstacle, PlayerParam.RADIUS_OBJECT, x1, a*x1 + b))
                    # print(Utils.verifyLine(a, b, x2, a*x2 + b))
                    # print(Utils.verifyCircle(xObstacle, yObstacle, PlayerParam.RADIUS_OBJECT, x2, a*x2 + b))
                    # print('---length---')
                    # print(d1, d2)

        elif obstacle.shape == "rectangle" or obstacle.shape == "wall":

            topLeft = [obstacle.xCenter - obstacle.width //
                       2, obstacle.yCenter - obstacle.height//2]
            topRight = [obstacle.xCenter + obstacle.width //
                        2, obstacle.yCenter - obstacle.height//2]
            botLeft = [obstacle.xCenter - obstacle.width //
                       2, obstacle.yCenter + obstacle.height//2]
            botRight = [obstacle.xCenter + obstacle.width //
                        2, obstacle.yCenter + obstacle.height//2]

            # left, bot, right, top
            # x1Point = topLeft[0] # phương trình đường thẳng song song với trục tung x = a
            # a1, b1, c1 = Utils.findLinePassTwoPoints(topLeft[0], topLeft[1], botLeft[0], botLeft[1])
            a2, b2, c2 = Utils.findLinePassTwoPoints(
                # a = 0, b = y
                botLeft[0], botLeft[1], botRight[0], botRight[1])
            # x3Point = topRight[0]
            # a3, b3, c3 = Utils.findLinePassTwoPoints(topRight[0], topRight[1], botRight[0], botRight[1])
            a4, b4, c4 = Utils.findLinePassTwoPoints(
                topLeft[0], topLeft[1], topRight[0], topRight[1])

            # Trường hợp tia thẳng đứng -> chỉ cắt 2 cạnh ngang
            if c == True:
                x2Point = xSource
                y2Point = a2*x2Point + b2

                x4Point = xSource
                y4Point = a4*x4Point + b4

                if x2Point >= botLeft[0] and x2Point <= botRight[0] \
                    and ((y2Point >= ySource and y2Point <= yTarget) or (y2Point <= ySource and y2Point >= yTarget)
                         or (y4Point >= ySource and y4Point <= yTarget) or (y4Point <= ySource and y4Point >= yTarget)):
                    d1 = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, x2Point, y2Point)
                    d2 = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, x4Point, y4Point)
                    if d1 < d2:
                        distance = d1
                        xPoint = x2Point
                        yPoint = y2Point
                    else:
                        distance = d2
                        xPoint = x4Point
                        yPoint = y4Point

            # Trường hợp tia trùng với cạnh
            elif a == a2:
                if b == b2:
                    d1 = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, botLeft[0], botLeft[1])
                    d2 = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, botRight[0], botRight[1])
                    if d1 < d2 and d1 <= PLAYER_SETTING.RADIUS_LIDAR:
                        distance = d1
                        xPoint = botLeft[0]
                        yPoint = botLeft[1]
                    elif d1 > d2 and d2 <= PLAYER_SETTING.RADIUS_LIDAR:
                        distance = d2
                        xPoint = botRight[0]
                        yPoint = botRight[1]
                elif b == b4:
                    d1 = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, topLeft[0], topLeft[1])
                    d2 = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, topRight[0], topRight[1])
                    if d1 < d2 and d1 <= PLAYER_SETTING.RADIUS_LIDAR:
                        distance = d1
                        xPoint = topLeft[0]
                        yPoint = topLeft[1]
                    elif d1 > d2 and d2 <= PLAYER_SETTING.RADIUS_LIDAR:
                        distance = d2
                        xPoint = topRight[0]
                        yPoint = topRight[1]
            else:

                x1Point = topLeft[0]
                y1Point = a*x1Point + b
                # print(x1Point, y1Point)
                # print(y1Point == x1Point * a1 + b1)
                if y1Point >= topLeft[1] and y1Point <= botLeft[1] \
                        and ((x1Point >= xSource and x1Point <= xTarget) or (x1Point <= xSource and x1Point >= xTarget)):
                    # print(x1Point, y1Point, topLeft[1], botLeft[1], xSource, ySource, xTarget, yTarget)
                    distance = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, x1Point, y1Point)
                    xPoint = x1Point
                    yPoint = y1Point

                x3Point = topRight[0]
                y3Point = a*x3Point + b
                if y3Point >= topRight[1] and y3Point <= botRight[1] \
                        and ((x3Point >= xSource and x3Point <= xTarget) or (x3Point <= xSource and x3Point >= xTarget)):
                    d = Utils.distanceBetweenTwoPoints(
                        xSource, ySource, x3Point, y3Point)
                    if d < distance:
                        distance = d
                        xPoint = x3Point
                        yPoint = y3Point

                if a - a2 != 0 and a - a4 != 0:
                    x2Point = (b2 - b) / (a - a2)
                    y2Point = b2
                    if x2Point >= botLeft[0] and x2Point <= botRight[0] \
                            and ((x2Point >= xSource and x2Point <= xTarget) or (x2Point <= xSource and x2Point >= xTarget)):
                        d = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x2Point, y2Point)
                        if d < distance:
                            distance = d
                            xPoint = x2Point
                            yPoint = y2Point
                    x4Point = (b4 - b) / (a - a4)
                    y4Point = b4
                    if x4Point >= topLeft[0] and x4Point <= topRight[0] \
                            and ((x4Point >= xSource and x4Point <= xTarget) or (x4Point <= xSource and x4Point >= xTarget)):
                        d = Utils.distanceBetweenTwoPoints(
                            xSource, ySource, x4Point, y4Point)
                        if d < distance:
                            distance = d
                            xPoint = x4Point
                            yPoint = y4Point

        # print("distance ", distance, xPoint, yPoint)
        return distance, xPoint, yPoint

    @staticmethod
    def verifyLine(a, b, x, y):
        # y = ax + b
        return a*x + b - y

    @staticmethod
    def verifyCircle(xCenter, yCenter, radius, x, y):
        return (x - xCenter)**2 + (y - yCenter)**2 - radius**2

    @staticmethod
    def inputUser():
        key = cv2.waitKey(delay=1)
        # Rotate left ()
        if key == ord('a'):
            # game.action(ACTIONS.TURN_LEFT_ACCELERATION)
            return ACTIONS.TURN_LEFT_ACCELERATION
        # Rotate right ()
        elif key == ord('d'):
            # game.action(ACTIONS.TURN_RIGHT_ACCELERATION)
            return ACTIONS.TURN_RIGHT_ACCELERATION
        # Increase forward velocity
        elif key == ord('w'):
            # game.action(ACTIONS.FORWARD_ACCELERATION)
            return ACTIONS.FORWARD_ACCELERATION
        elif key == ord('x'):
            # game.action(ACTIONS.BACKWARD_ACCELERATION)
            return ACTIONS.BACKWARD_ACCELERATION
        # Stop
        elif key == ord('s'):
            # game.action(ACTIONS.STOP)
            return ACTIONS.STOP
        elif key == 27 or (key & 0xFF == 'q'):
            return 27
        else:
            # game.action(ACTIONS.DO_NOTHING)
            return ACTIONS.DO_NOTHING

    @staticmethod
    def debugError(params):
        print(params)
