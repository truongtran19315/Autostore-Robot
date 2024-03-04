import pygame
import math
from consts import *
import cv2


class Utils:
        
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
    
    @staticmethod
    def isRobotWithinObstacle(obstacle, xPointA, yPointA):
        topLeft = [obstacle.xCenter - obstacle.width // 2 , \
            obstacle.yCenter - obstacle.height//2]
        xtopRight = obstacle.xCenter + obstacle.width // 2
        ybotLeft = obstacle.yCenter + obstacle.height//2
        if xPointA >= topLeft[0] and xPointA <= xtopRight \
            and yPointA >= topLeft[1] and yPointA <= ybotLeft:
                return True
        return False
    
    @staticmethod
    def length2RightAngleEdge(goal, xPointA, yPointA):
        return abs(xPointA - goal.xCenter) + abs(yPointA - goal.yCenter)

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
    def getDistanceFromObject(obstacle, xSource, ySource, xTarget, yTarget): 

        distance = INT_INFINITY
        xPoint = INT_INFINITY
        yPoint = INT_INFINITY

        # xSource, ySource is the source of the lidar, xTarget, yTarget is the target of the lidar
        #     # if abs(xTarget - xCenter) < 0.0001:
        # x = xCenter
        # (xCenter - xObstacle)^2 + (y - yObstacle)^2 = r^2
        # Pt đường thẳng lidar y = ax + b
        a, b, c = Utils.findLinePassTwoPoints(xSource, ySource, xTarget, yTarget)
            
        topLeft = [obstacle.xCenter - obstacle.width//2, obstacle.yCenter - obstacle.height//2]
        topRight = [obstacle.xCenter + obstacle.width//2, obstacle.yCenter - obstacle.height//2]
        botLeft = [obstacle.xCenter - obstacle.width//2, obstacle.yCenter + obstacle.height//2]
        botRight = [obstacle.xCenter + obstacle.width//2, obstacle.yCenter + obstacle.height//2]
        
        # left, bot, right, top		
        # x1Point = topLeft[0] # phương trình đường thẳng song song với trục tung x = a
        # a1, b1, c1 = Utils.findLinePassTwoPoints(topLeft[0], topLeft[1], botLeft[0], botLeft[1])
        a2, b2, c2 = Utils.findLinePassTwoPoints(botLeft[0], botLeft[1], botRight[0], botRight[1]) # a = 0, b = y
        # x3Point = topRight[0]
        # a3, b3, c3 = Utils.findLinePassTwoPoints(topRight[0], topRight[1], botRight[0], botRight[1])
        a4, b4, c4 = Utils.findLinePassTwoPoints(topLeft[0], topLeft[1], topRight[0], topRight[1])

        # Trường hợp tia thẳng đứng -> chỉ cắt 2 cạnh ngang
        if c == True:
            x2Point = xSource
            y2Point = b2
        
            x4Point = xSource
            y4Point = b4
        
            if x2Point >= botLeft[0] and x2Point <= botRight[0] \
                    and ((y2Point >= ySource and y2Point <= yTarget) or (y2Point <= ySource and y2Point >= yTarget) \
                                or (y4Point >= ySource and y4Point <= yTarget) or (y4Point <= ySource and y4Point >= yTarget)):
                d1 = Utils.distanceBetweenTwoPoints(xSource, ySource, x2Point, y2Point)
                d2 = Utils.distanceBetweenTwoPoints(xSource, ySource, x4Point, y4Point)
                if d1 < d2:
                    distance = d1
                    xPoint = x2Point
                    yPoint = y2Point
                else:
                    distance = d2
                    xPoint = x4Point
                    yPoint = y4Point
        
        # Trường hợp tia trùng với cạnh
        elif a == a2 and b == b2:
            d1 = Utils.distanceBetweenTwoPoints(xSource, ySource, botLeft[0], botLeft[1])
            d2 = Utils.distanceBetweenTwoPoints(xSource, ySource, botRight[0], botRight[1])
            if d1 < d2 and d1 <= PLAYER_SETTING.RADIUS_LIDAR:
                distance = d1
                xPoint = botLeft[0]
                yPoint = botLeft[1]
            elif d1 > d2 and d2 <= PLAYER_SETTING.RADIUS_LIDAR:
                distance = d2
                xPoint = botRight[0]
                yPoint = botRight[1]
        elif a == a2 and b == b4:
            d1 = Utils.distanceBetweenTwoPoints(xSource, ySource, topLeft[0], topLeft[1])
            d2 = Utils.distanceBetweenTwoPoints(xSource, ySource, topRight[0], topRight[1])
            if d1 < d2 and d1 <= PLAYER_SETTING.RADIUS_LIDAR:
                distance = d1
                xPoint = topLeft[0]
                yPoint = topLeft[1]
            elif d1 > d2  and d2 <= PLAYER_SETTING.RADIUS_LIDAR:
                distance = d2
                xPoint = topRight[0]
                yPoint = topRight[1]
                
        else:
            # left edge	
            x1Point = topLeft[0]
            y1Point = a*x1Point + b
            # print(x1Point, y1Point)
            # print(y1Point == x1Point * a1 + b1)
            if y1Point >= topLeft[1] and y1Point <= botLeft[1] \
                and ((x1Point >= xSource and x1Point <= xTarget) or (x1Point <= xSource and x1Point >= xTarget)):
                # print(x1Point, y1Point, topLeft[1], botLeft[1], xSource, ySource, xTarget, yTarget)
                distance =  Utils.distanceBetweenTwoPoints(xSource, ySource, x1Point, y1Point)
                xPoint = x1Point
                yPoint = y1Point
            
            # right edge
            x3Point = topRight[0]
            y3Point = a*x3Point + b
            if y3Point >= topRight[1] and y3Point <= botRight[1] \
                and ((x3Point >= xSource and x3Point <= xTarget) or (x3Point <= xSource and x3Point >= xTarget)):
                d = Utils.distanceBetweenTwoPoints(xSource, ySource, x3Point, y3Point)
                if d < distance:
                    distance = d
                    xPoint = x3Point
                    yPoint = y3Point
            
            if a - a2 != 0 and a - a4 != 0:
                # bot edge
                x2Point = (b2 - b) / (a - a2)
                y2Point = b2
                if x2Point >= botLeft[0] and x2Point <= botRight[0] \
                    and ((x2Point >= xSource and x2Point <= xTarget) or (x2Point <= xSource and x2Point >= xTarget)):
                    d = Utils.distanceBetweenTwoPoints(xSource, ySource, x2Point, y2Point)
                    if d < distance:
                        distance = d
                        xPoint = x2Point
                        yPoint = y2Point
                        
                # top edge
                x4Point = (b4 - b) / (a - a4)
                y4Point = b4                     
                if x4Point >= topLeft[0] and x4Point <= topRight[0] \
                    and ((x4Point >= xSource and x4Point <= xTarget) or (x4Point <= xSource and x4Point >= xTarget)):
                    d = Utils.distanceBetweenTwoPoints(xSource, ySource, x4Point, y4Point)
                    if d < distance:
                        distance = d
                        xPoint = x4Point
                        yPoint = y4Point
	
	    # print(d)
        return distance, xPoint, yPoint

    @staticmethod
    def inputUser():
        key = cv2.waitKey(delay=1)
        # Rotate left ()
        if key == ord('a'):
            # game.action(ACTIONS.TURN_LEFT_ACCELERATION)
            return ACTIONS.TURN_LEFT
        # Rotate right ()
        elif key == ord('d'):
            # game.action(ACTIONS.TURN_RIGHT_ACCELERATION)
            return ACTIONS.TURN_RIGHT
        # Increase forward velocity
        elif key == ord('w'):
            # game.action(ACTIONS.FORWARD_ACCELERATION)
            return ACTIONS.FORWARD
        elif key == ord('s'):
            # game.action(ACTIONS.BACKWARD_ACCELERATION)
            return ACTIONS.TURN_BACK
        # Stop
        elif key == 27 or (key & 0xFF == 'q'):
            return 27
        else:
            # game.action(ACTIONS.DO_NOTHING)
            return ACTIONS.DO_NOTHING
        