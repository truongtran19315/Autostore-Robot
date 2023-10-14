import pygame
import math
from const import *


class Utils:
	@staticmethod
	def blit_rotate_center(win, image, top_left, angle):
		rotated_image = pygame.transform.rotate(image, angle)
		new_rect = rotated_image.get_rect(
			center=image.get_rect(topleft=top_left).center)
		win.blit(rotated_image, new_rect.topleft)

	@staticmethod
	def distanceBetweenTwoPoints(xPointA, yPointA, xPointB, yPointB):
		return math.sqrt((xPointA - xPointB)**2 + (yPointA - yPointB)**2)

	@staticmethod
	def findLinePassTwoPoints(xPointA, yPointA, xPointB, yPointB):
		# y = ax + b
		if (xPointB - xPointA) == 0:
			# xPointB += 0.00001
			a = 0
		else:
			a = (yPointB - yPointA) / (xPointB - xPointA)
		b = yPointA - a * xPointA
		return a, b

	@staticmethod
	def findSolOfEquation(a, b, c):
		# aX^2 + bX + c = 0
		delta = b**2 - 4*a*c
		# print("delta: ", delta)
		if delta < 0:
			return EQUATION.NO_SOLUTION, 0, 0
		if delta == 0:
			return EQUATION.ONE_SOLUTION, -b/(2*a), -b/(2*a)
		else:
			return EQUATION.TWO_SOLUTION, (-b + math.sqrt(delta))/(2*a), (-b - math.sqrt(delta))/(2*a)
	
	@staticmethod
	def getDistanceFromObstacle(obstacle, xSource, ySource, xTarget, yTarget): 
		# xSource, ySource is the source of the lidar, xTarget, yTarget is the target of the lidar
		#     # if abs(xTarget - xCenter) < 0.0001:
		# x = xCenter
		# (xCenter - xObstacle)^2 + (y - yObstacle)^2 = r^2
		# Pt đường thẳng lidar y = ax + b
		a, b = Utils.findLinePassTwoPoints(xSource, ySource, xTarget, yTarget)
		# print("42: ", Utils.verifyLine(a, b, xCenter, yCenter))
		# print("41: ", Utils.verifyLine(a, b, xTarget, yTarget))
		# print("y = {}x + {}".format(a, b))
  
		d = INT_INFINITY

		if obstacle.shape == "circle":
			# (x - xObstacle)^2 + (y - yObstacle)^2 = r^2
			# (x - xObstacle)^2 + (a*x + b - yObstacle)^2 = r^2
			# x^2 - 2*x*xObstacle + xObstacle^2 + a^2*x^2 + 2*a*x*(b - yObstacle) + (b - yObstacle)^2 = r^2
			# Pt đường thẳng cắt hình tròn (a^2 + 1)x^2 - 2*(xObstacle - a*b + a*yObstacle)x + (b - yObstacle)**2 + xObstacle**2 - RADIUS_LIDAR**2 = 0
			a_temp = a**2 + 1
			b_temp = -2*obstacle.xCenter + 2*a*(b - obstacle.yCenter)
			c_temp = (b - obstacle.yCenter)**2 + obstacle.xCenter**2 - \
				obstacle.radius**2 # (PLAYER_SETTING.RADIUS_OBJECT)**2
			# print("a_temp = {}, b_temp = {}, c_temp = {}".format(a_temp, b_temp, c_temp))
			numberOfSolution, x1, x2 = Utils.findSolOfEquation(
				a_temp, b_temp, c_temp)

			if numberOfSolution == EQUATION.NO_SOLUTION:
				# print('NO_SOLUTION')
				pass
			elif numberOfSolution == EQUATION.ONE_SOLUTION:
				y1 = a*x1 + b
				if (x1 > xSource and x1 < xTarget) or (x1 < xSource and x1 > xTarget):
						d = Utils.distanceBetweenTwoPoints(xSource, ySource, x1, y1)
				# print('ONE_SOLUTION')
				# print("---> ", x1, a*x1 + b)

			else:
				# print('TWO_SOLUTION')
				y1 = a*x1 + b
				y2 = a*x2 + b
				if ((x1 > xSource and x1 < xTarget) or (x1 < xSource and x1 > xTarget)) \
						or ((x2 > xSource and x2 < xTarget) or (x2 < xSource and x2 > xTarget)):
						d = Utils.distanceBetweenTwoPoints(xSource, ySource, x1, y1)
						d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, x2, y2))
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
	
		elif obstacle.shape == "rectangle":
				
			topLeft = [obstacle.xCenter - obstacle.width//2, obstacle.yCenter - obstacle.height//2]
			topRight = [obstacle.xCenter + obstacle.width//2, obstacle.yCenter - obstacle.height//2]
			botLeft = [obstacle.xCenter - obstacle.width//2, obstacle.yCenter + obstacle.height//2]
			botRight = [obstacle.xCenter + obstacle.width//2, obstacle.yCenter + obstacle.height//2]
			
			a1, b1 = Utils.findLinePassTwoPoints(topLeft[0], topLeft[1], botLeft[0], botLeft[1])
			a2, b2 = Utils.findLinePassTwoPoints(botLeft[0], botLeft[1], botRight[0], botRight[1])
			a3, b3 = Utils.findLinePassTwoPoints(topRight[0], topRight[1], botRight[0], botRight[1])
			a4, b4 = Utils.findLinePassTwoPoints(topLeft[0], topLeft[1], topRight[0], topRight[1])
   
			if a == a1 and b == b1:
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, topLeft[0], topLeft[1]))
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, botLeft[0], botLeft[1]))
			elif a == a2 and b == b2:
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, botLeft[0], botLeft[1]))
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, botRight[0], botRight[1]))
			elif a == a3 and b == b3:
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, topRight[0], topRight[1]))
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, botRight[0], botRight[1]))
			elif a == a4 and b == b4:
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, topLeft[0], topLeft[1]))
				d = min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, topRight[0], topRight[1]))
			else:

				if a - a1 != 0:
					x1Point = (b1 - b) / (a - a1)
					y1Point = (a*b1 - a1*b) / (a - a1)
					if y1Point == x1Point * a1 + b1 and y1Point > topLeft[1] and y1Point < botLeft[1] \
       				and ((x1Point > xSource and x1Point < xTarget) or (x1Point < xSource and x1Point > xTarget)):
						d =  min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, x1Point, y1Point))
				
				if a - a2 != 0:
					x2Point = (b2 - b) / (a - a2)
					y2Point = (a*b2 - a2*b) / (a - a2)
					if x2Point > botLeft[0] and x2Point < botRight[0] and y2Point == x2Point * a2 + b2 \
       				and ((x2Point > xSource and x2Point < xTarget) or (x2Point < xSource and x2Point > xTarget)):
						d =  min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, x2Point, y2Point))
				
				if a - a3 != 0:
					x3Point = (b3 - b) / (a - a3)
					y3Point = (a*b3 - a3*b) / (a - a3)
					if y3Point == x3Point * a3 + b3 and y3Point > topRight[1] and y3Point < botRight[1] \
       				and ((x3Point > xSource and x3Point < xTarget) or (x3Point < xSource and x3Point > xTarget)):
						d =  min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, x3Point, y3Point))
				
				if a - a4 != 0:
					x4Point = (b4 - b) / (a - a4)
					y4Point = (a*b4 - a4*b) / (a - a4)                      
					if x4Point > topLeft[0] and x4Point < topRight[0] and y4Point == x4Point * a4 + b4 \
       				and ((x4Point > xSource and x4Point < xTarget) or (x4Point < xSource and x4Point > xTarget)):
						d =  min(d, Utils.distanceBetweenTwoPoints(xSource, ySource, x4Point, y4Point))
      
		return d
									
			
			
			

	@staticmethod
	def verifyLine(a, b, x, y):
		# y = ax + b
		return a*x + b - y

	@staticmethod
	def verifyCircle(xCenter, yCenter, radius, x, y):
		return (x - xCenter)**2 + (y - yCenter)**2 - radius**2

	@staticmethod
	def inputUser(game):
		keys = pygame.key.get_pressed()
		# Rotate left ()
		if keys[pygame.K_a]:
				game.action(ACTIONS.TURN_LEFT_ACCELERATION)
		# Rotate right ()
		elif keys[pygame.K_d]:
				game.action(ACTIONS.TURN_RIGHT_ACCELERATION)
		# Increase forward velocity
		elif keys[pygame.K_w]:
				game.action(ACTIONS.FORWARD_ACCELERATION)
		elif keys[pygame.K_x]:
				game.action(ACTIONS.BACKWARD_ACCELERATION)
		# Stop
		elif keys[pygame.K_s]:
				game.action(ACTIONS.STOP)
		else:
				game.action(ACTIONS.DO_NOTHING)
					
	@staticmethod
	def debugError(params):
		print(params)
