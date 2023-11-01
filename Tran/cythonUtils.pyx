import math
from consts import *
from obstacles import *

class CythonUtils:
	@staticmethod
	def distanceBetweenTwoPoints(float xPointA, float yPointA, float xPointB, float yPointB):
		return math.sqrt((xPointA - xPointB)**2 + (yPointA - yPointB)**2)

	@staticmethod
	def findLinePassTwoPoints(float xPointA, float yPointA, float xPointB, float yPointB):
		# y = ax + b
		cdef bool c = False # c = true khi đường thẳng đứng (góc = +- 90)
		if (xPointB - xPointA) == 0:
			xPointB += 0.00001
			c = True
		cdef float a = (yPointB - yPointA) / (xPointB - xPointA)
		cdef float b = yPointA - a * xPointA
		return a, b, c

	@staticmethod
	def findSolOfEquation(float a, float b, float c):
		# aX^2 + bX + c = 0
		cdef float delta = b**2 - 4*a*c
		# print("delta: ", delta)
		if delta < 0:
			return EQUATION.NO_SOLUTION, 0, 0
		if delta == 0:
			return EQUATION.ONE_SOLUTION, -b/(2*a), -b/(2*a)
		else:
			return EQUATION.TWO_SOLUTION, (-b + math.sqrt(delta))/(2*a), (-b - math.sqrt(delta))/(2*a)

	@staticmethod
	def getDistanceFromObstacle(Obstacles obstacle, float xSource, float ySource, float xTarget, float yTarget): 
    
		cdef float distance = INT_INFINITY
		cdef float xPoint = INT_INFINITY
		cdef float yPoint = INT_INFINITY

		# xSource, ySource is the source of the lidar, xTarget, yTarget is the target of the lidar
		#     # if abs(xTarget - xCenter) < 0.0001:
		# x = xCenter
		# (xCenter - xObstacle)^2 + (y - yObstacle)^2 = r^2
		# Pt đường thẳng lidar y = ax + b
		cdef float a, b
		cdef bool c
		(a, b, c) = CythonUtils.findLinePassTwoPoints(xSource, ySource, xTarget, yTarget)
		# print("42: ", Utils.verifyLine(a, b, xCenter, yCenter))
		# print("41: ", Utils.verifyLine(a, b, xTarget, yTarget))
		# print("y = {}x + {}".format(a, b))

		if obstacle.shape == "circle":
			if c == True:
				cdef float x = xSource
				
				cdef float a_temp = 1
				cdef float b_temp = -2*obstacle.yCenter
				cdef float c_temp = (x - obstacle.xCenter)**2 + obstacle.yCenter**2 - \
				obstacle.radius**2 # (PLAYER_SETTING.RADIUS_OBJECT)**2
				# print("a_temp = {}, b_temp = {}, c_temp = {}".format(a_temp, b_temp, c_temp))
				cdef float numberOfSolution, y1, y2
				numberOfSolution, y1, y2 = CythonUtils.findSolOfEquation(
				a_temp, b_temp, c_temp)

				if numberOfSolution == EQUATION.NO_SOLUTION:
				# print('NO_SOLUTION')
					pass
				elif numberOfSolution == EQUATION.ONE_SOLUTION:
					if (y1 >= ySource and y1 <= yTarget) or (y1 <= ySource and y1 >= yTarget):
						distance =  CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x, y1)
						xPoint = x
						yPoint = y1
					# print('ONE_SOLUTION')
					# print("---> ", x1, a*x1 + b)

				else:
					if ((y1 >= ySource and y1 <= yTarget) or (y1 <= ySource and y1 >= yTarget)) \
						or ((y2 >= ySource and y2 <= yTarget) or (y2 <= ySource and y2 >= yTarget)):
						cdef float d1 =  CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x, y1)
						cdef float d2 =  CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x, y2)
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
				cdef float a_temp = a**2 + 1
				cdef float b_temp = -2*obstacle.xCenter + 2*a*(b - obstacle.yCenter)
				cdef float c_temp = (b - obstacle.yCenter)**2 + obstacle.xCenter**2 - \
				obstacle.radius**2 # (PLAYER_SETTING.RADIUS_OBJECT)**2
				# print("a_temp = {}, b_temp = {}, c_temp = {}".format(a_temp, b_temp, c_temp))
				cdef float numberOfSolution, x1, x2
				numberOfSolution, x1, x2 =  CythonUtils.findSolOfEquation(
				a_temp, b_temp, c_temp)

				if numberOfSolution == EQUATION.NO_SOLUTION:
					# print('NO_SOLUTION')
					pass
				elif numberOfSolution == EQUATION.ONE_SOLUTION:
					cdef float y1 = a*x1 + b
					if (x1 >= xSource and x1 <= xTarget) or (x1 <= xSource and x1 >= xTarget):
						distance =  CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x1, y1)
						xPoint = x1
						yPoint = y1
					# print('ONE_SOLUTION')
					# print("---> ", x1, a*x1 + b)

				else:
					# print('TWO_SOLUTION')
					cdef float y1 = a*x1 + b
					cdef float y2 = a*x2 + b
					if ((x1 >= xSource and x1 <= xTarget) or (x1 <= xSource and x1 >= xTarget)) \
						or ((x2 >= xSource and x2 <= xTarget) or (x2 <= xSource and x2 >= xTarget)):
						cdef float d1 =  CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x1, y1)
						cdef float d2 =  CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x2, y2)
						if d1 < d2:
							distance = d1
							xPoint = x1
							yPoint = y1
						else:
							distance = d2
							xPoint = x2
							yPoint = y2

		elif obstacle.shape == "rectangle":
			
			cdef float[2] topLeft = [obstacle.xCenter - obstacle.width//2, obstacle.yCenter - obstacle.height//2]
			cdef float[2] topRight = [obstacle.xCenter + obstacle.width//2, obstacle.yCenter - obstacle.height//2]
			cdef float[2] botLeft = [obstacle.xCenter - obstacle.width//2, obstacle.yCenter + obstacle.height//2]
			cdef float[2] botRight = [obstacle.xCenter + obstacle.width//2, obstacle.yCenter + obstacle.height//2]
			
			cdef float a2, b2, c2, a4, b4, c4
			# left, bot, right, top		
			# x1Point = topLeft[0] # phương trình đường thẳng song song với trục tung x = a
			# a1, b1, c1 = Utils.findLinePassTwoPoints(topLeft[0], topLeft[1], botLeft[0], botLeft[1])
			a2, b2, c2 = CythonUtils.findLinePassTwoPoints(botLeft[0], botLeft[1], botRight[0], botRight[1]) # a = 0, b = y
			# x3Point = topRight[0]
			# a3, b3, c3 = Utils.findLinePassTwoPoints(topRight[0], topRight[1], botRight[0], botRight[1])
			a4, b4, c4 = CythonUtils.findLinePassTwoPoints(topLeft[0], topLeft[1], topRight[0], topRight[1])

			# Trường hợp tia thẳng đứng -> chỉ cắt 2 cạnh ngang
			if c == True:
				cdef float x2Point = xSource
				cdef float y2Point = a2*x2Point + b2
			
				cdef float x4Point = xSource
				cdef float y4Point = a4*x4Point + b4
			
				if x2Point >= botLeft[0] and x2Point <= botRight[0] \
					and ((y2Point >= ySource and y2Point <= yTarget) or (y2Point <= ySource and y2Point >= yTarget)):
					cdef float d1 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x2Point, y2Point)
					cdef float d2 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x4Point, y4Point)
				if d1 < d2:
					distance = d1
					xPoint = x2Point
					yPoint = y2Point
				else:
					distance = d2
					xPoint = x4Point
					yPoint = y4Point
			
			# Trường hợp tia trùng với cạnh
			# elif a == a1 and b == b1:
			# 	d1 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, topLeft[0], topLeft[1])
			# 	d2 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, botLeft[0], botLeft[1])
			# 	if d1 < d2:
			# 		distance = d1
			# 		xPoint = topLeft[0]
			# 		yPoint = topLeft[1]
			# 	else:
			# 		distance = d2
			# 		xPoint = botLeft[0]
			# 		yPoint = botLeft[1]
			elif a == a2 and b == b2:
				cdef float d1 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, botLeft[0], botLeft[1])
				cdef float d2 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, botRight[0], botRight[1])
				if d1 < d2:
					distance = d1
					xPoint = botLeft[0]
					yPoint = botLeft[1]
				else:
					distance = d2
					xPoint = botRight[0]
					yPoint = botRight[1]
			# elif a == a3 and b == b3:
			# 	d1 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, topRight[0], topRight[1])
			# 	d2 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, botRight[0], botRight[1])
			# 	if d1 < d2:
			# 		distance = d1
			# 		xPoint = topRight[0]
			# 		yPoint = topRight[1]
			# 	else:
			# 		distance = d2
			# 		xPoint = botRight[0]
			# 		yPoint = botRight[1]
			elif a == a4 and b == b4:
				cdef float d1 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, topLeft[0], topLeft[1])
				cdef float d2 = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, topRight[0], topRight[1])
				if d1 < d2:
					distance = d1
					xPoint = topLeft[0]
					yPoint = topLeft[1]
				else:
					distance = d2
					xPoint = topRight[0]
					yPoint = topRight[1]
			else:
			
				cdef float x1Point = topLeft[0]
				cdef float y1Point = a*x1Point + b
				# print(x1Point, y1Point)
				# print(y1Point == x1Point * a1 + b1)
				if y1Point >= topLeft[1] and y1Point <= botLeft[1] \
					and ((x1Point >= xSource and x1Point <= xTarget) or (x1Point <= xSource and x1Point >= xTarget)):
					# print(x1Point, y1Point, topLeft[1], botLeft[1], xSource, ySource, xTarget, yTarget)
					distance =  CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x1Point, y1Point)
					xPoint = x1Point
					yPoint = y1Point
				
			
				cdef float x3Point = topRight[0]
				cdef float y3Point = a*x3Point + b
				if y3Point >= topRight[1] and y3Point <= botRight[1] \
					and ((x3Point >= xSource and x3Point <= xTarget) or (x3Point <= xSource and x3Point >= xTarget)):
					cdef float d = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x3Point, y3Point)
				if d < distance:
					distance = d
					xPoint = x3Point
					yPoint = y3Point
				
				if a - a2 != 0 and a - a4 != 0:
					cdef float x2Point = (b2 - b) / (a - a2)
					cdef float y2Point = b2
				if x2Point >= botLeft[0] and x2Point <= botRight[0] \
					and ((x2Point >= xSource and x2Point <= xTarget) or (x2Point <= xSource and x2Point >= xTarget)):
					cdef float d = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x2Point, y2Point)
					if d < distance:
						distance = d
						xPoint = x2Point
						yPoint = y2Point
				cdef float x4Point = (b4 - b) / (a - a4)
				cdef float y4Point = b4                     
				if x4Point >= topLeft[0] and x4Point <= topRight[0] \
					and ((x4Point >= xSource and x4Point <= xTarget) or (x4Point <= xSource and x4Point >= xTarget)):
					cdef float d = CythonUtils.distanceBetweenTwoPoints(xSource, ySource, x4Point, y4Point)
					if d < distance:
						distance = d
						xPoint = x4Point
						yPoint = y4Point
		
		# print(d)
		return distance, int(xPoint), int(yPoint)
		
