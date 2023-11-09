import cv2 
import numpy as np 
import math






currAngle = math.pi /2
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
radius = 10
xPos = 640
yPos = 360
lidarRadius = 360
stepAngle = 2*math.pi / 360
lidarVisualize = [{"source": {"x": xPos, "y": yPos},
                "target": {"x": xPos, "y": yPos},
                "color": BLUE
                } for x in range(360)]


def test1():
	screen = np.zeros((720, 1280, 3), dtype = np.uint8)  
	startAngle = currAngle
	for ray in range(360):
		xTarget = int(xPos + math.cos(startAngle) * lidarRadius)
		yTarget = int(yPos - math.sin(startAngle) * lidarRadius)
		lidarVisualize[ray]["target"]["x"] = xTarget
		lidarVisualize[ray]["target"]["y"] = yTarget
		if ray >= 0 and ray <= 6 or ray >= 354 and ray <= 360:
			lidarVisualize[ray]["color"] = RED
		elif ray > 6 and ray <= 57 or ray >= 303 and ray < 354:
			lidarVisualize[ray]["color"] = YELLOW
		elif ray > 57 and ray <= 90 or ray >= 270 and ray < 303:
			lidarVisualize[ray]["color"] = GREEN
		xSource = lidarVisualize[ray]["source"]["x"]
		ySource = lidarVisualize[ray]["source"]["y"]
		color = lidarVisualize[ray]["color"]
		cv2.line(screen, (xSource, ySource), (xTarget, yTarget), color, 1)
		startAngle += stepAngle
		if startAngle > 2 * math.pi:
			startAngle = startAngle - 2*math.pi

	cv2.circle(screen, (640, 360), 10, (255, 255, 255), -1)
	cv2.circle(screen, (640, 210), 50, (0, 0, 0), -1) 
	

	cv2.imshow('test1', screen)
	cv2.waitKey(0)
 
def test2():
	screen = np.zeros((720, 1280, 3), dtype = np.uint8)  
	startAngle = currAngle + 1
	for ray in range(360):
		xTarget = int(xPos + math.cos(startAngle) * lidarRadius)
		yTarget = int(yPos - math.sin(startAngle) * lidarRadius)
		lidarVisualize[ray]["target"]["x"] = xTarget
		lidarVisualize[ray]["target"]["y"] = yTarget
		if ray >= 0 and ray <= 6 or ray >= 354 and ray <= 360:
			lidarVisualize[ray]["color"] = RED
		elif ray > 6 and ray <= 57 or ray >= 303 and ray < 354:
			lidarVisualize[ray]["color"] = YELLOW
		elif ray > 57 and ray <= 90 or ray >= 270 and ray < 303:
			lidarVisualize[ray]["color"] = GREEN
		xSource = lidarVisualize[ray]["source"]["x"]
		ySource = lidarVisualize[ray]["source"]["y"]
		color = lidarVisualize[ray]["color"]
		cv2.line(screen, (xSource, ySource), (xTarget, yTarget), color, 1)
		startAngle += stepAngle
		if startAngle > 2 * math.pi:
			startAngle = startAngle - 2*math.pi

	cv2.circle(screen, (640, 360), 10, (255, 255, 255), -1)
	cv2.circle(screen, (640, 210), 50, (0, 0, 0), -1) 

	cv2.imshow('test2', screen)
	cv2.waitKey(0)
 
def test3():
	screen = np.zeros((720, 1280, 3), dtype = np.uint8)  
	startAngle = currAngle - 1
	for ray in range(360):
		xTarget = int(xPos + math.cos(startAngle) * lidarRadius)
		yTarget = int(yPos - math.sin(startAngle) * lidarRadius)
		lidarVisualize[ray]["target"]["x"] = xTarget
		lidarVisualize[ray]["target"]["y"] = yTarget
		if ray >= 0 and ray <= 6 or ray >= 354 and ray <= 360:
			lidarVisualize[ray]["color"] = RED
		elif ray > 6 and ray <= 57 or ray >= 303 and ray < 354:
			lidarVisualize[ray]["color"] = YELLOW
		elif ray > 57 and ray <= 90 or ray >= 270 and ray < 303:
			lidarVisualize[ray]["color"] = GREEN
		xSource = lidarVisualize[ray]["source"]["x"]
		ySource = lidarVisualize[ray]["source"]["y"]
		color = lidarVisualize[ray]["color"]
		cv2.line(screen, (xSource, ySource), (xTarget, yTarget), color, 1)
		startAngle += stepAngle
		if startAngle > 2 * math.pi:
			startAngle = startAngle - 2*math.pi

	cv2.circle(screen, (640, 360), 10, (255, 255, 255), -1)
	cv2.circle(screen, (640, 210), 50, (0, 0, 0), -1) 

	cv2.imshow('test3', screen)
	cv2.waitKey(0)
 
 
 
 
# test1()
test2()
# test3()