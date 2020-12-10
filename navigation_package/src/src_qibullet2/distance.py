import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

def theta_calcul(image_length, person_position):
	HFOV = 56.3
	theta = HFOV*person_position/image_length
	return theta

def calcul_coordinates(x_pepper, y_pepper, alpha_pepper, distance, image_length, person_position):
	theta = theta_calcul(image_length, person_position)
	x_person = x_pepper + distance*math.cos((alpha_pepper - theta)*3.14/180)
	y_person = y_pepper + distance*math.sin((alpha_pepper - theta)*3.14/180)
	return x_person, y_person

def coord(x_pepper, y_pepper, alpha_pepper, distance, image_length, person_position):
	x_person, y_person = calcul_coordinates(x_pepper, y_pepper, alpha_pepper, distance, image_length, person_position)
	plt.xlim(-1, 15)
	plt.ylim(-5, 5)
	plt.scatter(x_pepper, y_pepper, c = 'blue')
	plt.scatter(x_person, y_person, c = 'red')
	plt.savefig("coord", dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches=None, pad_inches=0.1, frameon=None, metadata=None)

if __name__=='__main__':

	depthImage = np.load('depthImage.npy')
	coordinates = open("coordinates.txt", "r")
	lines=coordinates.readlines()
	img = cv2.imread('imageRGB.png')
	height, width, channels = img.shape
	
	j=0
	for i in range(len(lines)/2):
		x=int(lines[j])
		y=int(lines[j+1])
		distance = int(depthImage[y][x])*0.001
		coord(0, 0, 0, distance, width, x-width/2)
		cv2.putText(img, str(distance)+"m", (x-50,y+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
		j=j+2

	coordinates.close()

	cv2.imwrite('imageRGB.png', img)