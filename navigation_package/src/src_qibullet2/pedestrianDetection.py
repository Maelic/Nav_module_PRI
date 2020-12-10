import rospy
from sensor_msgs.msg import Image
import os
import cv2
from cv_bridge import CvBridge
from PIL import Image as image
import imutils 
import numpy as np

#openCV images
bridge = CvBridge()

# Initializing the HOG person detector 
hog = cv2.HOGDescriptor() 
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector()) 

def callback(msg):

	#Names files
	image_name="imageRGB.png"
	coordinates_file_name="coordinates.txt"

	#Get the image
	image = bridge.imgmsg_to_cv2(msg, "bgr8")

	#Resizing the image 
	image = imutils.resize(image, width=min(400, image.shape[1]))

	# Detecting all the regions in the image that has a pedestrians inside it 
	(regions, _) = hog.detectMultiScale(image, winStride=(4, 4), padding=(4, 4), scale=1.05)

	#Open the coordinates file
	coordinates_file = open(coordinates_file_name, "w")

	# Drawing the regions in the Image 
	for (x, y, w, h) in regions:
		#Draw a rectangle on pedestrian
		cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
		#Center of the rectangle
		cx=x+w/2
		cy=y+h/4
		cv2.circle(image, (cx, cy), 0, (0, 0, 255), 10)
		#Write coordinates in the specific file
		coordinates_file.write(str(cx)+"\n")
		coordinates_file.write(str(cy)+"\n")

	#Close the coordinates file
	coordinates_file.close()

	#Save th image
	cv2.imwrite(image_name, image)

if __name__=='__main__':
	rospy.init_node('pedestrianDetection')
	rospy.Subscriber("/naoqi_driver/camera/front/image_raw",Image, callback)
	rospy.spin()