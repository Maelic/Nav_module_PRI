import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils

cvb=CvBridge()

def imnormalize(xmax,image):
    """
    Normalize a list of sample image data in the range of 0 to 1
    : image:image data.
    : return: Numpy array of normalize data
    """
    xmin = 0
    a = 0
    b = 255
    
    return ((np.array(image,dtype=np.float32) - xmin) * (b - a)) / (xmax - xmin)

def checkdepth(msg):

    seq=msg.header.seq
    npy_name="depthImage.npy"
    jpg_name="depthImage.jpg"

    cv_image = cvb.imgmsg_to_cv2(msg,msg.encoding)

    cv_image = imutils.resize(cv_image, width=min(400, cv_image.shape[1]))

    image_normal= np.array(imnormalize(np.max(cv_image),cv_image),dtype=np.uint8)

    numpy_image= np.array(cv_image,dtype=np.uint16)

    depth_mem = np.copy(numpy_image)
    np.save(npy_name,numpy_image)
    #cv2.imwrite(jpg_name, image_normal)
    
if __name__ == '__main__':
    rospy.init_node("depthDetection")
    rospy.Subscriber("/naoqi_driver/camera/depth/image_raw",Image,checkdepth)
    rospy.spin()