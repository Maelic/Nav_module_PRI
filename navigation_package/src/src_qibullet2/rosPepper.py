#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import pybullet, pybullet_data
import cv2, random, time, base64, numpy as np

from qibullet import SimulationManager
from qibullet import PepperRosWrapper
from qibullet import PepperVirtual
from qibullet import Camera
from threading import Thread
from threading import Event
from Queue import Queue
from multiprocessing import Pool
import time
import subprocess
import shutil

class Cam(Thread):

    def __init__(self,pepper, queue):
        Thread.__init__(self)
        stopEvent = 0
        self.pepper = pepper
        self.q = queue
        self.event = Event()

        
    def run(self):
        i=1
        while True:
            #Subscribe to RGB camera, wait, then unsubscribe
            #handle = 
            self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP, resolution=Camera.K_VGA)
            #time.sleep(1)
            self.pepper.unsubscribeCamera(self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP, resolution=Camera.K_VGA))

            #Subscribe to depth camera, wait, then unsubscribe
            #handle = 
            self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH, resolution=Camera.K_VGA)
            #time.sleep(1)
            self.pepper.unsubscribeCamera(self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH, resolution=Camera.K_VGA))
            subprocess.call(["python", "distance.py"])
            shutil.copy("/home/antoine/Documents/WORK/EDUCATION_ONLINE_CHALLENGE/Project/src_qibullet2/imageRGB.png","/home/antoine/Documents/WORK/EDUCATION_ONLINE_CHALLENGE/Project/src_qibullet2/img/imageRGB"+str(i)+".png")
            shutil.copy("/home/antoine/Documents/WORK/EDUCATION_ONLINE_CHALLENGE/Project/src_qibullet2/coord.png","/home/antoine/Documents/WORK/EDUCATION_ONLINE_CHALLENGE/Project/src_qibullet2/img/coord"+str(i)+".png")
            i=i*10

    def stop(self):
        self.Terminated = True
          
if __name__ == "__main__":
    wrapper = PepperRosWrapper()
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    q = Queue()

    # Load environment

    pybullet.loadURDF("./models/wall.urdf", basePosition = [2,8,0], globalScaling = 0.17, physicsClientId = client)
    pybullet.loadURDF("./models/wall.urdf", basePosition = [-2,8,0], globalScaling = 0.17, physicsClientId = client)
    pybullet.loadURDF("./models/wall.urdf", basePosition = [6,8,0], globalScaling = 0.17, physicsClientId = client)

    pybullet.loadURDF("./models/man.urdf", basePosition = [2,4,0], globalScaling = 0.2, physicsClientId = client)
    pybullet.loadURDF("./models/woman.urdf", basePosition = [0,6,0], globalScaling = 0.2, physicsClientId = client)

    #Chair
    pybullet.loadURDF("./models/chair4.urdf", basePosition = [3,1,0], globalScaling = 0.15, physicsClientId = client)

    #Woman sitdown 
    woman2 = pybullet.loadURDF("./models/woman2.urdf", basePosition = [3,0,1], globalScaling = 10, physicsClientId = client)

    #PAS TOUCHER (rotation de la femme pour qu'elle soit dans le bon sens de la chaise)
    pos, orn = pybullet.getBasePositionAndOrientation(woman2)
    y2x = pybullet.getQuaternionFromEuler([-3.14/2, 0, 0])

    newpos, neworn = pybullet.multiplyTransforms([0, 0, 0],y2x,pos,orn)
    pybullet.resetBasePositionAndOrientation(woman2, newpos, neworn)

    #spawn du robot
    quaternion = pybullet.getQuaternionFromEuler([0, 0, 1.6])
    pepper = simulation_manager.spawnPepper(client, translation=[2, -4, 0], quaternion=quaternion, spawn_ground_plane=True)

    wrapper.launchWrapper(pepper, "naoqi_driver")

    pepper.goToPosture("Stand", 1.0)
    pepper.setAngles('HipPitch', -0.2, 1)

    #lancement du thread caméra
    camThread = Cam(pepper, q)
    camThread.start()

    #On fait bouger le robot
    # pour changer la distance modifier le 5 (1m = 1 carreau)
    # pour changer la speed modifier le 8.0 (vitesse non linéaire = lent au début et à la fin, comme dans la réalité)
    pepper.moveTo(5, 0, 0, frame=PepperVirtual.FRAME_ROBOT, speed = 0.0000000000000001) 
    camThread.stop()
    subprocess.call(["mencoder", "mf://img/imageRGB*png", "-mf", "fps=7", "-ovc", "xvid", "-xvidencopts", "bitrate=1200", "-o", "distance.avi"])
    subprocess.call(["mencoder", "mf://img/coord*png", "-mf", "fps=7", "-ovc", "xvid", "-xvidencopts", "bitrate=1200", "-o", "coord.avi"])

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
