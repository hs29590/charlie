#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import timeit
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy
import math
import time

def main(args):
  rospy.init_node('lineasdf_extractor', anonymous=True)
  bridge = CvBridge()
  image_pub = rospy.Publisher('image', Image, queue_size=1);
 
  cap = cv2.VideoCapture(0)

  while not rospy.is_shutdown():
    ret, frame = cap.read()
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame',frame)
    cv2.waitKey(2);
    image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"));
    #time.sleep(0.5);

if __name__ == '__main__':
    main(sys.argv)

