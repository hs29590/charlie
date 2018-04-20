#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('charlie')
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


class ImageProcessor:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/raspicam_node/image_raw",Image,self.callback)
 
    #print("Param is", rospy.get_param('default_param', 'default_value'));
    show_images_from_param = rospy.get_param('~show_images', 'False')
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~show_images'), show_images_from_param)
    self.showImages = bool(show_images_from_param);

    self.image = None;
    self.SCALE_FACTOR = 4;
    self.gray_image = None;
    self.prevCx = None;
    self.direction = 0
    self.lineFound = False;
    self.err_pub = rospy.Publisher('line_error', Float32, queue_size=1)
    self.line_state_pub = rospy.Publisher('line_visible', Bool, queue_size = 1);
    self.intersection_pub = rospy.Publisher('intersection_visible', Bool, queue_size = 1);

  def callback(self,msg):

    #start_time = timeit.default_timer()

    kernel = numpy.ones((3,3),numpy.uint8)
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    image = cv2.resize(image,(int(320/self.SCALE_FACTOR),int(240/self.SCALE_FACTOR)));
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    hue_img, sat_img, v_img = cv2.split(hsv)  # extracting red channel
   
    # Get the Red from the Hue Image. It ranges between 0,10 and 170,180
    lower_hue = numpy.array([0])
    upper_hue = numpy.array([10])

    lower_hue_wrap = numpy.uint8([170])
    upper_hue_wrap = numpy.uint8([180])

    hue_mask = cv2.inRange(hue_img, lower_hue, upper_hue);
    hue_mask_wrap = cv2.inRange(hue_img, lower_hue_wrap, upper_hue_wrap);
    hue_mask = cv2.bitwise_or(hue_mask, hue_mask_wrap);

    #lower_red = numpy.uint8([0, 0, 100])
    #upper_red = numpy.uint8([30, 70, 255])
    #red_mask = cv2.inRange(image, lower_red, upper_red);
    
    #masked_intersection = cv2.bitwise_and(hue_mask, hue_mask, mask = red_mask)
    
    cv2.imshow("hue mask", hue_mask);
    cv2.waitKey(3);

    n_white_pix = numpy.sum(hue_mask == 255)
    if(n_white_pix > 60):
        #print("Intersection Detected");
	self.intersection_pub.publish(True)
    else:
        #print("No Intersection");
	self.intersection_pub.publish(False)


    lower_sat = numpy.array([220])
    upper_sat = numpy.array([255])

    lower_yellow_hue = numpy.array([20])
    upper_yellow_hue = numpy.array([30])

    yellow_hue_mask = cv2.inRange(hue_img, lower_yellow_hue, upper_yellow_hue);

    cv2.imshow('yellow hue', yellow_hue_mask);
    cv2.waitKey(3)

    sat_mask = cv2.inRange(sat_img, lower_sat, upper_sat);
    yellow_hsv_mask = cv2.bitwise_or(sat_mask, yellow_hue_mask)
    
    cv2.imshow('yellow hue + sat', yellow_hsv_mask);
    cv2.waitKey(3)


    lower = numpy.uint8([0, 120, 120])
    upper = numpy.uint8([255, 255, 255])

    #yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow);
    #masked = cv2.bitwise_and(image, image, mask = yellow_mask)

    yellow_mask = cv2.inRange(image, lower, upper);
    if(self.showImages):
        cv2.imshow('sat',sat_mask);
        cv2.waitKey(3);
    #sat_masked = cv2.bitwise_and(image,image,mask=sat_mask);
    #masked = cv2.bitwise_and(sat_mask, sat_mask, mask = yellow_mask)
    masked = cv2.bitwise_and(sat_mask, yellow_mask)

    masked=cv2.GaussianBlur(masked,(3,3),2)#blur the grayscale image
    masked = cv2.morphologyEx(masked,cv2.MORPH_ERODE, kernel)
    #yellow_mask = cv2.inRange(image, lower, upper)
    if(self.showImages):
        cv2.imshow('y',masked);
        cv2.waitKey(3)
    #mask = closing;
    mask = masked;

    h, w, d = image.shape
    search_top = 0;
    search_bot = 2*h/3;
    search_left = w/4;
    search_right = 3*w/4;
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    mask[0:h, 0:search_left] = 0
    mask[0:h, search_right:w] = 0
    if(self.showImages):
        cv2.imshow("a",mask);
        cv2.waitKey(3);
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        #The proportional controller is implemented in the following four lines which
        #is reposible of linear scaling of an error to drive the control output.
        cv2.circle(image, (int(cx), int(cy)), 5, (0,0,255), -1)  

	
        if self.prevCx is None:
            self.prevCx = cx;
    
#smoothing cx

        cx = self.prevCx*0.5 + cx*0.5;

        self.prevCx = cx;

        err = cx - w/2

        err = self.SCALE_FACTOR * err;

        self.line_state_pub.publish(True);

    else: #Moment not available. Probably, line isn't there
        err = -1000.0;
        self.line_state_pub.publish(False);
        self.err_pub.publish(err);
    
    if(self.showImages):
        cv2.imshow("window", image)
        cv2.waitKey(3)

def main(args):
  rospy.init_node('image_processor', anonymous=True)
  ic = ImageProcessor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)













