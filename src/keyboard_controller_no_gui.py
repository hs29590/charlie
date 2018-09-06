#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('charlie')
import sys
import rospy
from planner import Planner
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from ca_msgs.msg import Mode
from ca_msgs.msg import ChargingState
from threading import Lock

import tf
import time
import numpy
import math
from subprocess import call
#from Tkinter import *
import threading

class DriveCreate2: 

  def __init__(self):
    
    self.timeOfLastActivity = rospy.Time.now();
    self.isAsleep = False;

    self.LINEAR_SPEED = 0.8;   
    self.state = "Stop"
    self.TIME_FOR_MOVING_TOWARDS_INTERSECTION = 2; #in seconds, this should also change with the LINEAR SPEED
    self.TIME_FOR_TURNING = 4; #in seconds, this should also change with the LINEAR_SPEED


    #Publishers
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.tone_pub = rospy.Publisher('buzzer1/tone', Int32, queue_size = 1)
    self.dock_pub = rospy.Publisher('/dock', Empty, queue_size = 1);
    self.undock_pub = rospy.Publisher('/undock', Empty, queue_size = 1);
    
    self.intersectionCodeMutex = Lock()

    #Planner
    self.pathPlanner = Planner();

    #Robot Variables

    self.docked = False;
    self.battery = "N/A"
    self.twist = Twist()
    self.yaw = None;
    self.last_odom = Odometry();
    self.sonar_drive = True;
    self.line_drive = True;
    self.odomRecd = False;
    self.last_drive_lin = 0.0;
    self.last_drive_ang = 0.0;
    self.noLineCount = 0;
    self.intersectionVisibleCount = 0;
    
    self.line_err = -1000.0;
    self.intersection_err = -1000.0;

    self.currentIntersectionCode = '';

    self.timeOfLastIntersection = rospy.Time.now();

    self.STOP_TONE = 2;
    self.FOLLOW_TONE = 5;
    self.currentPath = None;
    self.currentPathIndex = 0;

    #Subscribers
    self.bat_sub = rospy.Subscriber('/battery/charge_ratio', Float32, self.batteryCallback)
    self.line_visible_sub = rospy.Subscriber('line_visible', Bool, self.lineVisibleCallback)
    self.current_mode_sub = rospy.Subscriber('mode', Mode, self.current_mode_callback);
    self.err_sub = rospy.Subscriber('line_error', Float32, self.errCallback)
    self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
    self.sonar_sub = rospy.Subscriber('sonar_drive', Bool, self.sonarCallback);
    self.intersection_sub = rospy.Subscriber('/intersection_err', Float32, self.intersectionCallback); 
    self.qrcode_sub = rospy.Subscriber('/barcode_data', String, self.qrCodeCallback);
   
    self.timeOfLastActivity = rospy.Time.now();
    self.timeOfLastIntersection = rospy.Time.now();

    self.sourceSelected = None;
    self.destinationSelected = None;

    self.runThread = threading.Thread(target=self.runThreadFunc)
    self.runThread.daemon = True
      
  def resetPressed(self):
      self.timeOfLastActivity = rospy.Time.now();
      print("Reset doesn't work currently..")
      
  def goAhead(self):
      if(self.isAsleep):
          call(["rosservice", "call", "/raspicam_node/start_capture"]);
          self.isAsleep = False;
          rospy.loginfo("Coming out of sleep");

      self.timeOfLastActivity = rospy.Time.now();

      if(self.sourceSelected is not None and self.destinationSelected is not None):
          print("Source Selected: " + self.sourceSelected);
          print("Destination Selected: " + self.destinationSelected);
          self.pathPlanner.calculatePath();
          print("Path returned with length: "),
          self.currentPath = self.pathPlanner.getLeftRightTurnList();
          print(len(self.currentPath));
          self.currentPathIndex = 1;
          if(len(self.currentPath) > 1):
            #self.nextTurnVariable.set("Next Turn: " + self.currentPath[self.currentPathIndex]);
            print("Next Turn will be " + self.currentPath[self.currentPathIndex]);

          print(self.pathPlanner.getLeftRightTurnList()); 
          print(self.pathPlanner.getNodeList()); 
          if self.docked:
              print("Robot is Docked, Press Un-Dock First")
          else:
              self.undock_pub.publish();
              self.state = "FollowLine";
      else:
          rospy.logwarn("Either Source or Destination is not set");
          print("Set FROM and TO")

  def turnAndGo(self):
      if(self.isAsleep):
          call(["rosservice", "call", "/raspicam_node/start_capture"]);
          self.isAsleep = False;
          rospy.loginfo("Coming out of sleep");

      self.timeOfLastActivity = rospy.Time.now();
      
      if(self.sourceSelected is not None and self.destinationSelected is not None):
          print("Source Selected: " + self.sourceSelected);
          print("Destination Selected: " + self.destinationSelected);
          self.pathPlanner.calculatePath();
          print("Path returned with length: ");
          self.currentPath = self.pathPlanner.getLeftRightTurnList();
          self.currentPathIndex = 1;
          if(len(self.currentPath) > 1):
              print("Next Turn will be " + self.currentPath[self.currentPathIndex]);
          print(self.pathPlanner.getLeftRightTurnList()); 
          print(self.pathPlanner.getNodeList()); 
          if self.docked:
              print("Robot is Docked, Press Un-Dock First")
          else:
              self.state = "Turn";
              self.undock_pub.publish();
              if(self.command_turn(0.5, math.pi, 0)):
                  self.state = "FollowLine";
              else:
                  self.state = "Error, Turn not successfull";
      else:
          rospy.logwarn("Either Source or Destination is not set");
          print("Set FROM and TO")
              
  def Stop(self):
      self.undock_pub.publish();
      self.sendStopCmd();
      self.sendStopCmd();
      self.timeOfLastActivity = rospy.Time.now();
      self.state = "Stop";
      self.sendStopCmd();

  def dock(self):
      self.timeOfLastActivity = rospy.Time.now();
      self.dock_pub.publish();
      if self.docked:
          print("Robot is Already Docked")
      else:
          self.state = "Dock";

  def undock(self):
      self.timeOfLastActivity = rospy.Time.now();

      self.undock_pub.publish();
      time.sleep(0.3);
      for ss in range(0,100):
          self.smooth_drive(-0.2, 0);
          time.sleep(0.02);

      self.state = "Turn";
      if(self.command_turn(0.5, math.pi , 0)):
          self.state = "Stop";
      else:
          self.state = "Error, Turn not successfull";
 
  def intersectionCallback(self, msg):
      self.intersection_err = msg.data;

  def errCallback(self,err):
    self.line_err = err.data;

  def lineVisibleCallback(self,msg):
      self.line_drive = msg.data;

  def current_mode_callback(self,msg):
      pass;

  def qrCodeCallback(self, msg):
      if(rospy.Time.now() - self.timeOfLastIntersection > rospy.Duration(5)):
          print("Received QRCode: " + msg.data);
          self.intersectionCodeMutex.acquire();
          self.currentIntersectionCode = msg.data;
          self.intersectionCodeMutex.release();
          self.timeOfLastIntersection = rospy.Time.now();

  def batteryCallback(self,msg):
      self.battery = msg.data*100;
      pass

  def smooth_drive(self, lin, ang):
      self.twist.linear.x = 0.5*lin;
      self.twist.angular.z = ang;
      self.cmd_vel_pub.publish(self.twist);

  def sonarCallback(self, msg):
      self.sonar_drive = msg.data;

  def command_turn(self, angular_speed, relative_angle, clockwise):
    starting = self.yaw;
    turn_direction = 1;
    if clockwise:
        turn_direction = -1;

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle and self.state == "Turn" and not rospy.is_shutdown()):

        self.smooth_drive(0.0, turn_direction*angular_speed);
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
        angleDiff = self.checkAngleDifference(starting, self.yaw);
        if(abs(relative_angle - angleDiff) < 0.1):
            break;

    rospy.loginfo("Turn Difference: " + str(angleDiff));
    
    if(self.state != "Turn"):
        rospy.logwarn("Turn stopped due to state change");
      
    self.sendStopCmd();

    if(abs(relative_angle - angleDiff) < 0.2):
        return True;
    else:
        return False;

  def sendStopCmd(self):
      self.smooth_drive(0.0,0.0);

  def checkAngleDifference(self, ang1, ang2):
      ang1 = ang1 + math.pi;
      ang2 = ang2 + math.pi;
      diff = ang1 - ang2;
      if(ang2 > ang1):
          diff = ang2 - ang1;

      if(diff > math.pi):
          if(ang2 > ang1):
              diff = 2*math.pi - ang2 + ang1;
          if(ang1 >= ang2):
              diff = 2*math.pi - ang1 + ang2;

      return diff;

  def odomCallback(self,msg):
    if(not self.odomRecd):
        self.odomRecd = True;
    self.last_odom = msg;
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.yaw = euler[2]

  def executeTurn(self, nextTurn):
    if(nextTurn == 'E'):
        for stpCnter in range(50):
            self.smooth_drive(0.2, 0);
            time.sleep(0.02);
        self.sendStopCmd();
        self.sourceSelected = self.destinationSelected;
        self.destinationSelected = None;
        rospy.loginfo("Stopping at end");
        self.sendStopCmd();
        self.state = "Stop";
    
    elif(nextTurn == 'L' or nextTurn == 'R'):
        turnSign = 1;
        clockwise = 0;
        if(nextTurn == 'R'):
            turnSign = -1;
            clockwise = 1;

        for stpCnter in range(50):
            self.smooth_drive(0.2, 0);
            time.sleep(0.02);
        rospy.loginfo("Turning Now");
        self.sendStopCmd();
        self.state = "Turn";
        if(self.command_turn(0.2, math.pi/2 , clockwise)):
            self.state = "FollowLine";
            rospy.loginfo("Turned...");
        else:
            self.state = "Error, Turn not successfull";
            rospy.loginfo("Turn failed");
            self.sendStopCmd();

    elif(nextTurn == 'S'):
        self.smooth_drive(0.4, (-float(self.line_err)/40.0));
        rospy.loginfo("Going Straight");
    
    if(nextTurn == 'S' or nextTurn == 'L' or nextTurn == 'R'):
        print("Next Turn will be: " + self.currentPath[self.currentPathIndex]);

  def runThreadFunc(self):
      while not rospy.is_shutdown():
        time.sleep(0.02);
        if(self.state == "FollowLine" and self.line_drive and self.sonar_drive):
          self.tone_pub.publish(self.FOLLOW_TONE);

        if(not self.isAsleep and self.state == "Stop" and rospy.Time.now() - self.timeOfLastActivity > rospy.Duration(60)):
           call(["rosservice", "call", "/raspicam_node/stop_capture"]);
           self.isAsleep = True;
           rospy.loginfo("Going to sleep");

        if(self.state == "Stop"):
            self.sendStopCmd();
            continue;

        if (self.state != "FollowLine"):
          continue;
        
        if not self.sonar_drive:
          self.sendStopCmd();
          continue;
      
        if(len(self.currentIntersectionCode) > 0):
            nextTurn = self.currentPath[self.currentPathIndex];
            self.currentPathIndex = self.currentPathIndex + 1;
            rospy.loginfo("Next Turn is: " + nextTurn); 
            self.executeTurn(nextTurn);
            self.intersectionCodeMutex.acquire();
            self.currentIntersectionCode = '';
            self.intersectionCodeMutex.release();
        
        elif(self.intersection_err != -1000.0):
            self.smooth_drive(0.2, (-float(self.intersection_err)/60.0));
            
        elif(self.line_err != -1000.0):
            self.smooth_drive(self.LINEAR_SPEED, (-float(self.line_err)/40.0));
            self.noLineCount = 0;
        
        elif(self.line_err == -1000.0):
            self.noLineCount = self.noLineCount + 1;
            if(self.noLineCount > 40):
                rospy.loginfo_throttle(5,"Stopping since line isn't visible");
                self.sendStopCmd();
                self.state = "Stop";
        else:
          self.sendStopCmd();

      print("Thread exited cleanly");

def main(args):
  rospy.init_node('create_eyes_controller', anonymous=True)
  ic = DriveCreate2()
  ic.runThread.start();
  key = 's';
  while(True):
    key = raw_input('\nS: Stop\nD: Dock\nU: Undock\nG: Go\nT: Turn and Go\nF: From and To\nV: View Current Data\nQ: Quit\n\nYour Choice: ');
    print("Key Pressed: " + key);
    if(key == 'q' or key == 'Q'):
      break;
    elif(key == 'd' or key == 'D'):
      ic.dock();
    elif(key == 'u' or key == 'U'):
      ic.undock();
    elif(key == 's' or key == 'S'):
      ic.Stop();      
    elif(key == 'g' or key == 'G'):
      ic.goAhead();
    elif(key == 't' or key == 'T'):
      ic.turnAndGo();
    elif(key == 'f' or key == 'F'):
      print(ic.pathPlanner.station_names);
      ic.sourceSelected = raw_input('Enter Src[A-M]: ' );
      ic.destinationSelected = raw_input('Enter Dst[A-M]: ' );
      ic.pathPlanner.setStartNode(ic.sourceSelected);
      ic.pathPlanner.setEndNode(ic.destinationSelected);
    elif(key == 'v' or key == 'V'):
      print("\nState\t Src\t Dst\t Battery\t Line\tSonar");
      print(ic.state, '\t', ic.sourceSelected, '\t', ic.destinationSelected, '\t', ic.battery, '\t', ic.line_drive, '\t', ic.sonar_drive, '\n');
    else:
      print("Input Not Recognized..\n");
        
if __name__ == '__main__':
    main(sys.argv)
