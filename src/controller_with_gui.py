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
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from irobotcreate2.msg import Battery
import tf
import time
import numpy
import math
from subprocess import call
from Tkinter import *
import ttk
import tkMessageBox
import threading

class DriveCreate2:

  def __init__(self):
    
    self.timeOfLastActivity = rospy.Time.now();
    self.isAsleep = False;

    self.LINEAR_SPEED = 0.8;   
    self.state = "Stop"
    
    #Publishers
    self.cmd_vel_pub = rospy.Publisher('iRobot_0/cmd_vel', Twist, queue_size=1)
    self.mode_pub = rospy.Publisher('iRobot_0/mode', String, queue_size = 1)
    self.tone_pub = rospy.Publisher('buzzer1/tone', Int32, queue_size = 1)
    
    #GUI Variables
    self.root = Tk()
    self.root.title("GUI")
    framew = 500;
    frameh = 400;
    screenw = self.root.winfo_screenwidth();
    screenh = self.root.winfo_screenheight();
    posx = (screenw/2) - (framew/2);
    posy = (screenh/2) - (frameh/2);
    self.root.geometry( "%dx%d+%d+%d" % (framew,frameh,posx,posy))

    self.currentStatus = StringVar();
    self.currentStatus.set("State: Stop");

    self.batteryStatus = StringVar();
    self.batteryStatus.set("Battery: na%");
    
    self.intersectionVisible = StringVar();
    self.intersectionVisible.set("Intersection: False");
    
    self.lineVisible = StringVar();
    self.lineVisible.set("Line: False");

    self.current_oi_mode = StringVar();
    self.current_oi_mode.set('OI_Mode: NA');

    self.sonarStatus = StringVar();
    self.sonarStatus.set("Sonar: No Obstruction");

    self.mainframe = ttk.Frame(self.root, padding="10 10 30 30", height=400, width=500)
    self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
    self.mainframe.columnconfigure(0, weight=50,minsize=50)
    self.mainframe.rowconfigure(0, weight=50,minsize=50)

    self.pathPlanner = Planner();

    self.srcNode = StringVar();
    self.dstNode = StringVar();

    srcChoices = {'A', 'B', 'C', 'D', 'E', 'F', 'G'}#, 'H', 'I', 'J', 'K', 'L'}
    dstChoices = {'A', 'B', 'C', 'D', 'E', 'F', 'G'}#, 'H', 'I', 'J', 'K', 'L'}

    self.srcNode.set('A');
    self.dstNode.set('G');
    
    self.pathPlanner.setStartNode(self.srcNode.get());
    self.pathPlanner.setEndNode(self.dstNode.get());

    #GUI Text
    self.batteryLabel = ttk.Label(self.mainframe, textvariable=self.batteryStatus, font=('Helvetica',12));
    self.batteryLabel.grid(row=1, column=1);

    self.statusLabel = ttk.Label(self.mainframe, textvariable=self.currentStatus, font=('Helvetica',12));
    self.statusLabel.grid(row=1, column=0);

    self.lineLabel = ttk.Label(self.mainframe, textvariable=self.lineVisible, font=('Helvetica',12));
    self.lineLabel.grid(row=0, column=0);

    self.oiModeLabel = ttk.Label(self.mainframe, textvariable=self.current_oi_mode, font=('Helvetica',12));
    self.oiModeLabel.grid(row=10,column=1);

    self.sonarLabel = ttk.Label(self.mainframe, textvariable=self.sonarStatus, font=('Helvetica',12));
    self.sonarLabel.grid(row=0,column=1);

    self.intersectionLabel = ttk.Label(self.mainframe, textvariable=self.intersectionVisible, font=('Helvetica',12));
    self.intersectionLabel.grid(row=13, column=0);


    #GUI Buttons

    self.buttonStyle = ttk.Style()
    self.buttonStyle.configure('my.TButton', font=('Helvetica', 18))

    ttk.Button(self.mainframe, text="Go", style='my.TButton', command=self.goAhead, width=16).grid(row=2, rowspan=2, column=0, pady=25)
    ttk.Button(self.mainframe, text="Turn and Go", style='my.TButton', command=self.turnAndGo, width=16).grid(row=2, rowspan=2, column=1, pady=25)
    ttk.Button(self.mainframe, text="STOP", style='my.TButton', command=self.Stop, width=16).grid(row=4, rowspan=3, column=0, columnspan=3, pady=5)
    ttk.Button(self.mainframe, text="Dock", style='my.TButton', command=self.dock, width = 16).grid(row=8,column=0, pady=15)
    ttk.Button(self.mainframe, text="Un-Dock", style='my.TButton', command=self.undock, width = 16).grid(row=8,column=1, pady=15)

    ttk.Button(self.mainframe, text="Reset", style='my.TButton', command=self.resetPressed, width=16).grid(row=10, column=0, pady=5)

    srcSelectMenu = OptionMenu(self.mainframe, self.srcNode, *srcChoices)
    #Label(self.mainframe, text="Choose Process", font=("Helvetica", 14)).grid(row = 2, column=1, pady=(15,2))
    srcSelectMenu.grid(row=11, column=1)
    #srcSelectMenu.bind('<Button-1>', self.dropdownopen)
    self.srcNode.trace('w', self.srcNodeChanged)
    
    dstSelectMenu = OptionMenu(self.mainframe, self.dstNode, *dstChoices)
    #Label(self.mainframe, text="Choose Process", font=("Helvetica", 14)).grid(row = 2, column=1, pady=(15,2))
    dstSelectMenu.grid(row=11, column=2)
    #srcSelectMenu.bind('<Button-1>', self.dropdownopen)
    self.dstNode.trace('w', self.dstNodeChanged)

    self.root.after(1000, self.updateLabel);

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

    self.STOP_TONE = 2;
    self.FOLLOW_TONE = 5;
    self.currentPath = None;
    self.currentPathIndex = 0;

    #Subscribers
    self.bat_sub = rospy.Subscriber('iRobot_0/battery', Battery, self.batteryCallback)
    self.line_visible_sub = rospy.Subscriber('line_visible', Bool, self.lineVisibleCallback)
    self.current_mode_sub = rospy.Subscriber('iRobot_0/current_mode', String, self.current_mode_callback);
    self.err_sub = rospy.Subscriber('line_error', Float32, self.errCallback)
    self.odom_sub = rospy.Subscriber('iRobot_0/odom', Odometry, self.odomCallback)
    self.sonar_sub = rospy.Subscriber('sonar_drive', Bool, self.sonarCallback);
    self.intersection_sub = rospy.Subscriber('/intersection_err', Float32, self.intersectionCallback); 
   
    self.timeOfLastActivity = rospy.Time.now();

    self.runThread = threading.Thread(target=self.runThreadFunc)
    self.runThread.daemon = True
      
    
  def srcNodeChanged(self, *args):
      print('src node changed' + self.srcNode.get());
      self.pathPlanner.setStartNode(self.srcNode.get());
  
  def dstNodeChanged(self, *args):
      print('dst node changed' + self.dstNode.get());
      self.pathPlanner.setEndNode(self.dstNode.get());

  def resetPressed(self):
      self.timeOfLastActivity = rospy.Time.now();
      tkMessageBox.showerror("Error", "Trying to Reset The Robot, Please wait 10 sec..")
      rospy.loginfo("Publishing reset, waiting 7 sec..");
      self.mode_pub.publish("reset");
      time.sleep(6);
      self.mode_pub.publish("start");
      rospy.loginfo("Published Start..");
      
  def goAhead(self):
      if(self.isAsleep):
          call(["rosservice", "call", "/raspicam_node/start_capture"]);
          self.isAsleep = False;

      self.timeOfLastActivity = rospy.Time.now();
      if self.docked:
          tkMessageBox.showerror("Error", "Robot is Docked, Press Un-Dock First")
      else:
          self.mode_pub.publish("safe");
          self.state = "FollowLine";

      self.pathPlanner.calculatePath();
      print("Path returned with length: ");

      self.currentPath = self.pathPlanner.getLeftRightTurnList();
      self.currentPathIndex = 1;
      print(self.pathPlanner.getLeftRightTurnList()); 
      print(self.pathPlanner.getNodeList()); 

  def turnAndGo(self):
      if(self.isAsleep):
          call(["rosservice", "call", "/raspicam_node/start_capture"]);
          self.isAsleep = False;

      self.timeOfLastActivity = rospy.Time.now();
      if self.docked:
          tkMessageBox.showerror("Error", "Robot is Docked, Press Un-Dock First")
      else:
          self.state = "Turn";
          self.mode_pub.publish("safe");
          if(self.command_turn(math.pi)):
              self.state = "FollowLine";
          else:
              self.state = "Error, Turn not successfull";
      
      self.pathPlanner.calculatePath();
      print("Path returned with length: ");
      self.currentPath = self.pathPlanner.getLeftRightTurnList();
      self.currentPathIndex = 1;
      print(self.pathPlanner.getLeftRightTurnList()); 
      print(self.pathPlanner.getNodeList()); 
              
  def Stop(self):
      self.timeOfLastActivity = rospy.Time.now();
      self.state = "Stop";
      self.mode_pub.publish("safe");
      self.sendStopCmd();

  def dock(self):
      self.timeOfLastActivity = rospy.Time.now();
      if self.docked:
          tkMessageBox.showerror("Error", "Robot is Already Docked")
      else:
          self.state = "Dock";
          self.mode_pub.publish("dock");

  def undock(self):
      self.timeOfLastActivity = rospy.Time.now();
      if(not self.docked):
          tkMessageBox.showerror("Error", "Not Docked")
      else:
        self.state = "UnDock";
        self.mode_pub.publish("clean");
        time.sleep(8.5);
        self.mode_pub.publish("safe");

        self.state = "Stop";
        self.sendStopCmd();

  def updateLabel(self):
      self.currentStatus.set("State: " + self.state);
      self.statusLabel.update_idletasks();
      self.batteryLabel.update_idletasks();
      self.lineLabel.update_idletasks();
      self.oiModeLabel.update_idletasks();
      self.sonarLabel.update_idletasks();
      self.intersectionLabel.update_idletasks();
      self.intersectionVisible.set("Intersection: " + str(self.intersection_err));

      self.root.update_idletasks();

      if(self.state == "FollowLine" and self.line_drive and self.sonar_drive):
          self.tone_pub.publish(self.FOLLOW_TONE);
#    self.isAsleep = True;

#      if(self.state == "Stop" and rospy.Time.now() - self.timeOfLastActivity > rospy.Duration(10)):
#          call(["rosservice", "call", "/raspicam_node/stop_capture"]);

      self.root.after(200, self.updateLabel);
 
  def intersectionCallback(self, msg):
      self.intersection_err = msg.data;

  def errCallback(self,err):
    self.line_err = err.data;

  def lineVisibleCallback(self,msg):
      self.lineVisible.set("Line: " + str(msg.data));
      self.line_drive = msg.data;

  def current_mode_callback(self,msg):
      self.current_oi_mode.set("OI Mode: " + msg.data);

  def batteryCallback(self,msg):
      self.docked = msg.dock;
      self.batteryStatus.set(str("%.2f" % round(msg.level,2))+"%, Docked: " + str(self.docked));

  def smooth_drive(self, lin, ang):
      self.twist.linear.x = self.last_drive_lin*0.5 + lin*0.5;
      self.twist.angular.z = ang;

      if(self.twist.linear.x < 0.05):
          self.twist.linear.x = 0.0;
          
      self.cmd_vel_pub.publish(self.twist);

  def sonarCallback(self, msg):
      self.sonar_drive = msg.data;
      if(msg.data):
        self.sonarStatus.set('No Obstruction');
      else:
        self.sonarStatus.set('Obstruction');

  def command_turn(self, angleToTurn):
      if(not self.odomRecd):
          rospy.loginfo("Trying to turn without odom recd.");
          return False;
      starting = self.yaw;
      desired = starting + angleToTurn;
      if(desired > math.pi):
          desired = desired - 2*math.pi;
      elif(desired < -math.pi):
          desired = desired + 2*math.pi;
      
      current = starting;
      rospy.loginfo("Starting Turning: Current(Starting):" + str(current) + " Desired:" + str(desired));

      if(desired > starting):
          t_end = time.time() + 30;
          while(self.state == "Turn"):
              self.smooth_drive(0.0,0.5);
              time.sleep(0.01);
              current = self.yaw;
              #rospy.loginfo_throttle(5,"Turning: " + str(current) + " " + str(desired));
              rospy.loginfo("des > sta - Turning: Starting: " + str(starting) + " Current:" + str(current) + " Desired: " + str(desired));
              if(current > desired or current < starting):
                  self.sendStopCmd();
                  rospy.loginfo("Turn Successful!");
                  return True;
                  break;
              if(time.time() > t_end):
                  rospy.logwarn("Stopping the turn, couldn't finish it");
                  self.sendStopCmd();
                  return False;
                  break;
      elif(desired < starting):
          t_end = time.time() + 30;
          while(self.state == "Turn"):
              self.smooth_drive(0.0,-0.5);
              time.sleep(0.001);
              current = self.yaw;
              #rospy.loginfo_throttle(5,"Turning: " + str(current) + " " + str(desired));
              rospy.loginfo("des < sta Turning: Starting: " + str(starting) + " Current:" + str(current) + " Desired: " + str(desired));
              if(current < desired or current > starting):
                  self.sendStopCmd();
                  rospy.loginfo("Turn Successful!");
                  return True;
                  break;
              if(time.time() > t_end):
                  rospy.logwarn("Stopping the turn, couldn't finish it");
                  self.sendStopCmd();
                  return False;
                  break;
      
      self.sendStopCmd();
      if(self.state != "Turn"):
          rospy.logwarn("Turn stopped due to state change");
      
      return False;

  def sendStopCmd(self):
      self.smooth_drive(0.0,0.0);
      self.tone_pub.publish(self.STOP_TONE);

  def odomCallback(self,msg):
    if(not self.odomRecd):
        self.odomRecd = True;
    self.last_odom = msg;
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.yaw = euler[2]

  def runThreadFunc(self):
      while not rospy.is_shutdown():
        time.sleep(0.02);

        if not self.sonar_drive:
                  self.sendStopCmd();
      
        if (self.state != "FollowLine"):
            continue;
      
        if(self.intersection_err == -1000.0 and self.line_err == -1000.0):
            self.sendStopCmd();
            
        elif(self.intersection_err != -1000.0):
            while(self.intersection_err != -1000.0):
                self.smooth_drive(self.LINEAR_SPEED, (-float(self.intersection_err)/50.0));
            self.sendStopCmd();
            rospy.loginfo("[Intersection] Sent Stop cmd");
            nextTurn = self.currentPath[self.currentPathIndex];
            self.currentPathIndex = self.currentPathIndex + 1;
            rospy.loginfo("Next Turn is: " + nextTurn); 
            self.sendStopCmd();
            if(nextTurn == 'E'):
                rospy.loginfo("Stopping at end");
                self.state = "Stop";
            elif(nextTurn == 'L'):
                rospy.loginfo("Turning Left");
                for stpCnter in range(50):
                    self.smooth_drive(self.LINEAR_SPEED, 0.0);
                    time.sleep(0.01);
                for stpCnter in range(100):
                    self.smooth_drive(self.LINEAR_SPEED, 0.5);
                    time.sleep(0.02);
                while(abs(self.line_err) > 10):
                    self.smooth_drive(self.LINEAR_SPEED, 0.5);
                    time.sleep(0.02);
                #self.state = "Turn";
                #if(self.command_turn(math.pi/2)):
                self.state = "FollowLine";
                #else:
                #self.state = "Error, Turn not successfull";
            elif(nextTurn == 'R'):
                rospy.loginfo("Turning Right");
                for stpCnter in range(50):
                    self.smooth_drive(self.LINEAR_SPEED, 0.0);
                    time.sleep(0.01);
                for stpCnter in range(100):
                    self.smooth_drive(self.LINEAR_SPEED, -0.5);
                    time.sleep(0.02);
                while(abs(self.line_err) > 10):
                    self.smooth_drive(self.LINEAR_SPEED, -0.5);
                    time.sleep(0.02);

# for stpCnter in range(0,10):
#                    self.smooth_drive(self.LINEAR_SPEED, -0.1);
                #self.state = "Turn";
                #if(self.command_turn(-math.pi/2)):
                self.state = "FollowLine";
                #else:
                #self.state = "Error, Turn not successfull";
                rospy.loginfo("I Turned Right");
            elif(nextTurn == 'S'):
                rospy.loginfo("Going Straight");
            
        elif(self.line_err != -1000.0):
            self.smooth_drive(self.LINEAR_SPEED, (-float(self.line_err)/50.0));
            
      print("Thread exited cleanly");

def main(args):
  rospy.init_node('create_eyes_controller', anonymous=True)
  ic = DriveCreate2()
  ic.runThread.start();
  ic.root.mainloop();
#  ic.runThread.join();
  #rospy.spin is just a blocking call, which my guy above does as well.
#  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

