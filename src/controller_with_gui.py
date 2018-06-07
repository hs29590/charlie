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
    self.TIME_FOR_MOVING_TOWARDS_INTERSECTION = 2; #in seconds, this should also change with the LINEAR SPEED
    self.TIME_FOR_TURNING = 4; #in seconds, this should also change with the LINEAR_SPEED


    #Publishers
    self.cmd_vel_pub = rospy.Publisher('iRobot_0/cmd_vel', Twist, queue_size=1)
    self.mode_pub = rospy.Publisher('iRobot_0/mode', String, queue_size = 1)
    self.tone_pub = rospy.Publisher('buzzer1/tone', Int32, queue_size = 1)
    
    #GUI Variables
    self.root = Tk()
    self.root.title("GUI")
    framew = 500;
    frameh = 500;
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

    self.sourceDestinationVar = StringVar();
    self.sourceDestinationVar.set("Source:       Destination:   ");

#    srcChoices = {'A', 'B', 'C', 'D', 'E', 'F', 'G'}#, 'H', 'I', 'J', 'K', 'L'}
#    dstChoices = {'A', 'B', 'C', 'D', 'E', 'F', 'G'}#, 'H', 'I', 'J', 'K', 'L'}
#
    self.srcNode.set(' ');
    self.dstNode.set(' ');
    
#    self.pathPlanner.setStartNode(self.srcNode.get());
#    self.pathPlanner.setEndNode(self.dstNode.get());

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

    ttk.Button(self.mainframe, text="SOURCE", style='my.TButton', command=self.selectSource, width = 16).grid(row=13,column=0, pady=15)
    ttk.Button(self.mainframe, text="DESTINATION", style='my.TButton', command=self.selectDestination, width = 16).grid(row=13,column=1, pady=15)

    ttk.Button(self.mainframe, text="Reset", style='my.TButton', command=self.resetPressed, width=16).grid(row=10, column=0, pady=5)
    
    self.sourceDestinationLabel = ttk.Label(self.mainframe, textvariable=self.sourceDestinationVar, font=('Helvetica',12));
    self.sourceDestinationLabel.grid(row=14, column=0, columnspan=2);

#    srcSelectMenu = OptionMenu(self.mainframe, self.srcNode, *srcChoices)
#    #Label(self.mainframe, text="Choose Process", font=("Helvetica", 14)).grid(row = 2, column=1, pady=(15,2))
#    srcSelectMenu.grid(row=11, column=1)
#    #srcSelectMenu.bind('<Button-1>', self.dropdownopen)
#    self.srcNode.trace('w', self.srcNodeChanged)
    
#    dstSelectMenu = OptionMenu(self.mainframe, self.dstNode, *dstChoices)
#    dstSelectMenu.grid(row=11, column=2)
#    #srcSelectMenu.bind('<Button-1>', self.dropdownopen)
#    self.dstNode.trace('w', self.dstNodeChanged)

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
    self.left_line_err = -1000.0;
    self.right_line_err = -1000.0;
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
    self.right_err_sub = rospy.Subscriber('right_line_err', Float32, self.rightLineErrCallback);
    self.left_err_sub = rospy.Subscriber('left_line_err', Float32, self.leftLineErrCallback);

    self.odom_sub = rospy.Subscriber('iRobot_0/odom', Odometry, self.odomCallback)
    self.sonar_sub = rospy.Subscriber('sonar_drive', Bool, self.sonarCallback);
    self.intersection_sub = rospy.Subscriber('/intersection_err', Float32, self.intersectionCallback); 
   
    self.timeOfLastActivity = rospy.Time.now();

    self.sourceSelected = '0';
    self.destinationSelected = '0';

    self.runThread = threading.Thread(target=self.runThreadFunc)
    self.runThread.daemon = True
      
    

  def dstpopup(self):

    toplevel = Toplevel()
    toplevel.geometry( "%dx%d+%d+%d" % (400,400,self.root.winfo_x() + 10 ,self.root.winfo_y() + 10))
    def aButton():
	self.destinationSelected = 'A';
        self.pathPlanner.setEndNode(self.destinationSelected);
        toplevel.destroy();
    def bButton():
	self.destinationSelected = 'B';
        self.pathPlanner.setEndNode(self.destinationSelected);
        toplevel.destroy();
    def cButton():
	self.destinationSelected = 'C';
        self.pathPlanner.setEndNode(self.destinationSelected);
        toplevel.destroy();
    def dButton():
        self.destinationSelected = 'D';
        self.pathPlanner.setEndNode(self.destinationSelected);
        toplevel.destroy();
    def eButton():
        self.destinationSelected = 'E';
        self.pathPlanner.setEndNode(self.destinationSelected);
        toplevel.destroy();
    def fButton():
        self.destinationSelected = 'F';
        self.pathPlanner.setEndNode(self.destinationSelected);
        toplevel.destroy();
    def gButton():
        self.destinationSelected = 'G';
        self.pathPlanner.setEndNode(self.destinationSelected);
        toplevel.destroy();

    ttk.Button(toplevel, text="A", style='my.TButton', command=aButton).grid(column = 0, row = 0, padx = 10, pady = 10);
    ttk.Button(toplevel, text="B", style='my.TButton', command=bButton).grid(column = 1, row = 0, padx = 10, pady = 10);
    ttk.Button(toplevel, text="C", style='my.TButton', command=cButton).grid(column = 0, row = 1, padx = 10, pady = 10);
    ttk.Button(toplevel, text="D", style='my.TButton', command=dButton).grid(column = 1, row = 1, padx = 10, pady = 10);
    ttk.Button(toplevel, text="E", style='my.TButton', command=eButton).grid(column = 0, row = 2, padx = 10, pady = 10);
    ttk.Button(toplevel, text="F", style='my.TButton', command=fButton).grid(column = 1, row = 2, padx = 10, pady = 10);
    ttk.Button(toplevel, text="G", style='my.TButton', command=gButton).grid(column = 0, row = 3, padx = 10, pady = 10);

  def srcpopup(self):

    toplevel = Toplevel()
    toplevel.geometry( "%dx%d+%d+%d" % (400,400,self.root.winfo_x() + 10 ,self.root.winfo_y() + 10))
    def aButton():
	self.sourceSelected = 'A';
        self.pathPlanner.setStartNode(self.sourceSelected);
        toplevel.destroy();
    def bButton():
	self.sourceSelected = 'B';
        self.pathPlanner.setStartNode(self.sourceSelected);
        toplevel.destroy();
    def cButton():
	self.sourceSelected = 'C';
        self.pathPlanner.setStartNode(self.sourceSelected);
        toplevel.destroy();
    def dButton():
        self.sourceSelected = 'D';
        self.pathPlanner.setStartNode(self.sourceSelected);
        toplevel.destroy();
    def eButton():
        self.sourceSelected = 'E';
        self.pathPlanner.setStartNode(self.sourceSelected);
        toplevel.destroy();
    def fButton():
        self.sourceSelected = 'F';
        self.pathPlanner.setStartNode(self.sourceSelected);
        toplevel.destroy();
    def gButton():
        self.sourceSelected = 'G';
        self.pathPlanner.setStartNode(self.sourceSelected);
        toplevel.destroy();

    ttk.Button(toplevel, text="A", style='my.TButton', command=aButton).grid(column = 0, row = 0, padx = 10, pady = 10);
    ttk.Button(toplevel, text="B", style='my.TButton', command=bButton).grid(column = 1, row = 0, padx = 10, pady = 10);
    ttk.Button(toplevel, text="C", style='my.TButton', command=cButton).grid(column = 0, row = 1, padx = 10, pady = 10);
    ttk.Button(toplevel, text="D", style='my.TButton', command=dButton).grid(column = 1, row = 1, padx = 10, pady = 10);
    ttk.Button(toplevel, text="E", style='my.TButton', command=eButton).grid(column = 0, row = 2, padx = 10, pady = 10);
    ttk.Button(toplevel, text="F", style='my.TButton', command=fButton).grid(column = 1, row = 2, padx = 10, pady = 10);
    ttk.Button(toplevel, text="G", style='my.TButton', command=gButton).grid(column = 0, row = 3, padx = 10, pady = 10);

  def selectSource(self, *args):
      self.srcpopup();

  def selectDestination(self, *args):
      self.dstpopup();

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
          rospy.loginfo("Coming out of sleep");

      self.timeOfLastActivity = rospy.Time.now();
      if self.docked:
          tkMessageBox.showerror("Error", "Robot is Docked, Press Un-Dock First")
      else:
          self.mode_pub.publish("safe");
          self.state = "FollowLine";

      self.pathPlanner.calculatePath();
      print("Path returned with length: ");
      print("Source Selected: " + self.sourceSelected);
      print("Destination Selected: " + self.destinationSelected);

      self.currentPath = self.pathPlanner.getLeftRightTurnList();
      self.currentPathIndex = 1;
      print(self.pathPlanner.getLeftRightTurnList()); 
      print(self.pathPlanner.getNodeList()); 

  def turnAndGo(self):
      if(self.isAsleep):
          call(["rosservice", "call", "/raspicam_node/start_capture"]);
          self.isAsleep = False;
          rospy.loginfo("Coming out of sleep");

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

      self.sourceDestinationVar.set("SOURCE: " + self.sourceSelected + "    DESTINATION: " + self.destinationSelected);
      self.sourceDestinationLabel.update_idletasks();

      self.root.update_idletasks();

      if(self.state == "FollowLine" and self.line_drive and self.sonar_drive):
          self.tone_pub.publish(self.FOLLOW_TONE);

      if(not self.isAsleep and self.state == "Stop" and rospy.Time.now() - self.timeOfLastActivity > rospy.Duration(60)):
          call(["rosservice", "call", "/raspicam_node/stop_capture"]);
          self.isAsleep = True;
          rospy.loginfo("Going to sleep");


      self.root.after(200, self.updateLabel);
 
  def intersectionCallback(self, msg):
      self.intersection_err = msg.data;

  def leftLineErrCallback(self, err):
      self.left_line_err = err.data;

  def rightLineErrCallback(self, err):
      self.right_line_err = err.data;

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
              #rospy.loginfo("des > sta - Turning: Starting: " + str(starting) + " Current:" + str(current) + " Desired: " + str(desired));
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
              #rospy.loginfo("des < sta Turning: Starting: " + str(starting) + " Current:" + str(current) + " Desired: " + str(desired));
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
      
	if(self.line_err == -1000.0):
	    self.noLineCount = self.noLineCount + 1;
	    if(self.noLineCount > 20):
		rospy.loginfo_throttle(5,"Stopping since line isn't visible");
                self.sendStopCmd();
                self.state = "Stop";
	    if(self.intersection_err == -1000.0):
                self.sendStopCmd();
            
        elif(self.intersection_err != -1000.0):
            t_end = time.time() + self.TIME_FOR_MOVING_TOWARDS_INTERSECTION;
            while(time.time() < t_end):
                self.smooth_drive(self.LINEAR_SPEED, (-float(self.intersection_err)/50.0));
                time.sleep(0.02);

            nextTurn = self.currentPath[self.currentPathIndex];
            self.currentPathIndex = self.currentPathIndex + 1;
            rospy.loginfo("Next Turn is: " + nextTurn); 
            if(nextTurn == 'E'):
                rospy.loginfo("Stopping at end");
                self.sendStopCmd();
                self.state = "Stop";
            elif(nextTurn == 'L'):
                rospy.loginfo("Turning Left");
                t_end = time.time() + self.TIME_FOR_TURNING; 
                while(time.time() < t_end and self.left_line_err != -1000.0):
                    self.smooth_drive(self.LINEAR_SPEED, (-float(self.left_line_err)/40.0));
                    time.sleep(0.02);
                self.state = "FollowLine";
                rospy.loginfo("I Turned Left");
            elif(nextTurn == 'R'):
                rospy.loginfo("Turning Right");
                t_end = time.time() + self.TIME_FOR_TURNING; 
                while(time.time() < t_end and self.right_line_err != -1000.0):
                    self.smooth_drive(self.LINEAR_SPEED, (-float(self.right_line_err)/40.0));
                    time.sleep(0.02);
                self.state = "FollowLine";
                rospy.loginfo("I Turned Right");
            elif(nextTurn == 'S'):
                rospy.loginfo("Going Straight");
            
        elif(self.line_err != -1000.0):
            self.smooth_drive(self.LINEAR_SPEED, (-float(self.line_err)/50.0));
	    self.noLineCount = 0;
            
      print("Thread exited cleanly");

def main(args):
  rospy.init_node('create_eyes_controller', anonymous=True)
  ic = DriveCreate2()
  ic.runThread.start();
  ic.root.mainloop();
        
if __name__ == '__main__':
    main(sys.argv)
