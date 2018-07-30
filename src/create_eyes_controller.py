#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('charlie')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import tf
import time
import numpy
import math
from subprocess import call
from Tkinter import *
import ttk
import tkMessageBox

class DriveCreate2:

  def __init__(self):
    
    self.timeOfLastActivity = rospy.Time.now();
    self.isAsleep = False;

    self.LINEAR_SPEED = 0.8;
   
    self.state = "Stop"
    
    #Publishers
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.mode_pub = rospy.Publisher('iRobot_0/mode', String, queue_size = 1)
    self.tone_pub = rospy.Publisher('buzzer1/tone', Int32, queue_size = 1)
    self.dock_pub = rospy.Publisher('/dock', Empty, queue_size = 1);
    self.undock_pub = rospy.Publisher('/undock', Empty, queue_size = 1);
    
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

    #GUI Buttons
    self.buttonStyle = ttk.Style()
    self.buttonStyle.configure('my.TButton', font=('Helvetica', 18))

    ttk.Button(self.mainframe, text="Go", style='my.TButton', command=self.goAhead, width=16).grid(row=2, rowspan=2, column=0, pady=25)
    ttk.Button(self.mainframe, text="Turn and Go", style='my.TButton', command=self.turnAndGo, width=16).grid(row=2, rowspan=2, column=1, pady=25)
    ttk.Button(self.mainframe, text="STOP", style='my.TButton', command=self.Stop, width=16).grid(row=4, rowspan=3, column=0, columnspan=3, pady=5)
    ttk.Button(self.mainframe, text="Dock", style='my.TButton', command=self.dock, width = 16).grid(row=8,column=0, pady=15)
    ttk.Button(self.mainframe, text="Un-Dock", style='my.TButton', command=self.undock, width = 16).grid(row=8,column=1, pady=15)

    ttk.Button(self.mainframe, text="Reset", style='my.TButton', command=self.resetPressed, width=16).grid(row=10, column=0, pady=5)

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
    
    self.STOP_TONE = 2;
    self.FOLLOW_TONE = 5;

    #Subscribers
    self.bat_sub = rospy.Subscriber('battery/charge_ratio', Float32, self.batteryCallback)
    self.line_visible_sub = rospy.Subscriber('line_visible', Bool, self.lineVisibleCallback)
    self.current_mode_sub = rospy.Subscriber('iRobot_0/current_mode', String, self.current_mode_callback);
    self.err_sub = rospy.Subscriber('line_error', Float32, self.errCallback)
    self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
    self.sonar_sub = rospy.Subscriber('sonar_drive', Bool, self.sonarCallback);
    
    self.timeOfLastActivity = rospy.Time.now();

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
              
  def Stop(self):
      self.timeOfLastActivity = rospy.Time.now();
      self.state = "Stop";
      self.mode_pub.publish("safe");
      self.sendStopCmd();

  def dock(self):
      self.timeOfLastActivity = rospy.Time.now();
      self.dock_pub.publish();
      if self.docked:
          tkMessageBox.showerror("Error", "Robot is Already Docked")
      else:
          self.state = "Dock";
          self.mode_pub.publish("dock");

  def undock(self):
      self.timeOfLastActivity = rospy.Time.now();

      self.undock_pub.publish();
      time.sleep(0.3);
      for ss in range(0,100):
          self.smooth_drive(-0.2, 0);
          time.sleep(0.02);

      self.state = "Turn";
      if(self.command_turn(math.pi)):
          self.state = "Stop";
      else:
          self.state = "Error, Turn not successfull";

  def updateLabel(self):
      self.currentStatus.set("State: " + self.state);
      self.statusLabel.update_idletasks();
      self.batteryLabel.update_idletasks();
      self.lineLabel.update_idletasks();
      self.oiModeLabel.update_idletasks();
      self.sonarLabel.update_idletasks();

      self.root.update_idletasks();

      if(self.state == "FollowLine" and self.line_drive and self.sonar_drive):
          self.tone_pub.publish(self.FOLLOW_TONE);

      if(not self.isAsleep and self.state == "Stop" and rospy.Time.now() - self.timeOfLastActivity > rospy.Duration(60)):
          call(["rosservice", "call", "/raspicam_node/stop_capture"]);
          self.isAsleep = True;
          rospy.loginfo("Going to sleep");

      self.root.after(200, self.updateLabel);
 
  def lineVisibleCallback(self,msg):
      self.lineVisible.set("Line: " + str(msg.data));
      self.line_drive = msg.data;

  def current_mode_callback(self,msg):
      self.current_oi_mode.set("OI Mode: " + msg.data);

  def batteryCallback(self,msg):
      self.batteryStatus.set(str("Battery: %.2f" % round(msg.data*100,2)) + "%" );

  def smooth_drive(self, lin, ang):
      self.twist.linear.x = self.last_drive_lin*0.5 + lin*0.5;
      self.twist.angular.z = ang;

      if(self.twist.linear.x > 0 and self.twist.linear.x < 0.05):
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

      if(desired > starting):
          t_end = time.time() + 30;
          while(self.state == "Turn"):
              self.smooth_drive(0.0,0.5);
              time.sleep(0.01);
              current = self.yaw;
              rospy.loginfo_throttle(5,"Turning: " + str(current) + " " + str(desired));
              if(current > desired or current < starting):
                  self.sendStopCmd();
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
              rospy.loginfo_throttle(5,"Turning: " + str(current) + " " + str(desired));
              if(current < desired or current > starting):
                  self.sendStopCmd();
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

  def errCallback(self,err):
        
    if(self.state == "FollowLine"):
        if(err.data == -1000.0):
            #I will come here when I'm asked to follow line, but I can't see the line. User is expected to press go button again.
            #this is also done to stop the robot from following random things if it doesn't see the line
            self.noLineCount = self.noLineCount + 1;
            if(self.noLineCount >= 200):
                rospy.loginfo_throttle(5,"Stopping since line isn't visible");
                self.sendStopCmd();
                self.state = "Stop";

        elif not self.sonar_drive:
            self.sendStopCmd();

        else:
            self.smooth_drive(self.LINEAR_SPEED, (-float(err.data)/35.0));
            self.noLineCount = 0;

def main(args):
  rospy.init_node('create_eyes_controller', anonymous=True)
  ic = DriveCreate2()
  ic.root.mainloop();
        
if __name__ == '__main__':
    main(sys.argv)

