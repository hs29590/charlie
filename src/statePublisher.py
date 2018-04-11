#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
import time
#import _thread
import numpy
import math
from subprocess import call

#from tkinter import *
#from tkinter import ttk
from Tkinter import *
import ttk
import tkMessageBox


from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from irobotcreate2.msg import Battery


class Create2StatePublisher:

  def __init__(self):

    #self.gui_sub = rospy.Subscriber('gui_state', String, self.guiCallback)
    self.bat_sub = rospy.Subscriber('iRobot_0/battery', Battery, self.batteryCallback)

    self.line_visible_sub = rospy.Subscriber('line_visible', Bool, self.lineVisibleCallback)

    self.sonar_sub = rospy.Subscriber('sonar_drive', Bool, self.sonarCallback);
    self.master_state_pub = rospy.Publisher('master_state', String, queue_size=1)
    self.current_state_sub = rospy.Subscriber('iRobot_0/current_mode', String, self.current_mode_callback);

    self.gui_state = "Stop";
    
    self.last_gui_state = "Stop";
    
    

    #expect to start at Dock
    self.heading = "B";

    self.docked = True;

    self.publish("CONTROLLER_Stop");
    
    self.status = "Started"
    

    ##FROM GUI###
    self.battery = "N/A"
    self.facingTowardsA = False;

    self.root = Tk()
    self.root.title("GUI")
    
    framew = 500; # root w
    frameh = 400; # root h
    screenw = self.root.winfo_screenwidth();
    screenh = self.root.winfo_screenheight();
    posx = (screenw/2) - (framew/2);
    posy = (screenh/2) - (frameh/2);
    self.root.geometry( "%dx%d+%d+%d" % (framew,frameh,posx,posy))

    self.currentStatus = StringVar();
    self.currentStatus.set('Stop');

    self.batteryStatus = StringVar();
    self.batteryStatus.set('na%');
    
    self.headingStatus = StringVar();
    self.headingStatus.set('NA');
    
    self.lineVisible = StringVar();
    self.lineVisible.set(str(False));

    self.current_oi_mode = StringVar();
    self.current_oi_mode.set('NA');

    self.sonarStatus = StringVar();
    self.sonarStatus.set('No Obstruction');

    self.mainframe = ttk.Frame(self.root, padding="10 10 30 30", height=400, width=500)
    self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
    self.mainframe.columnconfigure(0, weight=50,minsize=50)
    self.mainframe.rowconfigure(0, weight=50,minsize=50)

    self.buttonStyle = ttk.Style()
    self.buttonStyle.configure('my.TButton', font=('Helvetica', 18))
    
    self.batteryLabel = ttk.Label(self.mainframe, textvariable=self.batteryStatus, font=('Helvetica',12));
    self.batteryLabel.grid(row=1, column=1);

    self.statusLabel = ttk.Label(self.mainframe, textvariable=self.currentStatus, font=('Helvetica',12));
    self.statusLabel.grid(row=1, column=0);

    self.headLabel = ttk.Label(self.mainframe, textvariable=self.headingStatus, font=('Helvetica',12));
    self.headLabel.grid(row=0,column=1);

    self.lineLabel = ttk.Label(self.mainframe, textvariable=self.lineVisible, font=('Helvetica',12));
    self.lineLabel.grid(row=0, column=0);

    self.oiModeLabel = ttk.Label(self.mainframe, textvariable=self.current_oi_mode, font=('Helvetica',12));
    self.oiModeLabel.grid(row=10,column=1);

    self.sonarLabel = ttk.Label(self.mainframe, textvariable=self.sonarStatus, font=('Helvetica',12));
    self.sonarLabel.grid(row=11,column=1, sticky=W);

    ttk.Button(self.mainframe, text="Go To A", style='my.TButton', command=self.goToA, width=16).grid(row=2, rowspan=2, column=0, pady=25)
    ttk.Button(self.mainframe, text="Go To B", style='my.TButton', command=self.goToB, width=16).grid(row=2, rowspan=2, column=1, pady=25)
    ttk.Button(self.mainframe, text="STOP", style='my.TButton', command=self.Stop, width=16).grid(row=4, rowspan=3, column=0, columnspan=3, pady=5)
    ttk.Button(self.mainframe, text="Dock", style='my.TButton', command=self.dock, width = 16).grid(row=8,column=0, pady=5)
    ttk.Button(self.mainframe, text="Un-Dock", style='my.TButton', command=self.undock, width = 16).grid(row=8,column=1, pady=5)

    ttk.Button(self.mainframe, text="Reset", style='my.TButton', command=self.resetPressed, width=16).grid(row=10, column=0, pady=5)

    self.root.after(1000, self.updateLabel);
    
    ##END FROM GUI##
 
  def resetPressed(self):
      tkMessageBox.showerror("Error", "Trying to Reset The Robot, Please wait 10 sec..")
      self.publish("CONTROLLER_Reset");
      time.sleep(9);

  def goToA(self):
      rospy.loginfo("GotoA");
      if(self.last_gui_state != "GoToA"):
          self.guiButtonUpdate("GoToA");
  def goToB(self):
      rospy.loginfo("GoToB");
      if(self.last_gui_state != "GoToB"):
          self.guiButtonUpdate("GoToB");

  def Stop(self):
      rospy.loginfo("Stop");
      if(self.last_gui_state != "Stop"):
          self.guiButtonUpdate("Stop");

  def dock(self):
      if(self.docked):
          tkMessageBox.showerror("Error", "Already Docked")
      else:
          self.publish("CONTROLLER_Dock");

  def undock(self):
      if(not self.docked):
          tkMessageBox.showerror("Error", "Not Docked")
      else:
        self.publish("CONTROLLER_UnDock");
        if(self.heading == "A"):
          self.heading = "B";
        elif(self.heading == "B"):
          self.heading = "A";

  def updateLabel(self):
      self.currentStatus.set("State: " + self.last_gui_state);
      self.headingStatus.set("Heading: " + self.heading);
      self.statusLabel.update_idletasks();
      self.batteryLabel.update_idletasks();
      self.headLabel.update_idletasks();
      self.lineLabel.update_idletasks();
      self.oiModeLabel.update_idletasks();
      self.sonarLabel.update_idletasks();

      self.root.update_idletasks();
      self.root.after(200, self.updateLabel);
 
  def lineVisibleCallback(self,msg):
      self.lineVisible.set("Line: " + str(msg.data));

  def current_mode_callback(self,msg):
      self.current_oi_mode.set("OI Mode: " + msg.data);

  def sonarCallback(self,msg):
      if(msg.data):
        self.sonarStatus.set('Safe To Drive');
      else:
        self.sonarStatus.set('Obstruction');

  def batteryCallback(self,msg):
      self.docked = msg.dock;
      self.batteryStatus.set(str("%.2f" % round(msg.level,2))+"%, Docked: " + str(self.docked));
      #if(msg.level < 0 or msg.level > 100):
      #  tkMessageBox.showerror("Error", "Trying to Reset The Robot, Please wait..")
      #  self.publish("CONTROLLER_Reset");
      #  time.sleep(9);
        

       
  def publish(self,msg):
      self.master_state_pub.publish(msg);
      self.last_state_published = msg;
      rospy.loginfo("Published: " + msg);

  def turnAround(self):
      self.publish("CONTROLLER_Turn");
      if(self.heading == "A"):
          self.heading = "B";
      elif(self.heading == "B"):
          self.heading = "A";

  def guiButtonUpdate(self,msg):
      if(msg == "GoToA"):
          call(["rosservice", "call", "/raspicam_node/start_capture"]);
          if(self.last_gui_state == "Stop"):
              if(self.heading == "A"):
                  self.publish("CONTROLLER_FollowLine");
              elif(self.heading == "B"):
                  self.turnAround();
                  #self.publish("CONTROLLER_FollowLine");

      elif(msg == "GoToB"):
          call(["rosservice", "call", "/raspicam_node/start_capture"]);
          if(self.last_gui_state == "Stop"):
              if(self.heading == "B"):
                  self.publish("CONTROLLER_FollowLine");
              elif(self.heading == "A"):
                  self.turnAround();
                  #self.publish("CONTROLLER_FollowLine");

      elif(msg == "Stop"):
          self.publish("CONTROLLER_Stop");
          call(["rosservice", "call", "/raspicam_node/stop_capture"]);
      
      else:
          rospy.logwarn("Unknown state: " + msg);
    
      self.last_gui_state = msg;

def main(args):
  rospy.init_node('create_eyes_state_publisher', anonymous=True)
  ic = Create2StatePublisher()
  ic.root.mainloop();
  #rospy.spin() is not needed because all it does is a blocking call which our gui mainloop will also do     

if __name__ == '__main__':
    main(sys.argv)

