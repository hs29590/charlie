#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('charlie')
from std_msgs.msg import String
from std_msgs.msg import Bool
import sys
import rospy
import time
import pigpio


class SonarClass:
   
    def cbfunc1(self,gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick1 is not None:
             echo = pigpio.tickDiff(self.high_tick1, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo1 was {} micros long ({:.1f} cms)".format(echo, cms))
             self.echo1Distance = self.echo1Distance*0.5 + 0.5*cms;
       else:
          self.high_tick1 = tick

    def cbfunc2(self,gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick2 is not None:
             echo = pigpio.tickDiff(self.high_tick2, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo2 was {} micros long ({:.1f} cms)".format(echo, cms))
             self.echo2Distance = self.echo2Distance*0.5 + 0.5*cms;
       else:
          self.high_tick2 = tick

    '''    def cbfunc3(self,gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick3 is not None:
             echo = pigpio.tickDiff(self.high_tick3, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo3 was {} micros long ({:.1f} cms)".format(echo, cms))
             self.echo3Distance = self.echo3Distance*0.5 + 0.5*cms;
       else:
          self.high_tick3 = tick
   '''       
    def __init__ (self):

        self.TRIGGER=19
        self.ECHO1=21
        self.ECHO2=26
        self.ECHO3=20
   
        self.echo1Distance = 1000;
        self.echo2Distance = 1000;
#       self.echo3Distance = 1000;

        self.USSensorStoppingRange = 20.0;

        self.high_tick1 = None # global to hold high tick.
        self.high_tick2 = None # global to hold high tick.
#       self.high_tick3 = None # global to hold high tick.
    
        self.state_pub = rospy.Publisher('sonar_drive', Bool, queue_size=1)
        self.safeToDrive = True;

        self.pi = pigpio.pi() # Connect to local Pi.

        self.pi.set_mode(self.TRIGGER, pigpio.OUTPUT)
        self.pi.set_mode(self.ECHO1, pigpio.INPUT)
        self.pi.set_mode(self.ECHO2, pigpio.INPUT)
        self.pi.set_mode(self.ECHO3, pigpio.INPUT)

        self.cb1 = self.pi.callback(self.ECHO1, pigpio.EITHER_EDGE, self.cbfunc1)
        self.cb2 = self.pi.callback(self.ECHO2, pigpio.EITHER_EDGE, self.cbfunc2)
#self.cb3 = self.pi.callback(self.ECHO3, pigpio.EITHER_EDGE, self.cbfunc3)
   
    def __del__(self):
        self.cb1.cancel() # Cancel callback.
        self.cb2.cancel() # Cancel callback.
#       self.cb3.cancel() # Cancel callback.
        self.pi.stop() # Close connection to Pi 

    def triggerAndPublish(self):
        self.safeToDrive = True;
        self.pi.gpio_trigger(self.TRIGGER, 10);
        time.sleep(0.02);
        
#if(self.echo1Distance < self.USSensorStoppingRange or self.echo2Distance < self.USSensorStoppingRange or self.echo3Distance < self.USSensorStoppingRange):
        if(self.echo1Distance < self.USSensorStoppingRange or self.echo2Distance < self.USSensorStoppingRange):
            self.state_pub.publish(False);
        else:
            self.state_pub.publish(True);


def main(args):
  rospy.init_node('sonar_node_create', anonymous=True)
  sc = SonarClass()
#  rospy.spin();
  while not rospy.is_shutdown():
    sc.triggerAndPublish(); #this sleeps for 0.2s internally
        
if __name__ == '__main__':
    main(sys.argv)

