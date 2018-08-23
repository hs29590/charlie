#!/usr/bin/env python
from __future__ import print_function
import threading
from threading import Lock
import roslib
import rospy
roslib.load_manifest('charlie')
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import sys
import time

class ReadFromScanner():

    def __init__(self, port):
        self.hid = { 4: 'S', 5: 'B', 6: 'C', 7: 'D', 8: 'E', 9: 'F', 10: 'G', 11: 'H', 12: 'I', 13: 'J', 14: 'K', 15: 'L', 16: 'M', 17: 'N', 18: 'O', 19: 'P', 20: 'Q', 21: 'R', 22: 'S', 23: 'T', 24: 'U', 25: 'V', 26: 'W', 27: 'X', 28: 'Y', 29: 'Z', 30: '1', 31: '2', 32: '3', 33: '4', 34: '5', 35: '6', 36: '7', 37: '8', 38: '9', 39: '0', 44: ' ', 45: '-', 46: '=', 47: '[', 48: ']', 49: '\\', 51: ';' , 52: '\'', 53: '~', 54: ',', 55: '.', 56: '/'  }
        try:
            self.fp = open(port, 'rb')
        except:
            rospy.logfatal("BARCODE READER NOT CONNECTED...");            
        self.decodedString = '';
        self.pastDecodedString = '';
        t1 = threading.Thread(target=self.readThread);
        t2 = threading.Thread(target=self.checkNewData);
        self.newDataAvailable = False;
        self.maxTimeBetweenCharacters = 0.1;
        self.lastCharacterReadTime = time.time();
        self.mutex = Lock()
        self.barcodeDataPub = rospy.Publisher('/barcode_data', String, queue_size=1);
        t1.daemon = True;
        t2.daemon = True;
        t1.start()
        t2.start();
    
    def __del__(self):
        self.fp.close();

    def checkNewData(self):
        while True:
            self.mutex.acquire()
            if(time.time() - self.lastCharacterReadTime > self.maxTimeBetweenCharacters):
            #if(self.pastDecodedString != self.decodedString):
                if(len(self.decodedString) > 0):
                    #print("\n\nString is: " ),
                    self.barcodeDataPub.publish(self.decodedString);
                    #print(self.decodedString)
                    #print("\n\n");
                    self.pastDecodedString = self.decodedString;
                    self.decodedString = '';
            self.mutex.release();
            time.sleep(0.2);

    def readThread(self):
        while True:
            buffer = self.fp.read(8)
            self.mutex.acquire();
            for c in buffer:
                if ord(c) > 0:
                    try:
                        self.decodedString = self.decodedString + self.hid[int(ord(c))]
                        self.lastCharacterReadTime = time.time();
                    except:
                        #print "passing ", str(ord(c))
                        pass
            self.mutex.release();

if __name__ == "__main__":
  rospy.init_node('barcode_node_charlie', anonymous=True)
  if(rospy.has_param('~barcode_port')):
      barcode_port = rospy.get_param('~barcode_port')
  else:
      barcode_port = '/dev/hidraw0';
  R = ReadFromScanner(barcode_port)
  rospy.spin()
  #b = raw_input('Press any key to exit..');
