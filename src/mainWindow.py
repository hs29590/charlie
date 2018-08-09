from PyQt4 import QtCore, QtGui
from ui_dialog import Ui_Dialog
import sys
import time
import datetime

class GUI(QtGui.QDialog):
    
    def __init__(self, *args, **kwargs):
        super(GUI, self).__init__(*args, **kwargs)
        self.dummy = "H";

    def goButtonPressed(self):
        print "Go";
        time.sleep(10);
    def srcListItemClicked(self, item):
        print "Srclist" + item.text();
    def dstListItemClicked(self, item):
        print "dstlist" + item.text();
    def stopButtonPressed(self):
        print "stop";
    def resetButtonPressed(self):
        print "reset";
    def dockButtonPressed(self):
        print "dock";
    def blankButtonPressed(self):
        print "blank";
    def undockButtonPressed(self):
        print "undock";
    def turnButtonPressed(self):
        print "turn";
    def pauseButtonPressed(self):
        print "pause";

class TEST():
    def __init__(self):
        self.app = QtGui.QApplication(sys.argv)
        self.gui = GUI();
        self.ui = Ui_Dialog();
        self.ui.setupUi(self.gui);
        self.ui.lineLabel.setText("HELLO")
    def update_label(self):
        print("in label update");
        current_time = str(datetime.datetime.now().time())
        self.ui.lineLabel.setText(current_time)
        


if __name__ == "__main__":
    test = TEST()
    test.gui.show()
    timer = QtCore.QTimer()
    timer.timeout.connect(test.update_label)
    timer.start(1000)  # every 10,000 milliseconds
    test.app.exec_();
    test.ui.lineLabel.setText("Hello 2");
    test.ui.lineLabel.setText("Hello 3");
    test.ui.lineLabel.setText("Hello 4");
    sys.exit()













