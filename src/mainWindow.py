from PyQt4 import QtCore, QtGui
from ui_dialog import Ui_Dialog
import sys
import time

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

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    gui = GUI();
    ui = Ui_Dialog();
    ui.setupUi(gui);
    ui.lineLabel.setText("HELLO")
    gui.show()
    sys.exit(app.exec_())
