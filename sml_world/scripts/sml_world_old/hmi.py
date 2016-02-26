import signal
import sys
from PyQt4 import QtGui, uic
from PyQt4.QtCore import *
import inspect
from threading import Thread, Timer
import time
from PyQt4.QtGui import QApplication, QMessageBox
import pprint
import math

import util

class HMI(Thread):
    def __init__(self,SML):
        Thread.__init__(self)
        self.SML = SML
        self.hmi = []

    def run(self):
        self.app = QtGui.QApplication(sys.argv)
        self.hmi.append(HMIWindow(self.SML))
        return self.app.exec_()

    def update(self):
        while self.hmi==[]:
            time.sleep(0.1)
            print "Wait for HMI"
        if self.hmi!=[]:
            keyList = sorted(self.SML.bodies_dict.keys())
            for k in keyList:
                self.hmi[0].listVehicle.addItem(str(k))

    def kill(self):
        self.hmi[0].close()

class HMIWindow(QtGui.QMainWindow):
    def __init__(self,SML):
        super(HMIWindow, self).__init__()

        self.SML = SML

        uic.loadUi('hmi.ui', self)
        self.brake.clicked.connect(self.brakeVehicle)
        self.timer = QTimer()
        self.timer.start(100) # lancer un top toutes les secondes (=1000 millisecondes)
        self.connect(self.timer, SIGNAL("timeout()"), self.getState)
        self.connect(self.timer, SIGNAL("timeout()"), self.appendConsole)
        self.show()

    def getVehicleSelected(self):
        if len(self.listVehicle.selectedItems())!=0:
            return int(self.listVehicle.selectedItems()[0].text())
        else:
            return None

    def brakeVehicle(self):
        idx = self.getVehicleSelected()
        self.SML.bodies_dict[idx].control_module.brake = not self.SML.bodies_dict[idx].control_module.brake
        print "Brake : "+str(idx)+ " = " + str(self.SML.bodies_dict[idx].control_module.brake)

    def getState(self):
        idx = self.getVehicleSelected()
        if idx in self.SML.bodies_dict.keys():
            vehicle = self.SML.bodies_dict[idx]
            linear_speed = math.hypot(vehicle.x_speed, vehicle.y_speed)
            state = {"brake":vehicle.control_module.brake,
                     "ACCC":str(vehicle.control_module.ACCC)+" target "+str(vehicle.control_module.ACCC_target_id),
                     "speed": str(linear_speed),
                     "thottle": str(vehicle.commands['throttle']),
                     "State":vehicle.supervisory_module.state}
            
            self.state.setText(pprint.pformat(state,width=1))
        else:
            self.state.setText("")

    def appendConsole(self):
        d = util.get_print()
        for l in d:
            self.console.append(l)

        if len(d)!=0:
            self.console.moveCursor(QtGui.QTextCursor.End)

if __name__ == '__main__':
    hmi = HMI([])
    hmi.start()

    signal.signal(signal.SIGINT, lambda s,f: hmi.kill())
    signal.pause()
