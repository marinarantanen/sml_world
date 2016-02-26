import sys
from subprocess import call,Popen
from PyQt4 import QtGui, uic
from PyQt4.QtCore import *
import inspect
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)


class ConfigurationWindow(QtGui.QMainWindow):
    def __init__(self):
        super(ConfigurationWindow, self).__init__()
        uic.loadUi('ConfigurationWindow.ui', self)
        self.actionSave.triggered.connect(self.save)
        self.actionLoad.triggered.connect(self.load)
        self.launch.clicked.connect(self.launchSimulation)
        self.show()

        self.load("default.ini")

    def save(self):
        fileName = QtGui.QFileDialog.getSaveFileName(self, 'Save configuration file', selectedFilter='*.ini')
        if fileName:
            self.guisave(QSettings(fileName, QSettings.IniFormat))
            print "Save configuration file to "+fileName
            self.setWindowTitle(fileName)

    def load(self,fn=""):
        if fn=="":
            fileName = QtGui.QFileDialog.getOpenFileName(self, 'Load configuration file', selectedFilter='*.ini')
            if fileName:
                self.filename = fileName
        else:
            self.filename = fn

        self.guirestore(QSettings(self.filename, QSettings.IniFormat))
        print "Load configuration file from "+self.filename
        self.setWindowTitle(self.filename)

        
    def guisave(self, settings):
        ui = self
        #for child in ui.children():  # works like getmembers, but because it traverses the hierarachy, you would have to call guisave recursively to traverse down the tree
        for name, obj in inspect.getmembers(ui):
            #if type(obj) is QComboBox:  # this works similar to isinstance, but missed some field... not sure why?
            if isinstance(obj,  QtGui.QComboBox):
                name   = obj.objectName()      # get combobox name
                index  = obj.currentIndex()    # get current index from combobox
                text   = obj.itemText(index)   # get the text for current index
                settings.setValue(name, text)   # save combobox selection to registry

            if isinstance(obj,  QtGui.QLineEdit):
                name = obj.objectName()
                value = obj.text()
                settings.setValue(name, value)    # save ui values, so they can be restored next time

            if isinstance(obj,  QtGui.QCheckBox):
                name = obj.objectName()
                state = obj.checkState()
                settings.setValue(name, state)

            if isinstance(obj,  QtGui.QSpinBox):
                name = obj.objectName()
                state = obj.value()
                settings.setValue(name, state)

            if isinstance(obj,  QtGui.QDoubleSpinBox):
                name = obj.objectName()
                state = obj.value()
                settings.setValue(name, state)

    #===================================================================
    # restore "ui" controls with values stored in registry "settings"
    # currently only handles comboboxes, editlines &checkboxes
    # ui = QMainWindow object
    # settings = QSettings object
    #===================================================================

    def guirestore(self, settings):
        ui = self
        for name, obj in inspect.getmembers(ui):
            if isinstance(obj, QtGui.QComboBox):
                index  = obj.currentIndex()    # get current region from combobox
                #text   = obj.itemText(index)   # get the text for new selected index
                name   = obj.objectName()

                value = settings.value(name).toString()  

                if value == "":
                    continue

                index = obj.findText(value)   # get the corresponding index for specified string in combobox

                if index == -1:  # add to list if not found
                    obj.insertItems(0,[value])
                    index = obj.findText(value)
                    obj.setCurrentIndex(index)
                else:
                    obj.setCurrentIndex(index)   # preselect a combobox value by index    

            if isinstance(obj,  QtGui.QLineEdit):
                name = obj.objectName()
                value = settings.value(name).toString()  # get stored value from registry
                obj.setText(value)  # restore lineEditFile

            if isinstance(obj, QtGui.QCheckBox):
                name = obj.objectName()
                value = settings.value(name)   # get stored value from registry
                if value != None:
                    obj.setCheckState(value)   # restore checkbox

            if isinstance(obj,  QtGui.QSpinBox):
                name = obj.objectName()
                value = settings.value(name).toInt()
                obj.setValue(value[0])

            if isinstance(obj,  QtGui.QDoubleSpinBox):
                name = obj.objectName()
                value = settings.value(name).toFloat()
                obj.setValue(value[0])


            #if isinstance(obj, QRadioButton):                

    def launchSimulation(self):
        command = ["python","world_class.py",self.filename]
        Popen(command)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = ConfigurationWindow()
    sys.exit(app.exec_())