# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created: Mon Nov  2 15:17:13 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(668, 406)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralWidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.tabWidget = QtGui.QTabWidget(self.centralWidget)
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.scenario1 = QtGui.QWidget()
        self.scenario1.setObjectName(_fromUtf8("scenario1"))
        self.tabWidget.addTab(self.scenario1, _fromUtf8(""))
        self.scenario2 = QtGui.QWidget()
        self.scenario2.setObjectName(_fromUtf8("scenario2"))
        self.tabWidget.addTab(self.scenario2, _fromUtf8(""))
        self.scenario3 = QtGui.QWidget()
        self.scenario3.setObjectName(_fromUtf8("scenario3"))
        self.tabWidget.addTab(self.scenario3, _fromUtf8(""))
        self.horizontalLayout.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 668, 25))
        self.menuBar.setObjectName(_fromUtf8("menuBar"))
        self.menuSave_congfig = QtGui.QMenu(self.menuBar)
        self.menuSave_congfig.setObjectName(_fromUtf8("menuSave_congfig"))
        MainWindow.setMenuBar(self.menuBar)
        self.actionSave = QtGui.QAction(MainWindow)
        self.actionSave.setObjectName(_fromUtf8("actionSave"))
        self.actionLoad = QtGui.QAction(MainWindow)
        self.actionLoad.setObjectName(_fromUtf8("actionLoad"))
        self.menuSave_congfig.addAction(self.actionSave)
        self.menuSave_congfig.addAction(self.actionLoad)
        self.menuBar.addAction(self.menuSave_congfig.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.scenario1), _translate("MainWindow", "Scenario 1", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.scenario2), _translate("MainWindow", "Scenario 2", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.scenario3), _translate("MainWindow", "Scenario 3", None))
        self.menuSave_congfig.setTitle(_translate("MainWindow", "Config", None))
        self.actionSave.setText(_translate("MainWindow", "Save", None))
        self.actionLoad.setText(_translate("MainWindow", "Load", None))

