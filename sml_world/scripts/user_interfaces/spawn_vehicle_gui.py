#!/usr/bin/python
"""
Basic GUI interface for adding a new vehicle.

Created on Apr 5 2016

@author danielcarballal

"""

import wx
#from sml_world import ROSLaunchExtended

class spawn_vehicle_gui(wx.Frame):

	def __init__(self,parent,id,title):
		wx.Frame.__init__(self,parent,id,title)
		self.parent = parent

		self.initialize()

	def initialize(self):
		sizer = wx.GridBagSizer()

		self.SetBackgroundColour(wx.BLUE)

		self.dropdown = wx.ComboBox(self, -1, choices=["BaseVehicle", "DummyVehicle", "WifiVehicle", "Bus"])
		sizer.Add(self.dropdown,(0,0),(1,1),wx.EXPAND)

		button = wx.Button(self,-1,label="Create vehicle!")
		sizer.Add(button, (1,4))

		self.name = wx.TextCtrl(self,-1,value=u"Vehicle name", style=0)
		sizer.Add(self.name,(2,0),(2,1),wx.EXPAND)

		button.Bind( wx.EVT_BUTTON, self.submit_spawn )

		self.speed = wx.TextCtrl(self, -1, value=u"Speed", style=0)
		sizer.Add(self.speed, (4,0), (4,1), wx.EXPAND)

		self.x = wx.TextCtrl(self, -1, value=u"x", style=0)
		sizer.Add(self.x, (8,0), (8,0))

		self.y = wx.TextCtrl(self, -1, value=u"y", style=0)
		sizer.Add(self.y, (8,1), (8,1))

		self.yaw = wx.TextCtrl(self, -1, value=u"yaw", style=0)
		sizer.Add(self.yaw, (8,2), (8,2))

		self.SetSizerAndFit(sizer)
		self.Show(True)


	def submit_spawn(frame, event): #Seems to require two args
		print self.dropdown.GetString
		req = name + " " + self.dropdown.GetString + " " + self.x + " " + self.y + " " + self.yaw + " " + self.speed 
		print req
		#ROSLaunchExtended.handle_spawn_vehicle()
		print "Submitting spawn"


if __name__ == "__main__":
	app = wx.App()
	frame = spawn_vehicle_gui(None,-1,'Spawn a vehicle')
	app.MainLoop()