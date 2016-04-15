#!/usr/bin/env python
"""
Connected Mobility Arena Presentation GUI Prototype.

@author: U{Marina Rantanen<marinar@kth.se>}
"""
import os
import gtk
import pygtk
pygtk.require('2.0')


class ButtonWindow(gtk.Window):
    """Create GUI window."""

    def on_queue_clicked(self, button):
        print("\"Queue event\" button was clicked")
        statsimagepath = self.base_path + '/resources/stats.jpg'
        self.statsimage.set_from_file(statsimagepath)

    def on_road_clicked(self, button):
        print("\"Road event\" button was clicked")
#        self.statsimage.set_from_file("jobstats.jpg")
        statsimagepath = self.base_path + '/resources/jobstats.jpg'
        self.statsimage.set_from_file(statsimagepath)

    def on_home_clicked(self, button):
        print("\"Hero event\" button was clicked")
#        self.statsimage.set_from_file("homestats.jpg")
        statsimagepath = self.base_path + '/resources/homestats.jpg'
        self.statsimage.set_from_file(statsimagepath)
        passtext = gtk.TextBuffer().set_text('Is this showing at all????')
        self.passview = gtk.TextView(buffer=passtext)
        self.Passtats.add(self.passview)

    def on_concert_clicked(self, button):
        print("\"Subway event\" button was clicked")
#        self.statsimage.set_from_file("concertstats.jpg")
        statsimagepath = self.base_path + '/resources/concertstats.jpg'
        self.statsimage.set_from_file(statsimagepath)

    def on_redbus_clicked(self, button):
        print("\"Red bus\" button was clicked")
        os.system("rosservice call spawn_vehicle \"{vehicle_id: 1, class_name: 'Bus', x: 0.0, y: 0.0, yaw: 0.0, v: 10.0, node_id: -384, toggle_sim: true}\" ")

    def on_greenbus_clicked(self, button):
        print("\"Green bus\" button was clicked")
        os.system("rosservice call spawn_vehicle \"{vehicle_id: 2, class_name: 'Bus', x: 0.0, y: 0.0, yaw: 0.0, v: 12.0, node_id: -100, toggle_sim: true}\" ")
#    def on_stop_clicked(self, button):
#        print("\"Stop\" button was clicked")

#    def on_log_clicked(self, button):
#        def on_close_clicked(button):
#            print("Closing application")
#            gtk.main_quit()
#            print("\"Log\" button was clicked")
#            logwin = gtk.Window()
#        logwin.set_size_request(500, 500)
#        vbox = gtk.VBox()
#        fixed = gtk.Fixed()
#        logwin.add(vbox)
#        vbox.add(fixed)
#        closeButton = gtk.Button("_Close", use_underline=True)
#        closeButton.connect("clicked", on_close_clicked)
#        fixed.put(closeButton, 0, 0)
#        logwin.show_all()

    def on_close_clicked(self, button):
        print("Closing application")
        gtk.main_quit()

    def __init__(self):
        gtk.Window.__init__(self)
        self.set_size_request(1200, 650)

        self.base_path = '/home/mma/catkin_ws/src/sml_world/scripts'

        self.bkg = gtk.Image()
        self.bkgimagepath = self.base_path + '/resources/bkg_four.png'
        self.bkg.set_from_file(self.bkgimagepath)

        self.statsimage = gtk.Image()
        self.statsimagepath = self.base_path + '/resources/stats.jpg'
        self.statsimage.set_from_file(self.statsimagepath)

        vbox = gtk.VBox()
        fixed = gtk.Fixed()
        self.add(vbox)
        vbox.add(fixed)

#        color = gtk.gdk.color_parse("pink")
        standardgray = gtk.gdk.Color(red = 25000, green = 25000,blue = 25000, pixel = 0)

        Eventwindow = gtk.Label("Events ")
        Eventwindow.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))

        DynamicStats = gtk.Label("Dynamic routing stats")
        DynamicStats.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))

        StaticStats = gtk.Label("Traffic ")
        StaticStats.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))

        PassengerStats = gtk.Label("Passenger stats ")
        PassengerStats.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))

#        stopButton = gtk.Button("Stop")
#        redlabel = stopButton.get_children()[0]
#        stopButton.modify_bg(gtk.STATE_NORMAL, color)
#        redlabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('red'))
#        stopButton.connect("clicked", self.on_stop_clicked)
#        stopButton.set_size_request(60,170)

        queueevent = gtk.Button("Queue on \n highway")
        blacklabel = queueevent.get_children()[0]
        queueevent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        queueevent.set_size_request(110, 110)
        queueevent.connect("clicked", self.on_queue_clicked)

        roadevent = gtk.Button("Closed road")
        blacklabel = roadevent.get_children()[0]
        roadevent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        roadevent.set_size_request(110, 110)
        roadevent.connect("clicked", self.on_road_clicked)

        heroevent = gtk.Button("National hero\ncelebration")
        blacklabel = heroevent.get_children()[0]
        heroevent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        heroevent.set_size_request(110, 110)
        heroevent.connect("clicked", self.on_home_clicked)

        subwayevent = gtk.Button("Subway\nmeltdown")
        blacklabel = heroevent.get_children()[0]
        subwayevent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        subwayevent.set_size_request(110, 110)
        subwayevent.connect("clicked", self.on_concert_clicked)

        redbusline = gtk.Button("Start red\nbus line")
        blacklabel = redbusline.get_children()[0]
        redbusline.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        redbusline.set_size_request(110, 110)
        redbusline.connect("clicked", self.on_redbus_clicked)

        greenbusline = gtk.Button("Start green\nbus line")
        blacklabel = greenbusline.get_children()[0]
        greenbusline.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        greenbusline.set_size_request(110, 110)
        greenbusline.connect("clicked", self.on_greenbus_clicked)

        closeButton = gtk.Button("_Close", use_underline=True)
        blacklabel = closeButton.get_children()[0]
        closeButton.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        closeButton.set_size_request(187, 44)
        closeButton.connect("clicked", self.on_close_clicked)

        Dynstats = gtk.ScrolledWindow()
        Dynstats.set_size_request(170, 240)
        textbuffer = gtk.TextBuffer()
        textbuffer.set_text('Bus information \n Active buses: '
                            '\n Requested stops: \n Expected passenger'
                            '\n change next hour: ')
        textview = gtk.TextView(buffer=textbuffer)
        Dynstats.add(textview)

        Dynvalues = gtk.ScrolledWindow()
        Dynvalues.set_size_request(80, 240)
        textbuffer = gtk.TextBuffer()
        textbuffer.set_text(' \n 4 \n 57 \n \n +50 ')
        textview2 = gtk.TextView(buffer=textbuffer)
        Dynvalues.add(textview2)

        self.Passtats = gtk.ScrolledWindow()
        self.Passtats.set_size_request(170, 240)
        passtext = gtk.TextBuffer()
        passtext.set_text('Demand / \n Active requests: '
                          ' \n Expected passenger \n change next hour: ')
        self.passview = gtk.TextView(buffer=passtext)
        self.Passtats.add(self.passview)

        fixed.put(self.bkg, 0, 0)
        fixed.put(self.statsimage, 450, 70)

        fixed.put(queueevent, 60, 85)
        fixed.put(roadevent, 60, 200)

        fixed.put(heroevent, 180, 85)
        fixed.put(subwayevent, 180, 200)

        fixed.put(redbusline, 450, 385)
        fixed.put(greenbusline, 570, 385)

        fixed.put(closeButton, 980, 600)

        fixed.put(Eventwindow, 60, 40)
        fixed.put(PassengerStats, 60, 365)
        fixed.put(DynamicStats, 450, 40)
        fixed.put(StaticStats, 450, 365)

        fixed.put(Dynstats, 870, 70)
        fixed.put(Dynvalues, 1041, 70)
        fixed.put(self.Passtats, 60, 400)

win = ButtonWindow()
win.connect("delete-event", gtk.main_quit)
win.show_all()
gtk.main()
