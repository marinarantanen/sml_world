#!/usr/bin/env python
"""
Connected Mobility Arena Presentation GUI Prototype.

@author: U{Marina Rantanen<marinar@kth.se>}
"""

#import roslaunch
import os
import gtk
import pygtk
pygtk.require('2.0')


class StartWindow(gtk.Window):
    """Create GUI window."""

    def on_kista_clicked(self, button):
        print("\"Kista\" button was clicked")

        os.system("roslaunch sml_world starter.launch")

#        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#        roslaunch.configure_logging(uuid)
#        launch = roslaunch.parent.ROSLaunchParent(uuid, '/home/mma/catkin_ws/'
#                                                        'src/sml_world/launch/'
#                                                        'starter.launch')
#        launch.start()

    def on_close_clicked(self, button):
        print("Closing application")
        gtk.main_quit()

    def __init__(self):
        gtk.Window.__init__(self)
        self.set_size_request(402, 329)

        self.base_path = '/home/mma/catkin_ws/src/sml_world/scripts'

        self.bkg = gtk.Image()
        self.bkgimagepath = self.base_path + '/resources/bkg_one.png'
        self.bkg.set_from_file(self.bkgimagepath)

        vbox = gtk.VBox()
        fixed = gtk.Fixed()
        self.add(vbox)
        vbox.add(fixed)

        standardgray = gtk.gdk.Color(red = 25000, green = 25000,blue = 25000, pixel = 0)

        mapswindow = gtk.Label("Maps ")
        mapswindow.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))

        kistaevent = gtk.Button("Kista map")
        blacklabel = kistaevent.get_children()[0]
        kistaevent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        kistaevent.set_size_request(110, 110)
        kistaevent.connect("clicked", self.on_kista_clicked)

        closeButton = gtk.Button("_Close", use_underline=True)
        blacklabel = closeButton.get_children()[0]
        closeButton.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        closeButton.set_size_request(187, 44)
        closeButton.connect("clicked", self.on_close_clicked)

        fixed.put(self.bkg, 0, 0)
        fixed.put(kistaevent, 60, 85)
        fixed.put(closeButton, 180, 270)
        fixed.put(mapswindow, 40, 30)

win = StartWindow()
win.connect("delete-event", gtk.main_quit)
win.show_all()
gtk.main()
