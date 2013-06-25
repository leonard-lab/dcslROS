#!/usr/bin/env python

import os

import rospy
import rospkg

import actionlib
from dcsl_miabot_driver.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class MiabotPlugin(Plugin):
    
    def __init__(self, context):
        super(MiabotPlugin, self).__init__(context)
        rp = rospkg.RosPack()
        # Get path to ui file.
        ui_file = os.path.join(rp.get_path('dcsl_miabot_driver'), 'resource', 'miabot_module.ui')
        self.setObjectName('MiabotPlugin')

        # Create QWidget
        self._widget = QWidget()
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MiabotPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). 
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget) 

        # Set up button callbacks
        self._widget._r0_button.clicked.connect(self._r0_clicked)
        '''
        self._widget._r1_button.clicked[bool].connect(self._r1_clicked)
        self._widget._r2_button.clicked[bool].connect(self._r2_clicked)
        self._widget._r3_button.clicked[bool].connect(self._r3_clicked)
        self._widget._r4_button.clicked[bool].connect(self._r4_clicked)
        self._widget._r5_button.clicked[bool].connect(self._r5_clicked)
        self._widget._r6_button.clicked[bool].connect(self._r6_clicked)
        
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        self.destroyed.connect(self.handle_destroy)
        '''

        # Miabot connection parameters
        self.n_robots = 7
        self.connected_list = [False]*self.n_robots

        # Initialize action client
        self.client = actionlib.SimpleActionClient('miabot0', ConnectMiabotAction)

    def _r0_clicked(self):
        if self.connected_list[0] is False:
            self._widget._r0_button.setStyleSheet('QPushButton {color: green}')
            self.connected_list[0] = True
        else:
            self._widget._r0_button.setStyleSheet('QPushButton {color: red}')
            self.connected_list[0] = False

    def shutdown_plugin(self):
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
