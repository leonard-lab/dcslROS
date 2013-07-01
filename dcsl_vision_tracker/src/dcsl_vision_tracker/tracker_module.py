#!/usr/bin/env python

import os

import rospy
import rospkg

import actionlib
from dcsl_vision_tracker.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Qt

class TrackerPlugin(Plugin):
    
    def __init__(self, context):
        super(TrackerPlugin, self).__init__(context)

        # Initialize action clients
        name = "dcsl_vision_tracker"
        self.client = actionlib.SimpleActionClient(name, ToggleTrackingAction)
        self.client.wait_for_server()
               
        # Get path to ui file.
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('dcsl_vision_tracker'), 'resource', 'tracker_module.ui')
        self.setObjectName('TrackerPlugin')

        # Create QWidget
        self._widget = QWidget()
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('TrackerPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). 
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget) 

        # Set up button callbacks
        self._widget.tracker_checkBox.toggled[bool].connect(self._check_callback)      
        self._widget.reset_pushButton.clicked.connect(self._reset_callback)
        '''
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        self.destroyed.connect(self.handle_destroy)
        '''

    def _check_callback(self, state):
        goal = ToggleTrackingGoal(state,False)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        response = self.client.get_result()
        if response.tracking is not state:
            self._widget.tracker_checkBox.setCheckState(Qt.Checked if state else Qt.Unchecked)

    def _reset_callback(self):
        goal = ToggleTrackingGoal(False, True)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        response = self.client.get_result()
        self._widget.tracker_checkBox.setCheckState(Qt.Checked if response.tracking else Qt.Unchecked)

    def shutdown_plugin(self):
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
