#!/usr/bin/env python

import os

import rospy
import rospkg

import actionlib
from dcsl_vision_tracker.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, qApp
from python_qt_binding.QtCore import Qt

class TrackerPlugin(Plugin):
    
    def __init__(self, context):
        super(TrackerPlugin, self).__init__(context)
               
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

        # Setup button callbacks
        self._widget.tracker_checkBox.toggled[bool].connect(self._check_callback)      
        self._widget.reset_pushButton.clicked.connect(self._reset_callback)

        self._widget.window_pushButton.clicked.connect(self._window_callback)

        self._widget.background_pushButton.clicked.connect(self._background_callback)
        self.block_background = False

        self._widget.background_progressBar.hide() # Hide because feedback not working.

        # Setup combo box
        items = ['0', '1', '2', '3']
        self._widget.comboBox.addItems(items)
        self.window_states = [False]*4
        '''
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        self.destroyed.connect(self.handle_destroy)
        '''

        # Initialize action clients
        timeout = 10 # seconds
        dur = rospy.Duration(timeout)
        name = "dcsl_vt_toggle_tracking"
        self.tracking_client = actionlib.SimpleActionClient(name, ToggleTrackingAction)
        self.tracking_client.wait_for_server(dur)

        name = "dcsl_vt_window"
        self.window_client = actionlib.SimpleActionClient(name, ToggleWindowAction)
        self.tracking_client.wait_for_server(dur)

        name = "dcsl_vt_background"
        self.background_client = actionlib.SimpleActionClient(name, GenerateBackgroundAction)
        self.background_client.wait_for_server(dur)

    def _check_callback(self, state):
        goal = ToggleTrackingGoal(state,False)
        self.tracking_client.send_goal(goal)
        self.tracking_client.wait_for_result()
        response = self.tracking_client.get_result()
        if response.tracking is not state:
            self._widget.tracker_checkBox.setCheckState(Qt.Checked if state else Qt.Unchecked)

    def _reset_callback(self):
        goal = ToggleTrackingGoal(False, True)
        self.tracking_client.send_goal(goal)
        self.tracking_client.wait_for_result()
        response = self.tracking_client.get_result()
        self._widget.tracker_checkBox.setCheckState(Qt.Checked if response.tracking else Qt.Unchecked)

    def _window_callback(self):
        camera_id = int(self._widget.comboBox.currentText())
        goal = ToggleWindowGoal(not self.window_states[camera_id], camera_id)
        self.window_client.send_goal(goal)
        self.window_client.wait_for_result()
        response = self.window_client.get_result()
        if response.success:
            self.window_states[camera_id] = not self.window_states[camera_id]
                                
    def _background_callback(self):
        self._widget.background_pushButton.setStyleSheet('QPushButton {color: yellow}')
        qApp.processEvents()
        goal = GenerateBackgroundGoal(True)
        self._widget.background_progressBar.setValue(0)
        # self.background_client.send_goal(goal, done_cb = self._background_done_callback, feedback_cb = self._background_feedback_callback) #done_cb feedback_cb aren't working due to threading.
        self.background_client.send_goal(goal)
        self.background_client.wait_for_result()
        response = self.background_client.get_result()
        if response.successful:
            self._widget.background_pushButton.setStyleSheet('QPushButton {color: green}')
        else:
            self._widget.background_pushButton.setStyleSheet('QPushButton {color: red}')

    def _background_done_callback(self, status, result):
        self.block_background = False
        self._widget.background_progressBar.setValue(100)

    def _background_feedback_callback(self, feedback):
        print int(feedback.progress*100.0)
        self._widget.background_progressBar.setValue(int(feedback.progress*100.0))

    def shutdown_plugin(self):
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
