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

        # Miabot connection parameters
        self.n_robots = rospy.get_param('n_robots')
        self.connected_list = [False]*self.n_robots

        # Initialize action clients
        self.client = []
        for i in xrange(0, self.n_robots):
            name = 'miabot' + str(i)
            self.client.append(actionlib.SimpleActionClient(name, ConnectMiabotAction))
            self.client[i].wait_for_server()

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
        self._widget._r0_button.clicked.connect(lambda: self._r_clicked(self._widget._r0_button, 0))      
        self._widget._r1_button.clicked.connect(lambda: self._r_clicked(self._widget._r1_button, 1))
        self._widget._r2_button.clicked.connect(lambda: self._r_clicked(self._widget._r2_button, 2))
        self._widget._r3_button.clicked.connect(lambda: self._r_clicked(self._widget._r3_button, 3))
        self._widget._r4_button.clicked.connect(lambda: self._r_clicked(self._widget._r4_button, 4))
        self._widget._r5_button.clicked.connect(lambda: self._r_clicked(self._widget._r5_button, 5))
        self._widget._r6_button.clicked.connect(lambda: self._r_clicked(self._widget._r6_button, 6))
        '''
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        self.destroyed.connect(self.handle_destroy)
        '''

        

    def _r_clicked(self, button, robot_ID):
        if self.connected_list[robot_ID] is False:
            goal = ConnectMiabotGoal(True)
            button.setStyleSheet('QPushButton {color: yellow}')
            self.client[robot_ID].send_goal(goal) # Thread this to improve performance
            self.client[robot_ID].wait_for_result()
            response = self.client[robot_ID].get_result()
            
            if response.connected is True:
                button.setStyleSheet('QPushButton {color: green}')
                button.setText("Disconnect")
                self.connected_list[robot_ID] = True
            else:
                button.setStyleSheet('QPushButton {color: red}')
                self.connected_list[robot_ID] = False
                button.setText("Connect")
        else:
            goal = ConnectMiabotGoal(False)
            button.setStyleSheet('QPushButton {color: yellow}')
            self.client[robot_ID].send_goal(goal)
            self.client[robot_ID].wait_for_result()
            response = self.client[robot_ID].get_result()
            if response.connected is False:
                button.setStyleSheet('QPushButton {color: green}')
                self.connected_list[robot_ID] = False
                button.setText("Connect")
            else:
                button.setStyleSheet('QPushButton {color: red}')
                self.connected_list[robot_ID] = True
                button.setText("Disconnect")

    def shutdown_plugin(self):
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
