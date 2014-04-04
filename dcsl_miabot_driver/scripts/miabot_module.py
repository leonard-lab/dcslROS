#!/usr/bin/env python

## @file
# An rqt plugin for connecting to a Merlin MiabotPro using the dcsl_miabot_node.py node through the ConnectMiabotAction.
#
# The ui module called miabot_module.ui is imported. It was generated using the program Qt Creator.
#
# A dropdown is provide to indicate which Miabot to connect to and a button connects, disconnects, and changes color to provide feedback.
#
# See http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin and http://wiki.ros.org/rqt/Tutorials/Using%20.ui%20file%20in%20rqt%20plugin for tutorials on rqt.

import os

import rospy
import rospkg

import actionlib
from dcsl_miabot_driver.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QWidget

## This class defines and implements the behavior of the plugin. It inherits the plugin class.
#
# It provides dropdowns for selecting a robot to connect to and buttons to connect/disconnect. The button changes color to provide feedback.
class MiabotPlugin(Plugin):
    

    ## Sets up the GUI, attaches callbacks to buttons, sets up client action server.
    #
    # @param context is required by and passed to the superclass.
    def __init__(self, context):

        # Miabot connection parameters
        self.n_robots = rospy.get_param('n_robots')
        self.connected_list = [False]*self.n_robots
        self.miabot_assignments = [None]*7

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

        # Set up combo box callbacks
        items = ['', '0', '1', '2', '3', '4', '5', '6']
        self.comboBoxList = [self._widget._r0_comboBox, self._widget._r1_comboBox, self._widget._r2_comboBox,
                        self._widget._r3_comboBox, self._widget._r4_comboBox, self._widget._r5_comboBox, 
                        self._widget._r6_comboBox]
        for i, cb in enumerate(self.comboBoxList):
            cb.addItems(items)
            

        '''
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        self.destroyed.connect(self.handle_destroy)
        '''

    ## Callback function for button clicks
    #
    # On a button click, if connected, a disconnect signal is sent and if disconnect connect signal sent. The selected robot to connect from the associated combo box is also sent over the action server. If the action is successful, the button turns green and if the action fails it turns red.
    # @param button is the associated button object so that its appearance can be modified.
    # @param robot_ID is the ID in the system (not on the robot itself) associated with the button.
    def _r_clicked(self, button, robot_ID):

        if self.connected_list[robot_ID] is False:
            if str(self.comboBoxList[robot_ID].currentText()) is '':
                button.setStyleSheet('QPushButton {color: red}')
                return

            goal = ConnectMiabotGoal(True, int(self.comboBoxList[robot_ID].currentText()))
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
            goal = ConnectMiabotGoal(False, -1)
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

    ## Called on shutdown of the widget.
    #
    # Not currently inplemented.
    def shutdown_plugin(self):
        pass
    
    ## Called on save settings command.
    #
    # Not currently implemented.
    def save_settings(self, plugin_settings, instance_settings):
        pass

    ## Called on restore settings.
    #
    # Not currently implemented.
    def restore_settings(self, plugin_settings, instance_settings):
        pass
