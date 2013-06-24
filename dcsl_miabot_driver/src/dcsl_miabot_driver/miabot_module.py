#!/usr/bin/env python

import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class MiabotPlugin(Plugin):
    
    def __init__(self, context):
        super(MiabotPlugin, self).__init__(context)
        self.setObjectName('MiabotPlugin')
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'miabot_module.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MiabotPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). 
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget) 

    def shutdown_plugin(self):
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
