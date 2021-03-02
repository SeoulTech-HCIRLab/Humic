import os
import rospkg
import rospy

from rqt_humicpkg.humic_widget import HumicWidget
from qt_gui.plugin import Plugin

class HumicController(Plugin):
    def __init__(self, context):
        super(HumicController, self).__init__(context)
        self.setObjectName('HumicController')
        self._widget = HumicWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)
    
    def shutdown_plugin(self):
        print('Reset Humic Controller.')
        self._widget.shutdown_widget()
        