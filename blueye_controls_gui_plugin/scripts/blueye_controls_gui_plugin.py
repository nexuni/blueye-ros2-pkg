#!/usr/bin/env python3

import sys

from blueye_controls_gui_plugin.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'blueye_controls_gui_plugin'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
