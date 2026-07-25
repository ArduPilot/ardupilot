#!/usr/bin/env python
# encoding: utf-8
# Federico Pellegrin, 2016 (fedepell)

#
# Example extra that chains to either qt5 or pyqt5 for QRC/UI files as
# just one handler for an extension can be natively defined. The extra
# has to be loaded after qt5 and pyqt5 and files need to have explicitly
# set the feature they want to use.
#

import os
from waflib.Tools import python
from waflib.Tools import cxx
from waflib.extras import pyqt5
from waflib.Tools import qt5
from waflib import Task
from waflib.TaskGen import extension
from waflib import Logs


EXT_RCC = ['.qrc']
"""
File extension for the resource (.qrc) files
"""

EXT_UI  = ['.ui']
"""
File extension for the user interface (.ui) files
"""


@extension(*EXT_RCC)
def create_chain_task(self, node):
	"Creates rcc and py task for ``.qrc`` files"
	if 'qt5' in self.features:
		qt5.create_rcc_task(self, node)
	elif 'pyqt5' in self.features:
		pyqt5.create_pyrcc_task(self, node)
	else:
		Logs.warn("No feature explicitly defined for '%s'",node)


@extension(*EXT_UI)
def create_chain_task(self, node):
	"Create uic tasks and py for user interface ``.ui`` definition files"
	if 'qt5' in self.features:
		qt5.create_uic_task(self, node)
	elif 'pyqt5' in self.features:
		pyqt5.create_pyuic_task(self, node)
	else:
		Logs.warn("No feature explicitly defined for '%s'",node)

