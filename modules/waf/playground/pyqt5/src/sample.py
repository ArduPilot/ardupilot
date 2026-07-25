import sys
# If pyqt5 bindings are used uncomment the following line:
from PyQt5 import QtCore, QtGui, QtWidgets
# If pyside2 bindings are used uncomment the following line:
#from PySide2 import QtCore, QtGui, QtWidgets

from firstgui import Ui_myfirstgui

class MyFirstGuiProgram(Ui_myfirstgui):
	def __init__(self, dialog):
		Ui_myfirstgui.__init__(self)
		self.setupUi(dialog)

		# Connect "add" button with a custom function (addInputTextToListbox)
		self.addBtn.clicked.connect(self.addInputTextToListbox)

	def addInputTextToListbox(self):
		txt = self.myTextInput.text()
		self.listWidget.addItem(txt)

if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	dialog = QtWidgets.QDialog()

	prog = MyFirstGuiProgram(dialog)

	dialog.show()
	sys.exit(app.exec_())
