

class Mod2Class(object):
	def __init__(self, pName):
		self.myName = pName + " from obj2 _init_"
		self.testVal = 2

	def getMyName(self):
		"""
		getMyName in Mod2Class returns the name all uppercase
		"""

		return self.myName.upper()
