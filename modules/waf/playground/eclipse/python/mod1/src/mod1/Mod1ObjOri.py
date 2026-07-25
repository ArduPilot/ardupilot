
"""
Class mod1 for tests

Doctest examples:

>>> a = Mod1Class("pippo")
>>> a.getMyName()
'pippo from obj1 _init_'

>>> a = Mod1Class("pLuTo")
>>> a.getMyName()
'pLuTo from obj1 _init_'

"""

class Mod1Class(object):

	def __init__(self, pName):
		"""
		Constructor stores pName in myName and appends the class string marker
		"""
		self.val = 1
		self.myName = pName  + " from obj1 _init_"

	def getMyName(self):
		"""
		getMyName in Mod1Class returns the name as is
		"""
		return self.myName
