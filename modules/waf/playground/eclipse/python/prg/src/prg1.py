#! /usr/bin/env python
# encoding: utf-8

import mod1.Mod1ObjOri
import mod2.Mod2ObjOri
import mod3.Mod3ObjOri


if __name__ == '__main__':
	print("Creating obj1 with string pippo")
	obj1 = mod1.Mod1ObjOri.Mod1Class("pippo")

	a = 1

	print("Creating obj1 with string pLUto")
	obj2 = mod2.Mod2ObjOri.Mod2Class("pLUto")

	print("Creating obj3 with string pLUto")
	obj3 = mod3.Mod3ObjOri.Mod3Class("pLUto")

	print("Hello World, this are my results:")
	print(obj1.getMyName())
	print(obj2.getMyName())
	print(obj3.getMyName())
