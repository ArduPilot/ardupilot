#ifndef D_VirtualCall_H
#define D_VirtualCall_H

#define send(obj,msg)\
              ((obj)->msg(obj))

#define send1(obj,msg,arg0)\
              ((obj)->msg((obj),(arg0)))

#define send2(obj,msg,arg0,arg1)\
              ((obj)->msg((obj),(arg0),(arg1)))

#define send3(obj,msg,arg0,arg1,arg2)\
              ((obj)->msg((obj),(arg0),(arg1),(arg2)))

#define send4(obj,msg,arg0,arg1,arg2,arg3)\
              ((obj)->msg((obj),(arg0),(arg1),(arg2),(arg3)))

#define vBind(obj,msg,newMethod)\
              (obj->msg=&newMethod)

#define castToDestroyer(Class) (Class* (*)(Class*))

#endif
