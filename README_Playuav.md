#APM Plus Hack#

We have made a hack of APM(ArduPilot Mega). We integrate an OSD module with the APM board which removes the necessity of external OSD module like miniosd.

我们在APM的板子上集成了一个OSD模块，这样就省去了外接OSD模块像miniosd的麻烦。

So communication between autopilot and OSD is realized through SPI bus rather than serial port of miniosd. As a result, the refresh rate is much faster than miniosd.

飞控和osd是通过SPI总线通信的，而不是像外置OSD通过串口。所以刷新率很快。

#Code hack#
The hacked code is located at branch PlaneOSD-3.2, CopterOSD-3.1.2 and CopterOSD-3.2.

修改的代码位于分支PlaneOSD-3.2, CopterOSD-3.1.2 and CopterOSD-3.2.

#Schematic#
You can find the PDF hardware layout at branch CopterOSD-3.12/APMPlus_HW

硬件的pdf放在 CopterOSD-3.12/APMPlus_HW

#More info#
Please visit PLAYUAV at: http://www.playuav.com

更多信息请访问：http://www.playuav.com
