VERSION=0_0_2
PACKAGE=mod_gsoap_win_$(VERSION).zip
WWWSERVICENAME="WWW-Publishing"
LOGFILEPATH=$(SYSTEMROOT)\system32\inetsrv

DELDIRS = .\Debug .\Release \
    .\gsoap\Debug .\gsoap\Release


all:
	msdev all4iis.dsw /MAKE "all4iis - Win32 Debug" 
	msdev all4iis.dsw /MAKE "all4iis - Win32 Release" 

clean:
    -del /s/q *.ncb
    -del /s/q *.opt
    -del /s/q *.plg
    -del /s/q *.clw
	-del /s/q *.pch
	-del /s/q *.aps
	-del /s/q *.log
	-attrib -h -s images\Thumbs.db
	-del images\Thumbs.db

	cd samples
		$(MAKE) -f samples.mak clean
	cd ..
	for %v in ($(DELDIRS)) DO if exist %v\*.* rmdir /s/q %v

pack:
	$(MAKE) clean
	cd ..
	if exist $(PACKAGE) del $(PACKAGE)
	zip -r9 $(PACKAGE) isapi
	cd isapi	
	dir ..\$(PACKAGE)

start:
	net start $(WWWSERVICENAME)

stop:
	-net stop $(WWWSERVICENAME)
	net stop IISAdmin


test:
	cd .\Debug
		-del *.log
		-del $(LOGFILEPATH)\RECV.log
		-del $(LOGFILEPATH)\SENT.log
		-del $(LOGFILEPATH)\TEST.log
		dimeclnt test.jpg http://localhost/gsoap/mod_gsoap.dll?dime test.jpg
		calcclnt add 3 4
		echo start notepad %SYSTEMROOT%\system32\inetsrv\RECV.log
	cd ..


