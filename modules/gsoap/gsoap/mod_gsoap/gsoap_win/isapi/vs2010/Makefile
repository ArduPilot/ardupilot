DELFILES= *.ncb *.opt *.plg *.clw *.pch *.aps *.pdb *.dll *.db \
	RECV.log SENT.log TEST.log
DELDIRS= .\Debug .\Release

all:
	cd samples
		$(MAKE)
	cd ..
	cd gsoap
		$(MAKE)
	cd ..


clean:
	for %v in ($(DELFILES)) DO if exist %v del %v
	for %v in ($(DELDIRS)) DO if exist %v\*.* rmdir /s/q %v
	cd samples
		$(MAKE) clean
	cd ..
	cd gsoap
		$(MAKE) clean
	cd ..
	
start:
	net start W3SVC

stop:
	net stop WAS