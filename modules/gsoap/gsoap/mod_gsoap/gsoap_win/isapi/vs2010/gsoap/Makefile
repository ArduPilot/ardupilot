DELFILES= *.ncb *.opt *.plg *.clw *.pch *.aps *.pdb *.dll
DELDIRS= .\Debug .\Release .\Win32

all:
	MsBuild isapi.vcxproj

clean:
	for %v in ($(DELDIRS)) DO if exist %v\*.* rmdir /s/q %v
	for %v in ($(DELFILES)) DO if exist %v del %v


