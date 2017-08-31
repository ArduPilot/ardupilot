goto %TOOLSET%

:cygwin

C:\cygwin64\setup-x86_64.exe -qnNdO -R C:\cygwin64 -s http://cygwin.mirror.constant.com -l C:\cygwin64\var\cache\setup -P gcc-g++,gcc-core,binutils,libxml2,libxml2-devel,libxslt-devel,python2,python2-devel,python2-pip,git,ccache
path C:\cygwin64\bin;%path%
set CHERE_INVOKING=yes
bash -lc "easy_install-2.7 empy pyserial future lxml"
bash -lc "git submodule sync --recursive"
bash -lc "git submodule foreach --recursive 'git fetch --tags'"
bash -lc "git submodule update --init --recursive"
bash -lc "./waf configure --board=sitl && ./waf clean && ./waf bin"

goto :eof
