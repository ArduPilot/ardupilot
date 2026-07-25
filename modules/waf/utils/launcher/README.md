#Waf-launcher
This is a simple wrapper for the 
[waf build system](https://waf.io/)

Since many windows users does not have python installed by default, 
the exe file from this project can be included along with the copy of 
waf to wrap waf to install python on demand.

The requirements is only .Net 2.0 whics is either Windows Server 2003 R2 
and up, or Windows Vista and up. There is a good chance it is installed 
in many OEM installs of Windows XP as well

##How to compile
use either waf or monodevelop (wscript and project files is in the repository)

##How to use

Assume we have a repository, myrepo, where wscript and waf already exists.

    myrepo/waf
    myrepo/wscript

now copy waf-launcher/bin/Release/waf.exe to the repository

    myrepo/waf.exe

Linux users can continue to use `./waf`, just like
Windows users can continue to use `python waf`

For Windows users there is now a second way to invoke waf by writing 
either `waf.exe` or simply just `waf`.

When the Windows users does that the following happens:

1. It tries "python waf"
2. If 1 failed it tries "C:\python27\python.exe waf"
3. If 2 failed it ask for permission to install python
4. If pemission is given it tries to install python silently
5. step 1 and 2 it done again
6. If this still fails we exit with an error

Any arguments to `waf.exe` is passed on to `waf`
