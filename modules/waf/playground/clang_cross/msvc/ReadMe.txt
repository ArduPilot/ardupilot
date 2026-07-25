To cross compile for Windows in MSVC mode from Linux, you will require the following:
* A partition with Windows installed (NTFS).
* Visual Studio (Tested with 2017).
* The Windows SDK.
* lowntfs-3g file system driver.

Make sure the Windows partition is mounted with "-t lowntfs-3g -o defaults,ignore_case,windows_names".
This will allow Clang to find all headers and libraries referenced by scripts and headers, otherwise you will run into case sensitivity errors.

Clang uses the following environment variables to detect the Visual Studio install: VCINSTALLDIR, VCToolsInstallDir, INCLUDE, LIB, LIBPATH
I just copied these from the output of the "set" command in an MSVC command prompt on Windows and translated the paths to Linux paths.
Notice how the semicolon is still used as a path separator.
See "example_environment_linux.sh" for how my setup looks like.
It expects the Windows partition to be mounted on /mnt/windows, with VS2017 installed and Windows 10 SDK 10.0.17763.0.
