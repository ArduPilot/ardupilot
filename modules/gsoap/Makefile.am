####### This is the input file for automake, which will generate Makefile.in ##########

SUBDIRS = . gsoap
CLEANFILES = *~
VERYVERYCLEANFILES = *~ configure Makefile.in compile config.guess config.sub config.log autoscan.log \
depcomp install-sh missing mkinstalldirs stamp-h.in config.h.in \
aclocal.m4 *.cache Makefile

## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.4

##
## Add support for pkg-config
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = gsoap.pc gsoap++.pc gsoapck.pc gsoapck++.pc gsoapssl.pc gsoapssl++.pc

EXTRA_DIST = gsoap.pc.in gsoap++.pc.in gsoapck.pc.in gsoapck++.pc.in gsoapssl.pc.in gsoapssl++.pc.in

.PHONY: backup

install-data-hook:
	echo "+--------------------------------------------------------+"; \
	echo "| You now have successfully built and installed gsoap.   |"; \
	echo "|                                                        |"; \
	echo "| You can link your programs with -lgsoap++ for          |"; \
	echo "| C++ projects created with soapcpp2 and you can link    |"; \
	echo "| with -lgsoap for C projects generated with soapcpp2 -c |"; \
	echo "|                                                        |"; \
	echo "| There are also corresponding libraries for SSL and     |"; \
	echo "| zlib compression support (-lgsoapssl and lgsoapssl++)  |"; \
	echo "| which require linking -lssl -lcrypto -lz               |"; \
	echo "|                                                        |"; \
	echo "| Thanks for using gsoap.                                |"; \
	echo "|                                                        |"; \
	echo "|               http://sourceforge.net/projects/gsoap2   |"; \
	echo "+--------------------------------------------------------+"; 

## remove also the links for files like install.sh etc. 
## use only for development of build script and makefiles
reallyveryveryclean:
	$(MAKE) distclean
	for file in $(VERYVERYCLEANFILES);do rm -rf $(VERYVERYCLEANFILES);done;

backup:
	$(MAKE) clean
	$(MAKE) distclean
	CURRENTDIR=`pwd` && \
	BACKUPDIR=`basename $$CURRENTDIR` && \
	cd .. && \
	tar -czhf $$BACKUPDIR.tgz $$BACKUPDIR && \
	cd $$BACKUPDIR && \
	ls -l $$CURRENTDIR.tgz

