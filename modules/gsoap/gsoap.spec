# $Id: gsoap-commercial.spec 878 2013-01-11 15:11:41Z bs827200 $

Summary: Generator Tools for Coding SOAP/XML Web Services in C and C++
Name: gsoap-commercial
Version: 2.8.114
Release: fxs.46.2.%{?dist}
# Bump the soversion when the new version is not backward compatible
%global soversion 2

License: Open source under GPLv2, commercial use requires commercial-use license
Group: Development/Tools
URL: http://www.genivia.com/
#Source0: http://downloads.sourceforge.net/gsoap2/%{name}_%{version}.zip
Source0: %{name}-%{version}.tar.bz2
#Source1: soapcpp2.1
#Source2: wsdl2h.1
Patch0: %{name}-libtool.patch
BuildRoot: %(mktemp -ud %{_tmppath}/%{name}-%{version}-%{release}-XXXXXX)

BuildRequires: flex
BuildRequires: bison
BuildRequires: findutils
BuildRequires: dos2unix
BuildRequires: openssl-devel
BuildRequires: autoconf
BuildRequires: automake
BuildRequires: libtool

Requires: openssl
Obsoletes: %{name}-examples < 2.8.11

%description
The gSOAP Web services development toolkit offers an XML to C/C++
language binding to ease the development of SOAP/XML Web services in C
and C/C++.

%package devel
Summary: Devel libraries and headers for linking with gSOAP generated stubs
Group: Development/System
Requires: %name = %version-%release
Requires: pkgconfig

%description devel
gSOAP libraries, headers and generators for linking with and creating
gSOAP generated stubs

%package doc
Summary: Documentation for gSOAP
Group: Development/System
Requires: %name = %version-%release
#BuildArch: noarch

%description doc
gSOAP documentation in html

%package debug
Summary: Debug binaries and code for gSOAP
Group: Development/System
Requires: %name = %version-%release

%description debug
Debug binaries and source code for gSOAP (*.debug)


%prep
%setup -q -n %name-%version

# enable use of libtool in configure.in and a few Makefile.am files
%patch0 -p1

# XML files non-executable
find . -name "*.xml" -exec chmod a-x {} \;

# we want all txt files to have unix end-of-line encoding
dos2unix README.txt LICENSE.txt NOTES.txt

# remove .DS_Stores
rm gsoap/doc/.DS_Store

# remove stuff with gsoap license only - not GPL
rm -rf gsoap/extras gsoap/mod_gsoap gsoap/Symbian
sed 's!$(srcdir)/extras/\*!!' -i gsoap/Makefile.am

echo "LDFLAGS = -lm" >> gsoap/samples/chaining/Makefile.am
#sed '' -i gsoap/samples/chaining/Makefile.in

%build
# patches change autoconf and automake files, so we must reconfigure
autoreconf --install --force

%configure --disable-static --prefix=/usr --enable-ipv6 --enable-samples
#%configure --disable-static --prefix=/usr --enable-ipv6 --enable-samples --disable-debug

# dependencies are not declared properly
#make %{?_smp_mflags}
make SOVERSION=%{soversion}

%install
rm -rf %{buildroot}
make install SOVERSION=%{soversion} DESTDIR=%{buildroot}
rm -f %{buildroot}/%_libdir/*.la

mkdir -p %{buildroot}/%_mandir/man1
#install -m 644 -p %{SOURCE1} %{SOURCE2} %{buildroot}/%_mandir/man1

%clean
rm -rf %{buildroot}

%check
make check

# NOTE: %{name}-libtool.patch prevents the build of .la and .a archives
%files
%defattr(-,root,root,-)
%doc README.txt NOTES.txt LICENSE.txt
%_libdir/libgsoap.so*
#%_libdir/libgsoap.a
%_libdir/libgsoap++.so*
#%_libdir/libgsoap++.a
%_libdir/libgsoapck.so*
#%_libdir/libgsoapck.a
%_libdir/libgsoapck++.so*
#%_libdir/libgsoapck++.a
%_libdir/libgsoapssl.so*
#%_libdir/libgsoapssl.a
%_libdir/libgsoapssl++.so*
#%_libdir/libgsoapssl++.a

%files devel
%defattr(-,root,root,-)
%doc README.txt NOTES.txt LICENSE.txt
%_bindir/soapcpp2
%_bindir/wsdl2h
#%_mandir/man1/soapcpp2.1*
#%_mandir/man1/wsdl2h.1*
%_libdir/libgsoap.so*
#%_libdir/libgsoap.a
%_libdir/libgsoap++.so*
#%_libdir/libgsoap++.a
%_libdir/libgsoapck.so*
#%_libdir/libgsoapck.a
%_libdir/libgsoapck++.so*
#%_libdir/libgsoapck++.a
%_libdir/libgsoapssl.so*
#%_libdir/libgsoapssl.a
%_libdir/libgsoapssl++.so*
#%_libdir/libgsoapssl++.a
%_includedir/stdsoap2.h
%dir %_datadir/gsoap
%dir %_datadir/gsoap/import
%_datadir/gsoap/import/c14n.h
%_datadir/gsoap/import/dom.h
%_datadir/gsoap/import/ds2.h
%_datadir/gsoap/import/ds.h
%_datadir/gsoap/import/README.txt
%_datadir/gsoap/import/soap12.h
%_datadir/gsoap/import/stldeque.h
%_datadir/gsoap/import/stl.h
%_datadir/gsoap/import/stllist.h
%_datadir/gsoap/import/stlset.h
%_datadir/gsoap/import/stlvector.h
%_datadir/gsoap/import/wsa3.h
%_datadir/gsoap/import/wsa4.h
%_datadir/gsoap/import/wsa5.h
%_datadir/gsoap/import/wsa.h
%_datadir/gsoap/import/WS-example.c
%_datadir/gsoap/import/WS-example.h
%_datadir/gsoap/import/WS-Header.h
%_datadir/gsoap/import/wsp.h
%_datadir/gsoap/import/wsrp.h
%_datadir/gsoap/import/wsse2.h
%_datadir/gsoap/import/wsse.h
%_datadir/gsoap/import/wsu.h
%_datadir/gsoap/import/xlink.h
%_datadir/gsoap/import/xmime4.h
%_datadir/gsoap/import/xmime5.h
%_datadir/gsoap/import/xmime.h
%_datadir/gsoap/import/xml.h
%_datadir/gsoap/import/xmlmime5.h
%_datadir/gsoap/import/xmlmime.h
%_datadir/gsoap/import/xop.h
%dir %_datadir/gsoap/WS
%_datadir/gsoap/WS/README.txt
%_datadir/gsoap/WS/WS-Addressing.xsd
%_datadir/gsoap/WS/WS-Addressing03.xsd
%_datadir/gsoap/WS/WS-Addressing04.xsd
%_datadir/gsoap/WS/WS-Addressing05.xsd
%_datadir/gsoap/WS/WS-Discovery.wsdl
%_datadir/gsoap/WS/WS-Enumeration.wsdl
%_datadir/gsoap/WS/WS-Policy.xsd
%_datadir/gsoap/WS/WS-Routing.xsd
%_datadir/gsoap/WS/WS-typemap.dat
%_datadir/gsoap/WS/discovery.xsd
%_datadir/gsoap/WS/ds.xsd
%_datadir/gsoap/WS/enumeration.xsd
%_datadir/gsoap/WS/typemap.dat
%_datadir/gsoap/WS/wsse.xsd
%_datadir/gsoap/WS/wsu.xsd
%dir %_datadir/gsoap/custom
%_datadir/gsoap/custom/README.txt
%_datadir/gsoap/custom/long_double.c
%_datadir/gsoap/custom/long_double.h
%_datadir/gsoap/custom/struct_timeval.c
%_datadir/gsoap/custom/struct_timeval.h
%_datadir/gsoap/custom/struct_tm.c
%_datadir/gsoap/custom/struct_tm.h
%dir %_datadir/gsoap/plugin
%_datadir/gsoap/plugin/README.txt
%_datadir/gsoap/plugin/cacerts.c
%_datadir/gsoap/plugin/cacerts.h
%_datadir/gsoap/plugin/httpda.c
%_datadir/gsoap/plugin/httpda.h
%_datadir/gsoap/plugin/httpdatest.c
%_datadir/gsoap/plugin/httpdatest.h
%_datadir/gsoap/plugin/httpform.c
%_datadir/gsoap/plugin/httpform.h
%_datadir/gsoap/plugin/httpget.c
%_datadir/gsoap/plugin/httpget.h
%_datadir/gsoap/plugin/httpgettest.c
%_datadir/gsoap/plugin/httpgettest.h
%_datadir/gsoap/plugin/httpmd5.c
%_datadir/gsoap/plugin/httpmd5.h
%_datadir/gsoap/plugin/httpmd5test.c
%_datadir/gsoap/plugin/httpmd5test.h
%_datadir/gsoap/plugin/httppost.c
%_datadir/gsoap/plugin/httppost.h
%_datadir/gsoap/plugin/logging.c
%_datadir/gsoap/plugin/logging.h
%_datadir/gsoap/plugin/md5evp.c
%_datadir/gsoap/plugin/md5evp.h
%_datadir/gsoap/plugin/plugin.c
%_datadir/gsoap/plugin/plugin.h
%_datadir/gsoap/plugin/smdevp.c
%_datadir/gsoap/plugin/smdevp.h
%_datadir/gsoap/plugin/threads.c
%_datadir/gsoap/plugin/threads.h
%_datadir/gsoap/plugin/wsaapi.c
%_datadir/gsoap/plugin/wsaapi.h
%_datadir/gsoap/plugin/wsse2api.c
%_datadir/gsoap/plugin/wsse2api.h
%_datadir/gsoap/plugin/wsseapi.c
%_datadir/gsoap/plugin/wsseapi.h
%_libdir/pkgconfig/gsoapck.pc
%_libdir/pkgconfig/gsoapck++.pc
%_libdir/pkgconfig/gsoap.pc
%_libdir/pkgconfig/gsoap++.pc
%_libdir/pkgconfig/gsoapssl.pc
%_libdir/pkgconfig/gsoapssl++.pc
# Additions in 2.7.12-1
%_datadir/gsoap/WS/WS-ReliableMessaging.wsdl
%_datadir/gsoap/WS/WS-ReliableMessaging.xsd
%_datadir/gsoap/WS/reference-1.1.xsd
%_datadir/gsoap/WS/ws-reliability-1.1.xsd
%_datadir/gsoap/import/ref.h
%_datadir/gsoap/import/wsrm.h
%_datadir/gsoap/import/wsrm4.h
%_datadir/gsoap/import/wsrx.h
# Additions in 2.7.13-1
%_datadir/gsoap/import/stdstring.h
%_datadir/gsoap/import/xsd.h
%_datadir/gsoap/plugin/wsseapi.cpp
# Additions in 2.7.16-1
%_datadir/gsoap/custom/duration.c
%_datadir/gsoap/custom/duration.h
%_datadir/gsoap/plugin/httpposttest.c
%_datadir/gsoap/plugin/httpposttest.h
%_datadir/gsoap/plugin/wsrmapi.c
%_datadir/gsoap/plugin/wsrmapi.h
# Additions in 2.7.17-1
%_datadir/gsoap/WS/WS-Policy12.xsd
%_datadir/gsoap/WS/WS-SecurityPolicy.xsd
%_datadir/gsoap/import/wsse11.h
# Additions in 2.8.3-1
%_datadir/gsoap/WS/xenc.xsd
%_datadir/gsoap/import/xenc.h
%_datadir/gsoap/plugin/mecevp.c
%_datadir/gsoap/plugin/mecevp.h
# Additions in 2.8.4-1
%_datadir/gsoap/import/wsdd.h
%_datadir/gsoap/import/wsdx.h
%_datadir/gsoap/plugin/wsddapi.c
%_datadir/gsoap/plugin/wsddapi.h
# Additions in 2.8.7-1
%_datadir/gsoap/import/wsdd10.h
# Additions in 2.8.12-1
%_datadir/gsoap/import/ser.h
%_datadir/gsoap/import/wsc.h
%_datadir/gsoap/import/wsrm5.h
%_datadir/gsoap/import/wsrx5.h
%_datadir/gsoap/import/wst.h
%_datadir/gsoap/import/wstx.h
%_datadir/gsoap/WS/WS-Trust.wsdl
%_datadir/gsoap/WS/WS-Trust.xsd
%_datadir/gsoap/WS/WS-SecureConversation.xsd
# Additions in 2.8.16-1
%_datadir/gsoap/import/wsc2.h
%_datadir/gsoap/plugin/calcrest.h

%files doc
%defattr(-,root,root,-)
%doc gsoap/doc/*
%doc README.txt NOTES.txt LICENSE.txt

# We can't use %libdir here because it subs /usr/lib64 and this stuff wants to explicitly go into /usr/lib
%files debug
%defattr(-,root,root,-)
#%_libdir/debug/usr/lib64/libgsoap*
#/usr/lib/debug/usr/lib64/libgsoap*
/usr/lib/debug/usr/bin/soapcpp2.debug
/usr/lib/debug/usr/bin/wsdl2h.debug
/usr/src/debug/%name-%version

%post -p /sbin/ldconfig

%postun -p /sbin/ldconfig

%changelog
* Fri Oct 06 2013 Robert van Engelen <engelen@genivia.com> - 2.8.17
- Updated for 2.8.17 release

* Fri Jan 11 2013 Brian A. Seklecki <bseklecki@fedex.com> - 2.8.12.fxs.46.2
- FedEx ITSS Snapshot Import (Binary Release of 2.8.12 from Genivia)

* Tue Oct 09 2012 Brian A. Seklecki <bseklecki@fedex.com> - 2.8.7.2.46.2
- FedEx ITSS Snapshot Import 

* Tue Feb 28 2012 Fedora Release Engineering <rel-eng@lists.fedoraproject.org> - 2.8.7-2
- Rebuilt for c++ ABI breakage

* Wed Feb 08 2012 Mattias Ellert <mattias.ellert@fysast.uu.se> - 2.8.7-1
- Update to gsoap 2.8.7
- Bump soname due to changes in struct soap w.r.t. previous Fedora version

* Fri Jan 13 2012 Fedora Release Engineering <rel-eng@lists.fedoraproject.org> - 2.8.4-4
- Rebuilt for https://fedoraproject.org/wiki/Fedora_17_Mass_Rebuild

* Mon Nov 07 2011 Mattias Ellert <mattias.ellert@fysast.uu.se> - 2.8.4-3
- Bump soname

* Tue Nov 01 2011 Mattias Ellert <mattias.ellert@fysast.uu.se> - 2.8.4-2
- Move openssl libraries to Libs.private in pkg-config files

* Mon Oct 31 2011 Mattias Ellert <mattias.ellert@fysast.uu.se> - 2.8.4-1
- Update to gsoap 2.8.4
- Drop gsoap-ipv6.patch implemented upstream
- Link gsoap SSL shared libraries with libssl

* Fri Oct 21 2011 Mattias Ellert <mattias.ellert@fysast.uu.se> - 2.8.3-2
- Fix an issue with IPv4 only sockets when IPv6 support is enabled

* Tue Aug 23 2011 Mattias Ellert <mattias.ellert@fysast.uu.se> - 2.8.3-1
- Update to gsoap 2.8.3
- Drop the examples sub-package - the examples are written to be built
  in the source tree, installing them does not make sense without a
  major rewrite
- Add manpages from Debian

* Sat Feb 12 2011 Mattias Ellert <mattias.ellert@fysast.uu.se> - 2.7.17-3
- Enable IPv6 support

* Wed Feb 09 2011 Fedora Release Engineering <rel-eng@lists.fedoraproject.org> - 2.7.17-2
- Rebuilt for https://fedoraproject.org/wiki/Fedora_15_Mass_Rebuild

* Tue Jul 28 2010 Ivan Romanov <drizt@land.ru> - 2.7.17-1
- Update to gsoap 2.7.17
- Removed openssl patch, it fixed by upstream
- Added gsoap-2.7-iostream.patch to fix using iostream oldstyle in examples
- Added examples sources and binaries
- Added documentation

* Sat Apr 24 2010  <matt@redhat> - 2.7.16-1
- Update to gsoap 2.7.16
- Thanks to matt@mattjamison.com for update patches
- Removed unused_args patch (upstream)

* Fri Sep 18 2009 Lubomir Rintel <lkundrak@v3.sk> - 2.7.13-2
- Fix build

* Mon May 11 2009  <matt@redhat> - 2.7.13-1
- Updated to gsoap 2.7.13

* Tue Feb 24 2009 Fedora Release Engineering <rel-eng@lists.fedoraproject.org> - 2.7.12-3
- Rebuilt for https://fedoraproject.org/wiki/Fedora_11_Mass_Rebuild

* Fri Jan 16 2009 Tomas Mraz <tmraz@redhat.com> - 2.7.12-2
- rebuild with new openssl

* Wed Dec 24 2008  <matt@redhat> - 2.7.12-1
- Updated to gsoap 2.7.12:
-  Numerous bug fixes - xml:lang, maxOccurs="unbounded", SSL, xmlns="", ...
-  New features, maintaining backward compatibility - MinGW, wsseapi, multi-endpoint connect, ...
- Patches removed (incorporated upstream):
-  datadir_importpath-2.7.10.patch
-  install_soapcpp2_wsdl2h_aux-2.7.10.patch
-  no_locale.patch as default off, enable by defining WITH_C_LOCALE
- Patches added (sent upstream):
-  unused_args.patch - eliminate many unused param warnings

* Thu Feb 21 2008  <mfarrellee@redhat> - 2.7.10-4
- Applied upd patch from upstream. It fixes glibc C locale issues,
  hp-ux h_errno definition, and xsd:dateTime timezone processing for
  WS-I
- Removed tru64_hp_ux patches, they are present in upstream's upd
  patch
- Added no_locale.patch to stop configure from checking for locale
  version of functions

* Tue Feb 19 2008 Fedora Release Engineering <rel-eng@fedoraproject.org> - 2.7.10-3
- Autorebuild for GCC 4.3

* Mon Feb 18 2008  <mfarrellee@redhat> - 2.7.10-2
- Removed --disable-namespaces from configure, result is code compiled
  against gsoap does not need to call set_soap_namespaces

* Sun Jan 27 2008  <mfarrellee@redhat> - 2.7.10-1
- Upgraded to 2.7.10 release
- Stopped hosting patches on grid.et.redhat.com
- Removed import_dom_h patch, it was integrated
- Removed large autotools patch, replaced with patch
  (use_libtool-2.7.10.patch) changing configure.in, gsoap/Makefile.am
  and gsoap/wsdl/Makefile.am, which enable libtool use, and a
  call to autoreconf
- Changed soapcpp2 references to gsoap as per new layout of source
  distribution
- Updated tru64_hp_up_c/pp patches to handle new source layout
- Install of soapcpp2/import with cp removed in favor of a patch to
  gsoap/Makefile.am (install_soapcpp2_wsdl2h_aux-2.7.10.patch)
- No pre-generated Makefiles are distributed, no longer removing them
- stdsoap2_cpp.cpp not in distribution, no longer removing it
- Added datadir_importpath-2.7.10.patch to set SOAPCPP2_IMPORT_PATH
  and WSDL2H_IMPORT_PATH, useful defaults, using ${datadir} instead of
  `pwd`
- Added autoconf, automake and libtool to BuildRequires, because
  configure.in and gsoap/Makefile.am are patched
- Added ?dist to Release

* Fri Nov 30 2007  <mfarrellee@redhat> - 2.7.9-0.4.l
- Added OpenSSL requirement

* Tue Nov 27 2007  <mfarrellee@redhat> - 2.7.9-0.3.l
- Decided soapcpp2/import/ files should be in /usr/share instead of
  /usr/include because they are not really headers gcc can
  process. Also, this is likely the location new versions of gsoap
  will install the import headers. People should use -I
  /usr/share/gsoap/import

* Mon Nov 26 2007  <mfarrellee@redhat> - 2.7.9-0.2.l
- Changed license tag to GPLv2+, which is more accurate
- Resolved bz399761 by adding soapcpp2/import/*.h to the -devel
  package as /usr/include/gsoap, users will need to add -I
  /usr/include/gsoap to their soapcpp2 line

* Sun Sep 30 2007  <mfarrellee@redhat> - 2.7.9-0.1.l
- Updated to 2.7.9l (2.7.9k patches still apply)
- Added patch for import/dom.h missing xsd__anyAttribute definitions
- Removed removal of .deps directories and autom4te.cache

* Mon Sep 24 2007  <mfarrellee@redhat> - 2.7.9-0.2.k
- Moved pkgconfig requirement to -devel package

* Tue Sep 11 2007  <mfarrellee@redhat> - 2.7.9-0.1.k
- Initial release

