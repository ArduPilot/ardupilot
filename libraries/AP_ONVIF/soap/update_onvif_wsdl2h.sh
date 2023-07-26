#!/bin/sh

cp ../../../modules/gsoap/gsoap/typemap.dat .
wsdl2h -O4 -P -s -x -o onvif.h \
  http://www.onvif.org/onvif/ver10/device/wsdl/devicemgmt.wsdl \
  http://www.onvif.org/onvif/ver10/media/wsdl/media.wsdl \
  http://www.onvif.org/onvif/ver20/ptz/wsdl/ptz.wsdl
#   http://www.onvif.org/onvif/ver10/events/wsdl/event.wsdl \
#   http://www.onvif.org/onvif/ver10/deviceio.wsdl \
#   http://www.onvif.org/onvif/ver20/imaging/wsdl/imaging.wsdl \

#   http://www.onvif.org/onvif/ver10/network/wsdl/remotediscovery.wsdl \
#   http://www.onvif.org/ver10/advancedsecurity/wsdl/advancedsecurity.wsdl
