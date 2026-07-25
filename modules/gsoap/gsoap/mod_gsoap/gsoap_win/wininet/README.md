
The WinInet plugin                                                   {#mainpage}
==================

[TOC]

By Jack Kustanowitz, Brodie Thiesfield, and Robert van Engelen.


Overview                                                             {#overview}
========

The WinInet plugin for gSOAP enables client applications (not servers) to
communicate through Microsoft's WinInet API on Windows. This offers all of the
advantages of WinInet-managed internet access through the `Internet Options`
control panel of Windows, such as HTTP (proxy) authentication, TLS/SSL, and
HTTP compression.  Therefore, "if IE works, gSOAP works." since these options
are shared by IE.

The WinInet project home is at <http://code.google.com/p/gsoapwininet>.

Features                                                             {#features}
========

- A plugin for gSOAP that is very easy to use.
- Complete support for HTTP features, including HTTPS, authentication (basic,
  digest, NTLM), proxy authentication
- Authentication warnings (e.g. invalid TLS/SSL certificate) can be resolved by
  the user via standard system dialog boxes.
- Timeouts for connect, receive, and send operations are obeyed when these are
  set BEFORE the plugin is registered with the engine.
- HTTP proxy settings are also obeyed when these are set BEFORE the plugin is
  registered with the engine.
- Supports all `SOAP_IO` modes of gSOAP (see [limitations](#limitations)).
- Can be used with C, C++, and MFC projects.
- Can be used in both MBCS and UNICODE projects.
- Compiles cleanly at warning level 4 supporting Win32 and Win64.
- Debug logging is supported (`TEST.log`, `SENT.log`, and `RECV.log`).
- MT safe and supports multiple threads, as usual with gSOAP clients.


Limitations                                                       {#limitations}
===========

- MIME and DIME attachments are not supported.
- The plugin may internally buffer the entire outgoing message before sending,
  even when the `SOAP_IO_CHUNK` mode is used.
- Because messages are buffered internally, do not use the `SOAP_IO_STORE`
  flag. Otherwise, the message may be buffered twice on every send.
- This plugin uses the following callback functions and is not compatible with
  any other plugin that uses these functions: `soap::fopen`, `soap::fposthdr`
  `soap::fsend`, `soap::frecv`, `soap::fclose`.


Using the WinInet plugin with gSOAP                                     {#usage}
===================================

Add the `gsoapWinInet2.h` and `gsoapWinInet2.cpp` files to your project. If you 
have a C project, change the extension to `.c`. Disable "precompiled headers"
for the `.cpp` file.

In your source code for the client, register the WinInet plugin with
`soap_register_plugin(soap, wininet_plugin)` after creation and initialization
of the `soap` context.

For example, when using a proxy object in C++ generated with soapcpp2 -j:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
     #include "gsoapWinInet.h"
     #include "soapProxy.h"
     Proxy proxy;
     proxy.soap->connect_timeout = 15; // 15 sec max connect time
     proxy.soap->recv_timeout = 10;    // 10 sec max recv time
     proxy.soap->send_timeout = 10;    // 10 sec max send time
     soap_register_plugin(proxy.soap, wininet_plugin);
     ...
     proxy.destroy(); // delete deserialized data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

and in plain C/C++, that is, without a proxy object:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
     #include "soapH.h"
     #include "gsoapWinInet.h"
     struct soap soap;
     soap_init(&soap);
     soap.connect_timeout = 15;  // 15 sec max connect time
     soap.recv_timeout = 10;     // 10 sec max recv time
     soap.send_timeout = 10;     // 10 sec max send time
     soap_register_plugin(&soap, wininet_plugin);
     ...
     soap_destroy(&soap); // delete deserialized data
     soap_end(&soap);     // delete temporary and C-based deserialized data
     soap_done(&soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that the receive and send timeouts limit the time to receive and send
data, respectively.  **This behavior differs from the gSOAP engine's timeouts
that limit the socket receive and send operation idle times.**  The gSOAP
engine uses `transfer_timeout` to limit the receive and send times.

To specify HTTP proxy settings, set the `soap.proxy_host` and `soap.proxy_port`
to the HTTP proxy host and port, respectively, and optionally set
`soap.proxy_userid` and `soap.proxy_passwd` to authenticate to the proxy.

Please make sure to compile all sources in C++ compilation mode. If you migrate
to a project file such as `.vcproj`, please set `CompileAs="2"` in your
`.vcproj` file.


WinInet plugin options                                                {#options}
======================

To control the WinInet's HttpOpenRequest options, register the WinInet plugin
with `soap_register_plugin_arg()` and supply an argument that is passed on to
HttpOpenRequest. For example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
     soap_register_plugin_arg(&soap, wininet_plugin, (void*)INTERNET_FLAG_IGNORE_CERT_CN_INVALID);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See the MSDN documentation on HttpOpenRequest for details of the
HttpOpenRequest flags. The `wininet.h` header must be included to use these
flags.

- `INTERNET_FLAG_KEEP_CONNECTION` use keep-alive semantics, if available, for
  the connection. This flag is required for Microsoft Network (MSN), NT LAN
  Manager (NTLM), and other types of authentication. This flag is set
  automatically when the soap context is initalized with `SOAP_IO_KEEPALIVE`.
- `INTERNET_FLAG_IGNORE_CERT_CN_INVALID` disables Microsoft Win32 Internet
  function checking of SSL/PCT- based certificates that are returned from the
  server against the host name given in the request. 
- `INTERNET_FLAG_IGNORE_CERT_DATE_INVALID` disables Win32 Internet function
  checking of SSL/PCT-based certificates for proper validity dates.

If there are errors in sending the HTTP request which would cause a dialog box
to be displayed in IE (for instance, invalid certificates on an HTTPS
connection), then a dialog will also be displayed by this library. At the
moment is is not possible to disable the UI. If you wish to remove the UI then
you will need to hack the source to remove the dialog box and resolve the
errors programmatically, or supply the appropriate flags to
`soap_register_plugin_arg()` to disable the unwanted warnings.


License                                                               {#license}
=======

MIT open source license.

This open source license is replaced by Genivia's license for commercial use
when a commercial-use license is purchased by customer.

The licence text below is the boilerplate "MIT Licence" used from:
http://www.opensource.org/licenses/mit-license.php

Copyright (c) 2009, Jack Kustanowitz, Brodie Thiesfield, Robert van Engelen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


Contributors                                                     {#contributors}
============

- 26 May 2003: Jack Kustanowitz (jackk@atomica.com):
  Original prototype version
- 29 September 2003: Brodie Thiesfield (code@jellycan.com):
  Rewritten as C plugin for gsoap. Bugs fixed and features added.
- 14 January 2004: Brodie Thiesfield (code@jellycan.com):
  Bug fix.
- 17 March 2009: Brodie Thiesfield (code@jellycan.com):
  Clean up and re-release.
- 8 October 2010: Robert van Engelen (engelen@genivia.com):
  Cleanup and bug fixes for error handling.
- 28 October 2015: Robert van Engelen (engelen@genivia.com):
  Plugin copy code added.

