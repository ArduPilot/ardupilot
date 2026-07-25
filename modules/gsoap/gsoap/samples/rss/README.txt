
RSS Support
===========

The gSOAP RSS library supports RSS 0.91, 0.92, and 2.0

Examples
--------

An example RSS client and server are included. As a command-line client
displays RSS feeds in text format. As a server reformats formats RSS feeds in
HTML to be displayed in your web pages via a JavaScript (see below).

Usage (client)
--------------

> rss [maxitems] URL
Prints RSS content in text format.

Examples:

> rss 10 http://newsrss.bbc.co.uk/rss/newsonline_world_edition/front_page/rss.xml
> rss 10 http:// < your_rss_file.xml

Usage (server)
--------------

Installed as CGI application prints Javascript code to view RSS feeds in Web
pages. The Javascript code produced by this application is executed with the
following example script embedded in the Web page, e.g. to display BBC news:

<script src="http://www.cs.fsu.edu/~engelen/rss.cgi/?rss=http%3A%2F%2Fnewsrss.bbc.co.uk%2Frss%2Fnewsonline_world_edition%2Ffront_page%2Frss.xml&max=10"></script>
<noscript><a href="http://newsrss.bbc.co.uk/rss/newsonline_world_edition/front_page/rss.xml">BBC News</a></noscript>

To control the appearance with cascading style sheets:

rss_box		the bounding div for the entire display 
rss_table	the table with title, image, and items
rss_title	the title of the feed and link style if displayed
rss_image	the image
rss_bar		the dividing bar
rss_item	the title of the item
rss_desc	the description of the item

Enabling HTTPS
--------------

To enable HTTPS you should install OpenSSL, then compile with:

	$ cc -DWITH_OPENSSL -o rss rss.c stdsoap2.c soapC.c -lssl -lcrypto

