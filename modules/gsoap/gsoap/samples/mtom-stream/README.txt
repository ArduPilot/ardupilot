MTOM attachment streaming processing example.

This application is a server and client to demonstrate MTOM attachment handling.

Usage (CGI server):

        Install as CGI application, e.g. under cgi-bin

Usage (stand-alone server):

$ mtom-stream-test 8085 &

        Starts a server on your host at port 8085.

Usage (client):

$ mtom-stream-test -p file1 file2 file3 ...

        Stores files file1, file2, etc. at the server side. The server
        saves them locally under a key. The storage keys are printed at
        the client side. The keys provide access to the data using
        option -g (get).

$ mtom-stream-test -g name1 name2 name3 ...

        Retrieves files stored under keys name1, name2, etc.
        The keys must correspond to the keys returned when storing
        files. Files are stored by the server locally under the key
        name.
