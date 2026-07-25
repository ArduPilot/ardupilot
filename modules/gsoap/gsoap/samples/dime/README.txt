Example DIME attachment processing.

Note: DIME attachments are deprecated in favor of MTOM attachments.

This demo illustrates streaming DIME attachments. DIME is an efficient binary
attachment format. DIME has been deprecated by the DIME authors in favor of
MTOM MIME-based attachments.

Client and server examples are included. Both use streaming DIME attachment
processing using data handlers.

The service implements three streaming operations:

putData stores multiple data sets on the server and returns named references to
        each data set
getData retrieves data sets given named references.
getImage is an example file-based image retrieval method

Usage (server side):

For stand-alone Web service functionality, run from the command line
with port number as command line argument:

dimeserver 8085 &

Usage (client side):

dimeclient
dimeclient name
dimeclient [-p] [-g] name ...

dimeclient
        Without args retrieves image.jpg
dimeclient name
        Retrieves image stored under name
dimeclient -p name1 name2 ...
        Stores files name1, name2, etc. The storage keys are printed.
        The keys provide access to the data on the server.
dimeclient -g name1 name2 ...
        Retrieves files stored under keys name1, name2, etc.
        The keys must correspond to the keys returned when storing
        files. Files are stored locally under the key name.

All files are stored and retrieved in the current directory.
