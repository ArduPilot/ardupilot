% PNET    - IO-function for TCP and UDP comunation in matlab.
%
% 
%     This function can be called with different options/commands to operate on
%     tcp/udp/ip connection in matlab. It supports data transfer with
%     different data types and byte orders. The function is implemented as an
%     MEX-file that needs to be 
%
% General syntax
% ==============
%
%        pnet('command',....)          or         pnet command  ....
%   
%        ppet(con,'command',....)      or         pnet con command ....
%  
%        ...where 'command' is a string specifying what to do. "con" is an 
%        connection handler holding a number that refers to an already open
%        connection/socket. In the cases where "con" is specified before
%        'command' is the commands associated to operate on that connection/
%        socket. Connection handler number (and some other numbers like size
%        values) can be specified as either a scalar value or a string
%        containing the scalar number.
%  
% Commands for TCP connections
% ============================
%  
%  con=pnet('tcpconnect','hostname',port)
%  
%     Creates tcp/ip connection to the specified 'hostname' and port. On error
%     it returns -1, on success it returns a conection handler as an integer
%     number >=0. This function call is used to act as a "tcp-client".
%     This function is alway nonblocking unless their is some troubles with
%     namelookup that can block interpretation for a while.
%  
%  sockcon=pnet('tcpsocket',port)
%  
%     Creates a local tcpsocket and bind port-number "port" to it. On fail
%     it rerurns -1, on success it returns a connection handler. To fetch
%     remote connection connecting to this socket use pnet(sockcon,'tcplisten')
%     
%  con=pnet(sockcon,'tcplisten', ['noblock'] )
%     
%     If some remote tcp-connection has connected to the socket "sockcon" this
%     functions returns a handler to that tcp connection. On non available
%     remote connection or error it returns -1. Unless the option 'noblock' is
%     specified or the readtimeout value (default to inf ) is reached it blocks
%     until a remote connection is established.
%     
%  elements=pnet(con,'write', data [,swapping])
%     
%     Writes all elements of the matlab array "data" to connection "con". 
%     Supported datatypes are: char, double, single, int8 int16, int32,
%     uint8, uint16  and uint32. Arrays of datatype char is written as single
%     bytes, all other datatypes are written as it matlabsize. By default 
%     the bytes are swapped to network byte order unless the swapping option
%     is specified. Swapping options:
%     'native'  no swapping use the computers native byte order.
%     'swap'    allways swap byte order.
%     'network' uses network byte order (little endian) and it is default.
%     'intel'   use intel CPUs byte order (big endian).
%     This operation blocks until writetimeout (is by default inf) is reached
%     or all elements are transfered. Returns number of elements successfull
%     written.     
%     This command can also be used with UDP packets, se the UDP section.
%
%  pnet(con,'printf', 'format',.....)
%     
%     Prints a formated string to the connection "con". SPRINTF is used to
%     process format string and following arguments. Se help on SPRINTF for more
%     information. It block until writetimeout and returns number of characters
%     succesfull written.
%     This command can also be used with UDP packets, se the UDP section.
%     
%  data=pnet(con,'read' [,size] [,datatype] [,swapping] [,'view'] [,'noblock'])
%     
%     Reads an array of elements from a connection. Unless option 'noblock' is
%     used the operation blocks until timeout is reached (by default inf), the
%     specified "size" is reached or the connection is disconnected by peer.
%     Specified "size" (by default 65536) can be a scalar or a vector of
%     specifying dimentions length for the multidimentional array to read.
%     If size is a scalar i.e. [10] it return 0 to 10 elements in a rowvector
%     depening on available elements. If size is a vector i.e. [1 10] or [5 5 5]
%     it returns an array if all elements is available, if not, an empty array
%     is returned. The datatype option is a string containing the name of the
%     datatype, if not specified its by default 'char'. Supported datatypes is
%     the same as for the 'write' command.  The option named "swapping" is
%     is the same for the 'read' command. The option 'view' gives an "preview"
%     of whats available, all data is left in the read buffer. In combination
%     with 'noblock' this is a powerfull command to check how much and whats
%     available at the moment without blocking.
%     This command can also be used with UDP packets, se the UDP section.
%     
%  str=pnet(con,'readline' [,limitsize] [,'view'] [,'noblock'])
%     
%     This commands works like 'read' but it reads a string of characters
%     until the newline character (code=10) are reached.
%     It returns the string without the newline character (or if exist just
%     before, the carage return character code=13 ). If the character line
%     is longer then the "limitsize" the line is splited at that length.
%     The default value for limitsize is 65536.
%     The function blocks unless a full line of characters are available or
%     'noblock' is specified until readtimeout is reached. The 'view'
%     option also leaves the returned characters in the read buffer.
%     This command can also be used with UDP packets, se the UDP section.
%     
%  ret=pnet(con,'readtofile','filename'[,bytes][,'view'][,'noblock'][,'append'])
%     
%     Reads data of specified number of "bytes" to a file 'filename'. It
%     returns successful number of bytes moved from the connection to the file.
%     If the option string 'append' is specified the data will be appended
%     th the end of the file, otherwise the file will be overwritten.
%     'view', 'noblock' and timeout behaves as earlier described for read
%     operations.
%     This command can also be used with UDP packets, se the UDP section.
%     
%  ret=pnet(con,'writefromfile','filename'[[,start] ,len])
%
%     Read data from a file and write it on the connection. All of the
%     file will be transmitted of not "start" and "len" specifies a segment.
%     The function returns -1 on fail.
%     This command can also be used with UDP packets, se the UDP section.
%     
%  stat=pnet(con,'status')
%     
%     This returns a number telling about the status of a connection/socket.
%     It is a different number for each type of connection or socket se
%     #define's in top of pnet.c. The most important status is when
%     peer has disconnect the status value is 0 else it is >0.
%     This command can also be used with UDP packets, se the UDP section.
%
%  stat=pnet(con,'setreadtimeout',sec)
%     
%     The value "sec" specifies how long read and listen commands blocks before
%     it timeouts. The unit is seconds specified as a floting point which means
%     that i.e. 0.1 can be specified for a maximum of 0.1 seconds blocking.
%     Setting readtimeout to 0 is the same as adding the 'noblock' option to
%     all followinging 'read' or 'listen' calls. 
%     This command can also be used with UDP packets, se the UDP section.
%
%  stat=pnet(con,'setwritetimeout',sec)
%     
%     The value "sec" specifies how long the 'write' commands blocks before
%     it timeouts. The unit is seconds specified as a floting point which means
%     that i.e. 0.1 can be specified for a maximum of 0.1 seconds blocking.
%     This command can also be used with UDP packets, se the UDP section.
%     
%  [ip,port]=pnet(con,'gethost')
%     
%     This command call returns ipnumber and port number for the remote host
%     that is (latest) associated with the connection/socket. After 'tcplisten'
%     you can get the the clients ip and port numbers. After a 'tcpconnect'
%     you can get the ip and port that you connected to. 
%     For UDP sockets this gives remote ip and and port for the latest operation
%     of 'readpacket', 'writepacket' (if not connected) or 'udpconnect'.
%
%  pnet(con,'close')
%
%     Closes a tcpconnection, tcpsocket or udpsocket. This comand must be called
%     after ending use of any socket or connection even, if its detected that the
%     connection is broken from the remote host side. 
%     This command should also be used with UDP sockets, se the UDP section.
%
%  pnet('closeall')
%
%     Closes all pnet connections/sockets used in this matlab session.
%
% UDP packets
% ===========
%  
%         With PNET it's possible send and receive UDP packets. Same read/write
%         commands can be used as for with TCP connection with the difference
%         that the operation is alway nonblocking and data is stored in the 
%         read/write buffer in memory. With  pnet(sock,'writepacket'...) can the
%         created UDP packet in the write buffer be sent. And with 
%         pnet(sock,'readpacket') a new packet can be recived before reading its
%         contents with read commands. The limitation is how big UDP packets
%         your network can transmitt. But about 65500 bytes is maximum.
%     
%  sock=pnet('udpsocket',port);
%     
%     Creates a UDP socket and binds it to an UDP port. On this socket can you
%     recive UDP packets destinated to this UDP port, and send UDP packets with
%     this sockat as source adress. Retruns -1 on fail or a handler >=0 on
%     success. In this sockets write buffer you can create a UDP packet that
%     is later sent with the 'writepacket' command. 
%     
%  pnet(sock,'udpconnect','hostname',port);
%     
%     With this command you can connect a destination host and port to the the
%     UDP socket causing that the 'writepacket' commands should not have any
%     'hostname' and port arguments supplied.
%     
%  pnet(sock,'writepacket' [,'hostname',port]);
%     
%     Sends contents of the sockets write buffer as a UDP packet. If the UDP
%     socket is not connected 'hostname' and port must be supplied, if connected
%     the arguments is ignored.
%     
%  size=pnet(sock,'readpacket'[,maxsize][,'noblock']);
%     
%     Reads next incoming UDP packet on UDP socket "sock".  Unless 'noblock'
%     is supplied as option it block until readtimeout or next UDP packet is
%     received. The optional argument "maxsize" limits the size of the packet.
%     The packet is stored in the sockets read buffer and can then be readed
%     from the buffer with same commands as for TCP connections. When reciving
%     a new packet old non used data from the last packet is discarded.
%
%  General alternative syntax
%  ==========================
%
%       pnet 1 write Hello_World
%     
%    is the same as    
%     
%       pnet(1,'write','Hello_World')
%     
%     Numbers like connection handlers and port-numbers can be specified
%     as strings in most cases. This syntax should generaly work for all
%     variants of calls and is usefull and easy to type when experimenting
%     interactively with tcp connection and udp packets from the matlab
%     prompt.
%
%
%   SE ALSO: PNET_REMOTE, PNET_PUTVAR PNET_GETVAR, WEBGET_DEMO, POPMAIL_DEMO
%            UDP_PLOTTER_DEMO, UDP_SEND_DEMO, WEBSERVER_DEMO
%
%
%   GOOD LUCK and HAVE FUN!  :-)
%
%       Peter Rydesäter, 
%       Mitthögskolan(Mid Sweden University) campus Östersund, SWEDEN
%

error('You need to compile pnet.c to a mex-file for your platform. Read the header of pnet.c');