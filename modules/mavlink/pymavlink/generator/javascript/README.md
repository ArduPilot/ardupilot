## Javascript MAVLink implementation ##

As of Nov 2020, the '../javascript_stable/' variant is considered deprecated in favour of this 'nextgen' implementation found in this ./javascript/ folder as this newer work contains a test suite, numerous fixes, etc. It is still pre-beta quality, but is undergoing more regular change, so you should expect its API to change without notice when needed to support newer features as they are added. 

This code generates ```npm``` modules that can be used with Node.js.  As with the other implementations in Python and C, the MAVLink protocol is specified in XML manifests which can be modified to add custom messages.

*See the gotcha's and todo's section below* for some important caveats.  This implementation should be considered pre-beta: it creates a working MAVLink parser, but there's plenty of rough edges in terms of API.

### Generating the JS implementation ###

Folders in the ```implementations/``` directory are ```npm``` modules, automatically generated from XML manifests that are in the [mavlink/mavlink](https://github.com/mavlink/mavlink) project.  If you wish to generate custom MAVLink packets, you would need to follow the directions there.

You need to have Node.js and npm installed to build.  

To build the Javascript implementations:

```bash
npm install
```

or

```bash
make
```

(which calls npm)

### Usage in Node.js ###

The generated modules emit events when valid MAVLink messages are encountered.  The name of the event is the same as the name of the message: ```HEARTBEAT```, ```FETCH_PARAM_LIST```, ```REQUEST_DATA_STREAM```, etc.  In addition, a generic ```message``` event is emitted whenever a message is successfully decoded.

The below code is a rough sketch of how to use the generated module in Node.js.  A somewhat more complete (though early, early alpha) example can be found [here](https://github.com/acuasi/ground-control-station).

#### Generating the parser

After running the generator, copy the version of the MAVLink protocol you need into your project's ```node_modules``` folder, then enter that directory and install its dependencies using ```npm install```:

```bash
cp -R javascript/implementations/mavlink_ardupilotmega_v1.0 /path/to/my/project/node_modules/
cd /path/to/my/project/node_modules/mavlink_ardupilotmega_v1.0 && npm install
```

Then, you can use the MAVLink module, as sketched below.

#### Initializing the parser

In your ```server.js``` script, you need to include the generated parser and instantiate it; you also need some kind of binary stream library that can read/write binary data and emit an event when new data is ready to be parsed (TCP, UDP, serial port all have appropriate libraries in the npm-o-sphere).  The connection's "data is ready" event is bound to invoke the MAVLink parser to try and extract a valid message.

```javascript
// requires Underscore.js and jspack
// can use Winston for logging
// see package.json for dependencies for the implementation
var mavlink = require('mavlink_ardupilotmega_v1.0'), 
	net = require('net');

// Instantiate the parser
// logger: pass a Winston logger or null if not used
// 1: source system id
// 50: source component id
mavlinkParser = new MAVLink(logger, 1, 50);

// Create a connection -- can be anything that can receive/send binary
connection = net.createConnection(5760, '127.0.0.1');

// When the connection issues a "got data" event, try and parse it
connection.on('data', function(data) {
	mavlinkParser.parseBuffer(data);
});
```

#### Receiving MAVLink messages

If the serial buffer has a valid MAVLink message, the message is removed from the buffer and parsed.  Upon parsing a valid message, the MAVLink implementation emits two events: ```message``` (for any message) and the specific message name that was parsed, so you can listen for specific messages and handle them.

```javascript
// Attach an event handler for any valid MAVLink message
mavlinkParser.on('message', function(message) {
	console.log('Got a message of any type!');
	console.log(message);
});

// Attach an event handler for a specific MAVLink message
mavlinkParser.on('HEARTBEAT', function(message) {
	console.log('Got a heartbeat message!');
	console.log(message); // message is a HEARTBEAT message
});
```

#### Sending MAVLink messages

*See the gotcha's and todo's section below* for some important caveats.  The below code is preliminary and *will* change to be more direct.  At this point, the MAVLink parser doesn't manage any state information about the UAV or the connection itself, so a few fields need to be fudged, as indicated below.

Sending a MAVLink message is done by creating the message object, populating its fields, and packing/sending it across the wire.  Messages are defined in the generated code, and you can look up the parameter list/docs for each message type there.  For example, the message ```REQUEST_DATA_STREAM``` has this signature:

```javascript
mavlink.messages.request_data_stream = function(target_system, target_component, req_stream_id, req_message_rate, start_stop) //...
```

Creating the message is done like this:

```javascript
request = new mavlink.messages.request_data_stream(1, 1, mavlink.MAV_DATA_STREAM_ALL, 1, 1);

// Create a buffer consisting of the packed message, and send it across the wire.
// You need to pass a MAVLink instance to pack. It will then take care of setting sequence number, system and component id.
// Hack alert: the MAVLink connection could/should encapsulate this.
p = new Buffer(request.pack(mavlinkParser));
connection.write(p);
```

### Gotchas and todo's ###

JavaScript doesn't have 64bit integers (long). The library that replaces Pythons struct converts ```q``` and ```Q``` into 3 part arrays: ```[lowBits, highBits, unsignedFlag]```. 
We rely on our own lightly modified version of [long.js](https://github.com/dcodeIO/long.js) 
See local_modules/jspack/test/int64.js for examples.
We rely on our own heavily modified version of [jspack.js](https://github.com/birchroad/node-jspack) 
See the output of the generator which produces a number of mavlink.js that call: jspack = require("jspack").jspack

To see best-practice for how to assemble packets etc, we recommend looking at the test suite/s and seeing how they do it. starting with test_gen_js.sh, and generator output such as mavlink.tests.js and the autogenerated test/made_tests.js mocha tests file.

Current implementation tries to be as robust as possible. It doesn't throw errors but emits bad_data messages. Also it discards the buffer of a possible message as soon as if finds a valid prefix. Future improvements:
* Implement not so robust parsing: throw errors (similar to the Python version)
* Implement trying hard: parse buffer char by char and don't just discard the expected length buffer if there is an error

This code isn't great idiomatic Javascript (yet!), instead, it's more of a line-by-line translation from Python as much as possible.

The Python MAVLink code manages some information about the connection status (system/component attached, bad packets, durations/times, etc), and that work isn't completely present in this code yet.

Code to create/send MAVLink messages to a client is very clumsy at this point in time *and will change* to make it more direct.

Publish generated scripts as npm module.

### Development ###

Unit tests cover as much of it as we can, including tests that try to parse C generated test packets, as we ll as basic packing/unpacking functionality against mock binary buffers representing valid MAVlink generated by the Python implementation.  You need to have [mocha](http://visionmedia.github.com/mocha/) installed to run the unit tests.

To run tests, use npm:

```bash
npm test
```

Specific instructions for generating Jenkins-friendly output is done through the makefile as well:

```bash
make ci
```

