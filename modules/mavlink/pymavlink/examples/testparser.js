//  minimal javascript tool to exercise pushbuffer and parsebuffer of different style mavlink including 1 and 2
//   - it reads a string-hex data stream from stdin (NOT strictly binary mavlink) that we know how to decode as mavlink, parse it and decode into valid packet/s.
//   - it reads data of the type output from command/s like these:
//    ardupilot mavlink2:
//    ./testmav2.0_ardupilotmega | grep '^fd'
//    ardupilot mavlink1:
//    ./testmav1.0_ardupilotmega | grep '^fe'
//
// (no external dependencies apart from the mavlink library itself and the stock filesystem library)
// by buzz june 2020
// if given any args, they should be the mavlink 'base' name and version number, eg: "node testparser.js ardupilotmega 2.0" if none given the default is 'ardupilot' and '2.0'
//
// This tool isn't smart enough to do anything other than print stats on the number of 'good' and 'unhandled' stuff that were
//   found in whatever was piped into its stdin.  
// Its certainly NOT able to know if these numbers are right or wrong.
// A much smarter and more complex version of this script is called 'make_tests.js' which makes tests that byte-level smart.

var base = process.argv[2];
var version = process.argv[3];

var verbosity = process.argv[4]; // 0 or absent means low verbosity, 1 means medium, 2 means high
function isNumeric(num){return !isNaN(num)}
if ( isNumeric(verbosity) )  { verbosity = parseInt(verbosity); }// extracts a numeric value from it 
//console.log('verbosity=',verbosity); 

if (!base) {
    base = 'ardupilotmega'; // or 'common'
}
if (!version){       version = '2.0';}
if (version == '1'){ version = '1.0';}
if (version == '2'){ version = '2.0';}

// run relative to this script, not the user, so the 'require' below works
process.chdir(__dirname);

var requirestring = '../generator/javascript/implementations/mavlink_'+base+'_v'+version+'/mavlink.js';
var M = require(requirestring);//module exports list
console.log("##############################################\n"); 
console.log("Using parser base:"+base+" and version:"+version);

// create mavlink handler
var MAVLinkProcessor;// which ever processor we are using
if (version == '2.0'){
    MAVLinkProcessor = M.MAVLink20Processor;
}
if (version == '1.0'){
    MAVLinkProcessor = M.MAVLink10Processor;
}
// instantiate a new one, now we know which to use
var mav = new MAVLinkProcessor();

// read test file from filesystem, needs to user to be in the same folder as this script to find the .tlog file
var fs = require('fs');

// attach minimal 'write' function that the js library requires to work.
var writer = {
                write: function(){}
            };
mav.file = writer;


// convenience wrappers to look more python-ish
function print(m){
console.log(m);
}
function str(m){
    return ''+m;
}
// and to display on stdout in a human-readable format:
function buf2hex(buffer) { // buffer is an ArrayBuffer
  return Array.prototype.map.call(new Uint8Array(buffer), x => ('00' + x.toString(16)).slice(-2)).join(' ');
}
function buf2decimal(buffer) { // buffer is an ArrayBuffer
  return Array.prototype.map.call(new Uint8Array(buffer), x => ('0' + x.toString(10)).slice(-3)).join(', ');
}

var goodpacketcount = 0;
var unknownbytecount = 0;
var packet_types = {}; // we'll keep a hash lookup of each type we got.

// just using the generic 'message' events from being emitted  from the mavlink library as well.
mav.on('message', function(message) {


                //    this._id = ${MAVHEAD}.MAVLINK_MSG_ID_BAD_DATA;
                if (message._id != -1) { // bad data has this set to -1

                     //console.log("test parsed packet type:"+message._name+" ... "+JSON.stringify(message));

                     //console.log("test parsed packet type:"+message._name);
                     process.stdout.write("test parsed packet type:"+message._name+"                   \r");
                     goodpacketcount = goodpacketcount+1;

                     // rememmber the packt type for stats at end
                     packet_types[message._name] = 1;
                }
                else {

                    // example of how to console.log just the first bad_data packet...
                    //if (unknownbytecount == 0 ) {
                    //    console.log(message); 
                    //}

                    if ( verbosity ==2  ) console.log(message);

                    unknownbytecount = unknownbytecount+1;// just count the number of packets we couldn't parse

                    if ((version == '2.0') && (message._msgbuf[0] == 0xfe)) {
                        if ( verbosity > 0 ) console.log("looks like maybe mavlink1 (0xfe) data when trying to parse mavlink2"); 
                    }
                    if ((version == '1.0') && (message._msgbuf[0] == 0xfd)) {
                        if ( verbosity > 0 ) console.log("looks like maybe mavlink2 (0xfd) data when trying to parse mavlink1"); 
                    }
            }
 });


process.stdin.on('readable', () => {
    let chunk;
    while ((chunk = process.stdin.read()) !== null) {
        var str = "".concat(chunk); 
        str2 = str.split(" ").join("");
        lines = str2.split("\n");// .join("");
        lines.forEach( e => { 
            if (e == "") return;  
            var array_of_chars = Buffer.from(e,'hex');
            mav.pushBuffer(array_of_chars);
            m = mav.parseBuffer();
        } );
    }
});

// little summary at the end
process.stdin.on('end', function() {
    console.log("\n\nunique packet types:"+Object.keys(packet_types).length); 
    if ((goodpacketcount != 0 ) || (unknownbytecount != 0 ) ){
    console.log("good packets       :"+goodpacketcount); goodpacketcount=0;
    console.log("unhandled bytes    :"+unknownbytecount+"\n");
    console.log("----------------------------------------------\n"); 
    unknownbytecount=0;
    }

});





