#!/usr/bin/env node
// very minimal javascript impl to exercise both pack and unpack of mavlink2 
// 1 - read a mavlink2 file parse it and print valid packet/s from inside to stdout as hexadecimal
// 2 - create a mavlink2 packet as if for 'send'-ing and then just print to stdout as hexadecimal
// (no external dependencies apart from the mavlink library itself and the stock filesystem library)
// throw in some mavlink signing for one packet as well, very minimally
// by buzz june 2020
var {mavlink20, MAVLink20Processor} = require('../generator/javascript/implementations/mavlink_ardupilotmega_v2.0/mavlink.js');

// read test file from filesystem, needs the user to be in the same folder as this script to find the .tlog file
var fs = require('fs');
filename = "../generator/javascript/test/capture.mav2.battery_status.tlog";
serialStream = fs.readFileSync(filename);
var array_of_chars = Uint8Array.from(serialStream)


// create mavlink handler
var mav = new MAVLink20Processor();
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


mav.on('message', function(message) {

                if (message._id != -1) { // bad data has this set to -1
                    
                    if (message._name == 'BATTERY_STATUS'){ // demo of a specific packet
                        p = message; // p is shorthand for 'packet' or message
                        print('mlen='+str(p._header.mlen))
                        print('seq='+str(p._header.seq))
                        print('srcSystem='+str(p._header.srcSystem))
                        print('srcComponent='+str(p._header.srcComponent))
                        print('msgId='+str(p._header.msgId))
                        print('incompat_flags='+str(p._header.incompat_flags))
                        print('compat_flags='+str(p._header.compat_flags))
                        print('id='+str(p._id))
                        print('battery_function='+str(p.battery_function))
                        print('type='+str(p.type))
                        print('temperature='+str(p.temperature))
                        print('voltages=['+str(p.voltages)+"]")
                        print('current_battery='+str(p.current_battery))
                        print('current_consumed='+str(p.current_consumed))
                        print('energy_consumed='+str(p.energy_consumed))
                        print('energy_consumed='+str(p.energy_consumed))
                        print('battery_remaining='+str(p.battery_remaining))
                        print('time_remaining='+str(p.time_remaining))
                        print('charge_state='+str(p.charge_state))
                    }else{
                        console.log(message);//demo of a generic packet
                    }

                }
 });

// 
mav.pushBuffer(array_of_chars);
var messages = mav.parseBuffer();

console.log("from file: (unsigned)["+buf2decimal(array_of_chars.slice(8,56))+"]"); 

// 2:------------------------------

// eg:
// sysid=42, component=150
//mav = new MAVLink20Processor(null, 42, 150);

// sysid=1, component=1
a = {}
source_system = 42 
source_component = 11 
mav = new MAVLink20Processor(a, source_system, source_component);

mav.signing.secret_key =  Buffer.from([ 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42 ])
mav.signing.timestamp = [0,0,0,0,0,1];
mav.signing.link_id = 0
mav.signing.sign_outgoing = 1

mav.seq = 234

// after signing is active, create one packet , that should be signed and display it
mmm = new mavlink20.messages.heartbeat(type=17, autopilot=84, base_mode=151, custom_mode=963497464, system_status=218, mavlink_version=3);
ppp= mmm.pack(mav);

var b = new Buffer.from(ppp);
//print(b);
//console.log(buf2hex(b)); 
console.log("hb created: (signed) ["+buf2decimal(b)+"]"); 


// now we'll do an unsigned packet as well..
mav.seq = 172;
mav.signing.sign_outgoing = 0

// create a packet of this type and populate it
var bs = new mavlink20.messages.battery_status(
     type=0,
     id=0,
     battery_function=0,
     temperature=32767,
     voltages = [12587, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535],
     current_battery = 0,
     current_consumed = 0,
     energy_consumed = 0,
     battery_remaining = 100,
     time_remaining = 0,
     charge_state = 0
);


var b = new Buffer.from(bs.pack(mav));
//print(b);
//console.log(buf2hex(b)); 
console.log("bs created: (unsigned) ["+buf2decimal(b)+"]"); 


