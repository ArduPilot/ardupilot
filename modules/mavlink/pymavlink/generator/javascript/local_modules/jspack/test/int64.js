//  This file is MODIFIED from the original, by buzz 2020, please see README.md in the upper level folder for more details.
var should = require('should');
var jspack = require('../jspack.js').jspack;
var Long = require('long');

describe('Test long integration (examples):', function() {

    // Demonstrating the use together with Long.js (https://github.com/dcodeIO/Long.js)
    //
    // Packing a long requires the input of a 2 part array containing the [low, high] bits
    // of the specific long value.
    // Unpacking a long results in a 3 part array containing [low, high, unsigned] bits and flag.
    // The decoded value can be applied directly to Long.fromBits()
    //
    // Test number u            228290380562207 (BE: 0x00, 0x00, 0xcf, 0xa0, 0xff, 0x09, 0xff, 0x1f)
    //                                          (LE: 0x1f, 0xff, 0x09, 0xff, 0xa0, 0xcf, 0x00, 0x00)
    // Test number s           -228290380562207 (BE: 0xff, 0xff, 0x30, 0x5f, 0x00, 0xf6, 0x00, 0xe1)
    //                                          (LE: 0xe1, 0x00, 0xf6, 0x00, 0x5f, 0x30, 0xff, 0xff)

    it('pack <Q', function() {
        //var num = new Long(0xff09ff1f, 0x0000cfa0, true);
        var num = Long.fromNumber(228290380562207, true);

        // Pass long representation to jspack
        var buf = jspack.Pack('<Q', [[num.getLowBitsUnsigned(), num.getHighBitsUnsigned()]]);

        buf.should.be.eql([0x1f, 0xff, 0x09, 0xff, 0xa0, 0xcf, 0x00, 0x00]);
    });

    it('unpack <Q', function() {
        var testNum = new Long(0xff09ff1f, 0x0000cfa0, true); // unsigned
        var buf = jspack.Unpack('<Q', [0x1f, 0xff, 0x09, 0xff, 0xa0, 0xcf, 0x00, 0x00]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);

        // Create long by passing unpacked buffer to it (either way works)
        var res = Long.fromBits.apply(undefined, buf[0]);
        var res2 = new Long(buf[0][0], buf[0][1], buf[0][2]);

        testNum.equals(res).should.be.true;
        testNum.equals(res2).should.be.true;
        res.toNumber().should.be.eql(228290380562207);
    });

    it('pack <q', function() {
        //var num = new Long(0x00f600e1, 0xffff305f, true);
        var num = Long.fromNumber(-228290380562207); // signed

        // Pass long representation to jspack
        var buf = jspack.Pack('<q', [[num.getLowBitsUnsigned(), num.getHighBitsUnsigned()]]);

        buf.should.be.eql([0xe1, 0x00, 0xf6, 0x00, 0x5f, 0x30, 0xff, 0xff]);
    });

    it('unpack <q', function() {
        var testNum = new Long(0x00f600e1, 0xffff305f); // signed
        var buf = jspack.Unpack('<q', [0xe1, 0x00, 0xf6, 0x00, 0x5f, 0x30, 0xff, 0xff]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        
        // Create long by passing unpacked buffer to it (either way works)
        var res = Long.fromBits.apply(undefined, buf[0]);
        var res2 = new Long(buf[0][0], buf[0][1], buf[0][2]);

        testNum.equals(res).should.be.true;
        testNum.equals(res2).should.be.true;
        res.toNumber().should.be.eql(-228290380562207);
    });

});

describe('Test signed/unsigned int64:', function() {

    // Number 0xffa0ffe1ffff, packed with Python struct:
    // little endian:
    // 0xff, 0xff, 0xe1, 0xff, 0xa0, 0xff, 0x00, 0x00
    // big endian:
    // 0x00, 0x00, 0xff, 0xa0, 0xff, 0xe1, 0xff, 0xff

    it('pack <Q', function() {
        var buf = jspack.Pack('<Q', [[0xffe1ffff, 0xffa0]]);
        buf.should.be.eql([0xff, 0xff, 0xe1, 0xff, 0xa0, 0xff, 0x00, 0x00]);
    });

    it('pack >Q', function() {
        var buf = jspack.Pack('>Q', [[0xffe1ffff, 0xffa0]]);
        buf.should.be.eql([0x00, 0x00, 0xff, 0xa0, 0xff, 0xe1, 0xff, 0xff]);
    });

    it('unpack <Q', function() {
        var buf = jspack.Unpack('<Q', [0xff, 0xff, 0xe1, 0xff, 0xa0, 0xff, 0x00, 0x00]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0xffe1ffff);
        buf[0][1].should.be.eql(0xffa0);
        buf[0][2].should.be.true;
    });

    it('unpack >Q', function() {
        var buf = jspack.Unpack('>Q', [0x00, 0x00, 0xff, 0xa0, 0xff, 0xe1, 0xff, 0xff]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0xffe1ffff);
        buf[0][1].should.be.eql(0xffa0);
        buf[0][2].should.be.true;
    });

    // Test lower-case q as well. This only test the matching of the character and the unsigned bit,
    // the parsing is the same as for upper-case Q (since we don't actually convert to a number).
    it('pack >q (signed)', function() {
        var buf = jspack.Pack('>q', [[0xffe1ffff, 0xffa0]]);
        buf.should.be.eql([0x00, 0x00, 0xff, 0xa0, 0xff, 0xe1, 0xff, 0xff]);
    });

    it('unpack <q (signed)', function() {
        var buf = jspack.Unpack('<q', [0xff, 0xff, 0xe1, 0xff, 0xa0, 0xff, 0x00, 0x00]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0xffe1ffff);
        buf[0][1].should.be.eql(0xffa0);
        buf[0][2].should.be.false;
    });

});

//32 bits
function dec2bin(dec){
    var x = (dec >>> 0).toString(2);
    y = ("00000000000000000000000000000000" + x).slice(-32)
    y1 = y.substring(0,8);
    y2 = y.substring(8,16);
    y3 = y.substring(16,24);
    y4 = y.substring(24,32);
    return [y,y1,y2,y3,y4];
}
function dec2bin_ws(dec) {
        var str = dec2bin(dec);
        var bb = str.slice(1); //1-4 skipping zero
        var bbj = bb.join(' ');
    return bbj;
}

describe('ASCII Boundary tests:', function() {

    it('pack <4s correctly over the ascii 127->128->129 boundary', function() { // should work in range 0-255 if u use 'binary' encoding

        this.format = '<4s';

        this.ascii_bytes = new Buffer.from([ 126, 127, 128, 129]).toString('binary'); // 'binary' encoding is important here, as without it values above 128 are treated as unicode.
        var buf = jspack.Pack(this.format, [ this.ascii_bytes]);
        body = [ 0x7e, 0x7f, 0x80, 0x81];  // expected result
        buf.should.be.eql(body);

    });

    it('long Q buzz', function() { // should work in range 0-255 if u use 'binary' encoding

//from aoa_ssa

        this.format = '<Q';

        var num  = Long.fromNumber(93372036854775807, true); // fieldtype: uint64_t  isarray: False 

       // console.log(JSON.stringify(num)); 

        this.time_usec =  [num.getLowBitsUnsigned(), num.getHighBitsUnsigned()];

        //console.log(JSON.stringify(this.time_usec)); 

        var orderedfields = [ this.time_usec];
        //var flattened=[].concat.apply([], orderedfields);
        var buf = jspack.Pack(this.format, orderedfields);
        //console.log(JSON.stringify(buf)); 

        body = [  0, 0, 175, 112, 95, 185, 75, 1];  // expected result
        buf.should.be.eql(body);
    });


    it('IBB buzz', function() { // should work in range 0-255 if u use 'binary' encoding

//from device_op_write

            this.format = '<IBBBBBBB';

            this.request_id = 963497464; // fieldtype: uint32_t  isarray: False 
            this.target_system = 17; // fieldtype: uint8_t  isarray: False 
            this.target_component = 84; // fieldtype: uint8_t  isarray: False 
            this.bustype = 151; // fieldtype: uint8_t  isarray: False 
            this.bus = 218; // fieldtype: uint8_t  isarray: False 
            this.address = 29; // fieldtype: uint8_t  isarray: False 
            //this.busname = "JKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUV"; // fieldtype: char  isarray: False 
            this.regstart = 216; // fieldtype: uint8_t  isarray: False 
            this.count = 27; // fieldtype: uint8_t  isarray: False 
            //this.data = new Buffer.from([94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221]).toString("binary");
       // console.log(JSON.stringify(num)); 


            var orderedfields = [ this.request_id, this.target_system, this.target_component, this.bustype, this.bus, this.address, this.regstart, this.count];

            var buf = jspack.Pack(this.format, orderedfields);
            //console.log(JSON.stringify(buf)); 

            var reference = [0xf8, 0xcd, 0x6d, 0x39, 0x11, 0x54, 0x97, 0xda, 0x1d, 0xd8, 0x1b]; // while ref packet

            // var body = new Buffer.from(reference.slice(10,-2)); // strip header and crc

            buf.should.be.eql(reference);

    });

   it('signed 8-bit outside of -127-127 range', function() { 

//from end of sys_status

        this.format = '<b';

        this.battery_remaining =  223;

        var ui8 = (new Int8Array([this.battery_remaining]))[0]; // cast value that is clearly outside the -127-0-127 range to signed data

        console.log(JSON.stringify(ui8)); 

        var orderedfields = [ ui8 ];
        //var flattened=[].concat.apply([], orderedfields);
        var buf = jspack.Pack(this.format, orderedfields);
        //console.log(JSON.stringify(buf)); 

        body = [  223 ];  // expected result
        buf.should.be.eql(body);
    });


   it('32 bit overflow buzz', function() { // should work in range 0-255 if u use 'binary' encoding

//from aoa_ssa

        this.format = '<Q';

//        var ui64 = Long.fromNumber(93372036854775807, true); // fails and is clearly a bug in the Long library
        var ui64 = Long.fromString('93372036854775807', true,10); // passes and is an ok workaround

        // according to the internet 93372036854775807 =  00000001 01001011 10111001 01011111 01110000 10101110 11111111 11111111
        // which means the LOW 32bits are: 01110000 10101110 11111111 11111111

        var lb = ui64.getLowBitsUnsigned();
        var strlb = dec2bin_ws(lb);
        console.log("lb1:"+strlb)
        strlb.should.be.eql('01110000 10101110 11111111 11111111');

        var x = [ui64.getLowBitsUnsigned(), ui64.getHighBitsUnsigned()]

        this.time_usec = [ui64.getLowBitsUnsigned(), ui64.getHighBitsUnsigned()]; // fieldtype: uint64_t  isarray: False 
        this.AOA = 73.0; // fieldtype: float  isarray: False 
        this.SSA = 101.0; // fieldtype: float  isarray: False 

        var orderedfields = [ this.time_usec];

        var buf = jspack.Pack(this.format, orderedfields);
        console.log(JSON.stringify("thing:"+this.time_usec)); 

        body = [  0xff, 0xff, 0xae, 0x70, 0x5f, 0xb9, 0x4b, 0x01,  ];  // expected result
        buf.should.be.eql(body);
    });


   it('ftp ascii buzz', function() { // should work in range 0-255 if u use 'binary' encoding

//from file_transfer_protocol

      this.format = '<BBB251s';

      this.target_network = 5; // fieldtype: uint8_t  isarray: False 
      this.target_system = 72; // fieldtype: uint8_t  isarray: False 
      this.target_component = 139; // fieldtype: uint8_t  isarray: False 
      this.payload = new Buffer.from([206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200]).toString("binary"); // fieldtype: uint8_t  isarray: True 

      // console.log("lb1:"+strlb)
      var orderedfields = [ this.target_network, this.target_system, this.target_component, this.payload];

        var buf = jspack.Pack(this.format, orderedfields);
        console.log(JSON.stringify("thing:"+this.time_usec)); 

        var body = [ 0x05, 0x48, 0x8b, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8];

        buf.should.be.eql(body);
    });


   it('jpg thing buzz', function() { // should work in range 0-255 if u use 'binary' encoding

//from data_transmission_handshake

    this.format = '<IHHHBBB';

      this.size = 963497464; // fieldtype: uint32_t  isarray: False 
      this.width = 17443; // fieldtype: uint16_t  isarray: False 
      this.height = 17547; // fieldtype: uint16_t  isarray: False 
      this.packets = 17651; // fieldtype: uint16_t  isarray: False 
      this.type = 163; // fieldtype: uint8_t  isarray: False 
      this.payload = 230; // fieldtype: uint8_t  isarray: False 
      this.jpg_quality = 41; // fieldtype: uint8_t  isarray: False 

       var orderedfields = [ this.size, this.width, this.height, this.packets, this.type, this.payload, this.jpg_quality];

        var buf = jspack.Pack(this.format, orderedfields);
      
        var body = [0xf8, 0xcd, 0x6d, 0x39, 0x23, 0x44, 0x8b, 0x44, 0xf3, 0x44, 0xa3, 0xe6, 0x29];

        buf.should.be.eql(body);
    });



 it('header thing buzz', function() {

// header 10 bytes from 'encode data_transmission_handshake from C'

//    var orderedfields =  [253, this.mlen, this.incompat_flags, this.compat_flags, this.seq, this.srcSystem, this.srcComponent, ((this.msgId & 0xFF) << 8) | ((this.msgId >> 8) & 0xFF), this.msgId>>16];

    this.msgId = 130;

    var v1 = ((this.msgId & 0xFF) << 8) | ((this.msgId >> 8) & 0xFF);
    var v2 = this.msgId>>16;

    v1.should.be.eql(33280);
    v2.should.be.eql(0);

    var orderedfields =  [253,13,0,0,40,11,10,33280,0];

    console.log("------------------------------------------------------------------------\nmavheader:"+JSON.stringify(orderedfields));
    var hdr =  jspack.Pack('BBBBBBBHB',orderedfields);

     buf = [0xfd, 0x0d, 0x00, 0x00, 0x28, 0x0b, 0x0a, 0x82, 0x00, 0x00];

     buf.should.be.eql(hdr);
    });

});

describe('Q Boundary tests:', function() {

    it('unpack >Q full', function() {
        var buf = jspack.Unpack('>Q', [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0xffffffff);
        buf[0][1].should.be.eql(0xffffffff);
        buf[0][2].should.be.true;
    });

    it('pack >Q full', function() {
        var buf = jspack.Pack('>Q', [[0xffffffff, 0xffffffff]]);
        buf.should.be.eql([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
    });

    it('unpack <Q full', function() {
        var buf = jspack.Unpack('<Q', [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0xffffffff);
        buf[0][1].should.be.eql(0xffffffff);
        buf[0][2].should.be.true;
    });

    it('pack <Q full', function() {
        var buf = jspack.Pack('<Q', [[0xffffffff, 0xffffffff]]);
        buf.should.be.eql([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
    });

    it('unpack >Q zero', function() {
        var buf = jspack.Unpack('>Q', [0, 0, 0, 0, 0, 0, 0, 0]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0);
        buf[0][1].should.be.eql(0);
        buf[0][2].should.be.true;
    });

    it('pack >Q zero', function() {
        var buf = jspack.Pack('>Q', [[0, 0]]);
        buf.should.be.eql([0, 0, 0, 0, 0, 0, 0, 0]);
    });

    it('unpack <Q zero', function() {
        var buf = jspack.Unpack('<Q', [0, 0, 0, 0, 0, 0, 0, 0]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0);
        buf[0][1].should.be.eql(0);
        buf[0][2].should.be.true;
    });

    it('pack <Q zero', function() {
        var buf = jspack.Pack('<Q', [[0, 0]]);
        buf.should.be.eql([0, 0, 0, 0, 0, 0, 0, 0]);
    });

    it('unpack >Q one', function() {
        var buf = jspack.Unpack('>Q', [1, 1, 1, 1, 1, 1, 1, 1]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0x01010101);
        buf[0][1].should.be.eql(0x01010101);
        buf[0][2].should.be.true;
    });

    it('pack >Q one', function() {
        var buf = jspack.Pack('>Q', [[0x01010101, 0x01010101]]);
        buf.should.be.eql([1, 1, 1, 1, 1, 1, 1, 1]);
    });

    it('unpack <Q one', function() {
        var buf = jspack.Unpack('<Q', [1, 1, 1, 1, 1, 1, 1, 1]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0x01010101);
        buf[0][1].should.be.eql(0x01010101);
        buf[0][2].should.be.true;
    });

    it('pack <Q one', function() {
        var buf = jspack.Pack('<Q', [[0x01010101, 0x01010101]]);
        buf.should.be.eql([1, 1, 1, 1, 1, 1, 1, 1]);
    });

    it('unpack >Q 0xfe', function() {
        var buf = jspack.Unpack('>Q', [0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0xfefefefe);
        buf[0][1].should.be.eql(0xfefefefe);
        buf[0][2].should.be.true;
    });

    it('pack >Q 0xfe', function() {
        var buf = jspack.Pack('>Q', [[0xfefefefe, 0xfefefefe]]);
        buf.should.be.eql([0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe]);
    });

    it('unpack <Q 0xfe', function() {
        var buf = jspack.Unpack('<Q', [0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe]);
        buf.length.should.be.eql(1);
        buf[0].length.should.be.eql(3);
        buf[0][0].should.be.eql(0xfefefefe);
        buf[0][1].should.be.eql(0xfefefefe);
        buf[0][2].should.be.true;
    });

    it('pack <Q 0xfe', function() {
        var buf = jspack.Pack('<Q', [[0xfefefefe, 0xfefefefe]]);
        buf.should.be.eql([0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe]);
    });

});

