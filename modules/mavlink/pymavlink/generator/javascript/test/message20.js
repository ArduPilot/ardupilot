should = require('should');

describe('MAVLink 2.0 message registry', function() {

    beforeEach(function() {
        var {mavlink20, MAVLink20Processor} = require('../implementations/mavlink_ardupilotmega_v2.0/mavlink.js');
    });

    it('defines constructors for every message', function() {
        mavlink20.messages['gps_raw_int'].should.be.a.function;
    });

    it('assigns message properties, _format with int64 (q), gps_raw_int', function() {
        var m = new mavlink20.messages['gps_raw_int']();
        m._format.should.equal("<QiiiHHHHBBiIIIIH");
        m.order_map.should.eql([0, 8, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15]); // should.eql = shallow comparison
        m.crc_extra.should.equal(24);
        m._id.should.equal(mavlink20.MAVLINK_MSG_ID_GPS_RAW_INT);
    });

    it('assigns message properties, heartbeat', function() {
        var m = new mavlink20.messages['heartbeat']();
        m._format.should.equal("<IBBBBB");
        m.order_map.should.eql([1, 2, 3, 0, 4, 5]); // should.eql = shallow comparison
        m.crc_extra.should.equal(50);
        m._id.should.equal(mavlink20.MAVLINK_MSG_ID_HEARTBEAT);
    });

});

describe('Complete MAVLink 2.0 packet', function() {

    beforeEach(function() {
        var {mavlink20, MAVLink20Processor} = require('../implementations/mavlink_ardupilotmega_v2.0/mavlink.js');
        this.mav = new MAVLink20Processor(null, 42, 150);
    });

    it('encode gps_raw_int', function() {

        // 0x75bcd15 = 123456789
        // as long as the number is no bigger than max signed int (2147483648) it can be passed like the following
        var gpsraw = new mavlink20.messages.gps_raw_int(
            time_usec=[123456789, 0]
            , fix_type=3
            , lat=47123456
            , lon=8123456
            , alt=50000
            , eph=6544
            , epv=4566
            , vel=1235
            , cog=1234
            , satellites_visible=9
        );

        this.mav.seq = 5;
        this.mav.srcSystem=42;
        this.mav.srcComponent=150;

        // Create a buffer that matches what the Python version of MAVLink creates
        var reference = new Buffer.from([0xfd, 0x1e, 0x00, 0x00, 0x05, 0x2a, 0x96, 0x18, 0x00, 0x00, 0x15, 0xcd, 0x5b, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xcf, 0x02, 0x40, 0xf4, 0x7b, 0x00, 0x50, 0xc3, 0x00, 0x00, 0x90, 0x19, 0xd6, 0x11, 0xd3, 0x04, 0xd2, 0x04, 0x03, 0x09, 0x20, 0x2d]);
        new Buffer.from(gpsraw.pack(this.mav)).should.eql(reference);

    });

    it('encode gps_raw_int with long integer', function() {

        // number ~2^60
        // 1152221500606846977 = 0xffd8359 9e3d1801
        var gpsraw = new mavlink20.messages.gps_raw_int(
            time_usec=[0x9e3d1801, 0xffd8359]
            , fix_type=3
            , lat=47123456
            , lon=8123456
            , alt=50000
            , eph=6544
            , epv=4566
            , vel=1235
            , cog=1234
            , satellites_visible=9
        );

        this.mav.seq = 5;
        this.mav.srcSystem=42;
        this.mav.srcComponent=150;
        
        // Create a buffer that matches what the Python version of MAVLink creates
        var reference = new Buffer.from([0xfd, 0x1e, 0x00, 0x00, 0x05, 0x2a, 0x96, 0x18, 0x00, 0x00, 0x01, 0x18, 0x3d, 0x9e, 0x59, 0x83, 0xfd, 0x0f, 0x00, 0x0c, 0xcf, 0x02, 0x40, 0xf4, 0x7b, 0x00, 0x50, 0xc3, 0x00, 0x00, 0x90, 0x19, 0xd6, 0x11, 0xd3, 0x04, 0xd2, 0x04, 0x03, 0x09, 0xa2, 0xd3]);
        new Buffer.from(gpsraw.pack(this.mav)).should.eql(reference);
    });


    it('encode heartbeat', function() {

        var heartbeat = new mavlink20.messages.heartbeat(
            type=5
            , autopilot=3
            , base_mode=45
            , custom_mode=68
            , system_status=13
        );
        
        this.mav.seq = 7;

        // Create a buffer that matches what the Python version of MAVLink creates
         var reference = new Buffer.from([0xfd, 0x09, 0x00, 0x00, 0x07, 0x2a, 0x96, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x05, 0x03, 0x2d, 0x0d, 0x03, 0xa6, 0xe4]);
        new Buffer.from(heartbeat.pack(this.mav)).should.eql(reference);

    });


   // in mavlink2, this packet has extensions, so being sure they work is useful.
   it('encode BATTERY_STATUS, zeroed extensions', function() {

        var bs = new mavlink20.messages.battery_status(
            type=0
            , id=0
            , battery_function=0
            , temperature=32767
            , voltages = [12587, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]
            , current_battery = 0
            , current_consumed = 0
            , energy_consumed = 0
            , battery_remaining = 100
            , time_remaining = 0  // if we zero all these extensions, we get the short version
            , charge_state = 0    // if we zero all these extensions, we get the short version
        );
        
        this.mav.seq = 172;
        this.mav.srcSystem=1;
        this.mav.srcComponent=1;

        // Create a buffer that matches what the Python version of MAVLink creates

        // without extension fields it looks like: ( shorter, at length 0x24  )
        var reference = new Buffer.from([0xfd, 0x24, 0x00, 0x00, 0xac, 0x01, 0x01, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x2b, 0x31, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x15, 0xc3]);

        new Buffer.from(bs.pack(this.mav)).should.eql(reference);
    });

  // in mavlink2, this packet has extensions, so being sure they work is useful.
   it('encode BATTERY_STATUS, non-zeroed extensions', function() {

        var bs = new mavlink20.messages.battery_status(
            type=0
            , id=0
            , battery_function=0
            , temperature=32767
            , voltages = [12587, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]
            , current_battery = 0
            , current_consumed = 0
            , energy_consumed = 0
            , battery_remaining = 100
            , time_remaining = 2  // if we were to zero all these extensions, we'd get the short version
            , charge_state = 1    // if we were to zero all these extensions, we'd get the short version
        );
        
        this.mav.seq = 172;
        this.mav.srcSystem=1;
        this.mav.srcComponent=1;

        // Create a buffer that matches what the Python version of MAVLink creates

        // with extension bytes it looks like: ( longer, than the above (0x24) at length 0x29 )
        // current impl has xxx bytes worth of extension fields being inserted after the 0x64 but before final 2 checksum bytes
        // at the moment we just zero-fill them and tweak the checksum...
         var reference = new Buffer.from([0xfd, 0x29, 0x00, 0x00, 0xac, 0x01, 0x01, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x2b, 0x31, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 
0x02, 0x00, 0x00, 0x00, //time_remaining is 32 bits
0x01,                   //charge_state is 8 bits 
 0xed, 0xf6]); // revised checksum

        new Buffer.from(bs.pack(this.mav)).should.eql(reference);
    });


 it('decode BATTERY_STATUS with array, and no extension bytes', function() {


        // Create a 'short' buffer that matches what the Python version of MAVLink creates
         var reference = new Buffer.from([0xfd, 0x24, 0x00, 0x00, 0xac, 0x01, 0x01, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x2b, 0x31, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x15, 0xc3]);

        var m = new MAVLink20Processor();

        var msg = m.parseBuffer(reference);

       // console.log(JSON.stringify(msg));

        // how to check if crc was invalid or parsing failed for some reason and gave a _reason
        msg[0].should.not.have.property('_reason');

        // check header
        msg[0]._header.seq.should.eql(172);
        msg[0]._header.srcSystem.should.eql(1);
        msg[0]._header.srcComponent.should.eql(1);
        msg[0]._header.msgId.should.eql(147);

        // check packet id, note that ._id and .id are different here.
        msg[0]._id.should.eql(147);

        // check payload
        msg[0].id.should.eql(0);
        msg[0].battery_function.should.eql(0);
        msg[0].type.should.eql(0);
        msg[0].temperature.should.eql(32767);
        msg[0].voltages.should.eql([12587, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]);
        msg[0].current_battery.should.eql(0);
        msg[0].current_consumed.should.eql(0);
        msg[0].energy_consumed.should.eql(0);
        msg[0].battery_remaining.should.eql(100);

        // check zero'd extension fields
        msg[0].time_remaining.should.eql(0);
        msg[0].charge_state.should.eql(0);
        //msg[0].voltages_ext.should.eql([0,0,0,0]); this extension isn't part of the ardupilot xml

    });

 it('decode BATTERY_STATUS with array, and also zero-d extension bytes', function() {


        // Create a 'long' buffer that matches what the Python version of MAVLink creates
         var reference = new Buffer.from([0xfd, 0x31, 0x00, 0x00, 0xac, 0x01, 0x01, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x2b, 0x31, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // zero fill of 13 bytes
 0x80, 0xa7]);

        var m = new MAVLink20Processor();

        var msg = m.parseBuffer(reference);

        //console.log(JSON.stringify(msg));

        // how to check if crc was invalid or parsing failed for some reason and gave a _reason
        msg[0].should.not.have.property('_reason');

        // check header
        msg[0]._header.seq.should.eql(172);
        msg[0]._header.srcSystem.should.eql(1);
        msg[0]._header.srcComponent.should.eql(1);
        msg[0]._header.msgId.should.eql(147);

        // check payload
        msg[0].id.should.eql(0);
        msg[0].battery_function.should.eql(0);
        msg[0].type.should.eql(0);
        msg[0].temperature.should.eql(32767);
        msg[0].voltages.should.eql([12587, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]);
        msg[0].current_battery.should.eql(0);
        msg[0].current_consumed.should.eql(0);
        msg[0].energy_consumed.should.eql(0);
        msg[0].battery_remaining.should.eql(100);

        //extension fields ... these could/should be non-zero for a proper check, but zero for now as it's not really working with non-zero
        msg[0].time_remaining.should.eql(0);
        msg[0].charge_state.should.eql(0);
        //msg[0].voltages_ext.should.eql([0,0,0,0]); this extension isn't part of the ardupilot xml

    });


 it('decode BATTERY_STATUS with array, and also non-zerod extension bytes', function() {


        // Create a un-signed 'long' buffer that matches what the Python version of MAVLink creates,
        //   with incompat_flags = 0  and a valid crc
         var reference = new Buffer.from([0xfd, 0x29, 0x00, 0x00, 0x58, 0x0b, 0x0a, 0x93, 0x00, 0x00, 0xf8, 0xcd, 0x6d, 0x39, 0xc8, 0xce, 0x6d, 0x39, 0xf3, 0x44, 0x5b, 0x45, 0x5c, 0x45, 0x5d, 0x45, 0x5e, 0x45, 0x5f, 0x45, 0x60, 0x45, 0x61, 0x45, 0x62, 0x45, 0x63, 0x45, 0x64, 0x45, 0x6b, 0x49, 0x65, 0xa8, 0xeb, 0x2e, 0x48, 0xd5, 0x6d, 0x39, 0x7d, 0x90, 0x0a]);

        var m = new MAVLink20Processor();

        var msg = m.parseBuffer(reference); // skipping parseChar

        //console.log(JSON.stringify(msg));

        // how to check if crc was invalid or parsing failed for some reason and gave a _reason
        msg[0].should.not.have.property('_reason');

        // check header
        msg[0]._header.seq.should.eql(88);
        msg[0]._header.srcSystem.should.eql(11);
        msg[0]._header.srcComponent.should.eql(10);
        msg[0]._header.msgId.should.eql(147);
        msg[0]._header.incompat_flags.should.eql(0);
        msg[0]._header.compat_flags.should.eql(0);

        // check payload
        msg[0].id.should.eql(101);
        msg[0].battery_function.should.eql(168);
        msg[0].type.should.eql(235);
        msg[0].temperature.should.eql(17651);
        msg[0].voltages.should.eql([17755,17756,17757,17758,17759,17760,17761,17762,17763,17764]);
        msg[0].current_battery.should.eql(18795);
        msg[0].current_consumed.should.eql(963497464);
        msg[0].energy_consumed.should.eql(963497672);
        msg[0].battery_remaining.should.eql(46);

        //extension fields ... these could/should be non-zero for a proper check, but zero for now as it's not really working with non-zero
        msg[0].time_remaining.should.eql(963499336);
        msg[0].charge_state.should.eql(125);
        //msg[0].voltages_ext.should.eql([19367,19368,19369,19370]); this extension isn't part of the ardupilot xml

    });

    it('decode gps_raw_int with long integer', function() {

        // number ~2^60
        // 1152221500606846977 = 0xffd8359 9e3d1801

        // Create a buffer that matches what the Python version of MAVLink creates
        var reference = new Buffer.from([0xfd, 0x1e, 0x00, 0x00, 0x05, 0x2a, 0x96, 0x18, 0x00, 0x00, 0x01, 0x18, 0x3d, 0x9e, 0x59, 0x83, 0xfd, 0x0f, 0x00, 0x0c, 0xcf, 0x02, 0x40, 0xf4, 0x7b, 0x00, 0x50, 0xc3, 0x00, 0x00, 0x90, 0x19, 0xd6, 0x11, 0xd3, 0x04, 0xd2, 0x04, 0x03, 0x09, 0xa2, 0xd3]);

        var m = new MAVLink20Processor();

        var msg = m.parseBuffer(reference);

        // check header
        msg[0]._header.seq.should.eql(5);
        msg[0]._header.srcSystem.should.eql(42);
        msg[0]._header.srcComponent.should.eql(150);

        // check payload
        msg[0].time_usec.should.eql([0x9e3d1801, 0xffd8359, true]);
        msg[0].fix_type.should.eql(3);
        msg[0].lat.should.eql(47123456);
        msg[0].lon.should.eql(8123456);
        msg[0].alt.should.eql(50000);
        msg[0].eph.should.eql(6544);
        msg[0].epv.should.eql(4566);
        msg[0].vel.should.eql(1235);
        msg[0].cog.should.eql(1234);
        msg[0].satellites_visible.should.eql(9);

    });


});

describe('MAVLink 2.0 header', function() {
    beforeEach(function() {
        var {mavlink20, MAVLink20Processor} = require('../implementations/mavlink_ardupilotmega_v2.0/mavlink.js');
        this.h = new mavlink20.header(mavlink20.MAVLINK_MSG_ID_PARAM_REQUEST_LIST, 1, 2, 3, 4);
    });

    it('Can pack itself', function() {
        this.h.pack().should.eql([253, 1, 0, 0, 2, 3, 4, 21, 0, 0]);
    });

});

describe('MAVLink 2.0 message', function() {

    beforeEach(function() {
        var {mavlink20, MAVLink20Processor} = require('../implementations/mavlink_ardupilotmega_v2.0/mavlink.js');
        // This is a heartbeat packet from a GCS to the APM.
        this.heartbeat = new mavlink20.messages.heartbeat(
            mavlink20.MAV_TYPE_GCS, // 6
            mavlink20.MAV_AUTOPILOT_INVALID, // 8
            0, // base mode, mavlink.MAV_MODE_FLAG_***
            0, // custom mode
            mavlink20.MAV_STATE_STANDBY, // system status
        );

        this.mav = new MAVLink20Processor();

    });

    it('has a set function to facilitate vivifying the object', function() {
        this.heartbeat.type.should.equal(mavlink20.MAV_TYPE_GCS);
        this.heartbeat.autopilot.should.equal(mavlink20.MAV_AUTOPILOT_INVALID);
        this.heartbeat.base_mode.should.equal(0);
        this.heartbeat.custom_mode.should.equal(0);
        this.heartbeat.system_status.should.equal(mavlink20.MAV_STATE_STANDBY);
    });

    // TODO: the length below (9) should perhaps be instead 7.  See mavlink.unpack().
    // might have to do with the length of the encoding (<I is 4 symbols in the array) 
    it('Can pack itself', function() {

        var packed = this.heartbeat.pack(this.mav);
        packed.should.eql([253, 9, 0, 0, 0, 0, 0, mavlink20.MAVLINK_MSG_ID_HEARTBEAT, 0, 0, // that bit is the header,
            // this is the payload, arranged in the order map specified in the protocol,
            // which differs from the constructor.
            0, 0, 0, 0, // custom bitfield -- length 4 (type=I)
            mavlink20.MAV_TYPE_GCS,
            mavlink20.MAV_AUTOPILOT_INVALID,
            0,
            mavlink20.MAV_STATE_STANDBY,
            3,
            207, // CRC
            58 // CRC
            ]);

    });

    describe('decode 2.0 function', function() {

        beforeEach(function() {
            var {mavlink20, MAVLink20Processor} = require('../implementations/mavlink_ardupilotmega_v2.0/mavlink.js');
            this.m = new MAVLink20Processor();
        });

        // need to add tests for the header fields as well, specifying seq etc.
        it('Can decode itself', function() {

            var packed = this.heartbeat.pack(this.m);
            var message = this.m.decode(packed);

            // this.fieldnames = ['type', 'autopilot', 'base_mode', 'custom_mode', 'system_status', 'mavlink_version'];
            message.type.should.equal(mavlink20.MAV_TYPE_GCS);  // supposed to be 6
            message.autopilot.should.equal(mavlink20.MAV_AUTOPILOT_INVALID); // supposed to be 8
            message.base_mode.should.equal(0); // supposed to be 0
            message.custom_mode.should.equal(0);
            message.system_status.should.equal(mavlink20.MAV_STATE_STANDBY); // supposed to be 3
            message.mavlink_version.should.equal(3); //only heartbeats have this, it's NOT a mavlink1 vs mavlink2 thing

        });

        it('throws an error if the message has a bad prefix', function() {
            var packed = [0, 3, 0, 0, 5, 7, 9, 0, 0, 11]; // bad data prefix in header (0, not 253)
            var m = this.m;
            (function() { m.decode(packed); }).should.throw('Invalid MAVLink prefix (0)');
        });

        it('throws an error if the message ID is not known', function() {
            var packed = [253, 1, 0, 0, 0, 3, 0, 199, 0, 0, 1, 0, 0]; // 199 = invalid ID
            var m = this.m;
            (function() { m.decode(packed); }).should.throw('Unknown MAVLink message ID (199)');
        });

        it('throws an error if the message length is invalid', function() {
            var packed = [253, 3, 0, 0, 257, 0, 0, 0, 0, 0, 0, 0];
            var m = this.m;
            (function() { m.decode(packed); }).should.throw('Invalid MAVLink message length.  Got 0 expected 3, msgId=0');
        });

        it('throws an error if the CRC cannot be unpacked', function() {
            
        });

        it('throws an error if the CRC can not be decoded', function() {

        });

        it('throws an error if it is unable to unpack the payload', function() {

        });

        it('throws an error if it is unable to instantiate a MAVLink message object from the payload', function() {

        });

    });
});
