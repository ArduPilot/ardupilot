var {mavlink20, MAVLink20Processor} = require('../implementations/mavlink_common_v2.0/mavlink.js'),
    should = require('should'),
    sinon = require('sinon'),
    fs = require('fs');

// Actual data stream taken from APM.
global.fixtures = global.fixtures || {};
global.fixtures.serialStream = fs.readFileSync("test/capture.mavlink");
//global.fixtures.heartbeatBinaryStream = fs.readFileSync("javascript/test/heartbeat-data-fixture");

describe("Generated MAVLink 2.0 protocol handler object", function() {

    beforeEach(function() {
        this.m = new MAVLink20Processor();

        // Valid heartbeat payload
        this.heartbeatPayload = new Buffer.from([0xfd, 0x09, 0x00, 0x00, 0x03, 0xff , 0x00, 0x00, 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x06 , 0x08 , 0x00 , 0x00 , 0x03, 0xc5, 0xa5]);

        // Complete but invalid message
        this.completeInvalidMessage = new Buffer.from([0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0xe0, 0x00, 0x00]);
    });

    describe("message header handling", function() {
        
        it("IDs and sequence numbers are set on send", function(){
            var mav = new MAVLink20Processor(null, 42, 99);
            var writer = {
                write: function(){}
            };
            mav.file = writer;
            var spy = sinon.spy(writer, 'write');

            var msg = new mavlink20.messages['heartbeat']();
            mav.send(msg);

            spy.calledOnce.should.be.true;
            spy.getCall(0).args[0][4].should.be.eql(0); // seq
            spy.getCall(0).args[0][5].should.be.eql(42); // sys
            spy.getCall(0).args[0][6].should.be.eql(99); // comp
        });

        it("sequence number increases on send", function(){
            var mav = new MAVLink20Processor(null, 42, 99);
            var writer = {
                write: function(){}
            };
            mav.file = writer;
            var spy = sinon.spy(writer, 'write');

            var msg = new mavlink20.messages['heartbeat']();
            mav.send(msg);
            mav.send(msg);

            spy.callCount.should.be.eql(2);
            spy.getCall(0).args[0][4].should.be.eql(0); // seq
            spy.getCall(0).args[0][5].should.be.eql(42); // sys
            spy.getCall(0).args[0][6].should.be.eql(99); // comp
            spy.getCall(1).args[0][4].should.be.eql(1); // seq
            spy.getCall(1).args[0][5].should.be.eql(42); // sys
            spy.getCall(1).args[0][6].should.be.eql(99); // comp
        });

        it("sequence number turns over at 256", function(){
            var mav = new MAVLink20Processor(null, 42, 99);
            var writer = {
                write: function(){}
            };
            mav.file = writer;
            var spy = sinon.spy(writer, 'write');

            var msg = new mavlink20.messages['heartbeat']();

            for(var i = 0; i < 258; i++){
                mav.send(msg);
                var seq = i % 256;
                spy.getCall(i).args[0][4].should.be.eql(seq); // seq
            }
        });

    });

    describe("buffer decoder (parseBuffer)", function() {

        // This test prepopulates a single message as a binary buffer.
        it("decodes a binary stream representation of a single message correctly", function() {
            this.m.pushBuffer(global.fixtures.heartbeatBinaryStream);
            var messages = this.m.parseBuffer();
            
        });

        // This test includes a "noisy" signal, with non-mavlink data/messages/noise.
        it("decodes a real serial binary stream into an array of MAVLink messages", function() {
            this.m.pushBuffer(global.fixtures.serialStream);
            var messages = this.m.parseBuffer();
        });

        it("decodes at most one message, even if there are more in its buffer", function() {

        });
        
        it("returns null while no packet is available", function() {
            (this.m.parseBuffer() === null).should.equal(true); // should's a bit tortured here
        });

    });

    describe("decoding chain (parseChar)", function() {

        it("returns a bad_data message if a borked message is encountered", function() {
            var b = new Buffer.from([3, 0, 1, 2, 3, 4, 5]); // invalid message
            var message = this.m.parseChar(b);
            message.should.be.an.instanceof(mavlink20.messages.bad_data);      
        });

        it("emits a 'message' event, provisioning callbacks with the message", function(done) {
            this.m.on('message', function(message) {
                message.should.be.an.instanceof(mavlink20.messages.heartbeat);
                done();
            });
            this.m.parseChar(this.heartbeatPayload);
        });

        it("emits a 'message' event for bad messages, provisioning callbacks with the message", function(done) {
            var b = new Buffer.from([3, 0, 1, 2, 3, 4, 5, 6, 7]); // invalid message
            this.m.on('message', function(message) {
                message.should.be.an.instanceof(mavlink20.messages.bad_data);
                done();
            });
            this.m.parseChar(b);
        });

        it("on bad prefix: cuts-off first char in buffer and returns correct bad data", function() {
            var b = new Buffer.from([3, 0, 1, 2, 3, 4, 5, 6, 7]); // invalid message
            var message = this.m.parseChar(b);
            message.msgbuf.length.should.be.eql(1);
            message.msgbuf[0].should.be.eql(3);
            this.m.buf.length.should.be.eql(8);
            // should process next char
            message = this.m.parseChar();
            message.msgbuf.length.should.be.eql(1);
            message.msgbuf[0].should.be.eql(0);
            this.m.buf.length.should.be.eql(7);
        });

        it("on bad message: cuts-off message length and returns correct bad data", function() {
            var message = this.m.parseChar(this.completeInvalidMessage);
            message.msgbuf.length.should.be.eql(12);
            message.msgbuf.should.be.eql(this.completeInvalidMessage);
            this.m.buf.length.should.be.eql(0);
        });

        it("error counter is raised on error", function() {
            var message = this.m.parseChar(this.completeInvalidMessage);
            this.m.total_receive_errors.should.equal(1);
            var message = this.m.parseChar(this.completeInvalidMessage);
            this.m.total_receive_errors.should.equal(2);
        });

        // TODO: there is a option in python: robust_parsing. Maybe we should port this as well.
        // If robust_parsing is off, the following should be tested:
        // - (maybe) not returning subsequent errors for prefix errors
        // - errors are thrown instead of caught inside

        // TODO: add tests for "try hard" parsing when implemented

    });

    describe("stream buffer accumulator", function() {

        it("increments total bytes received", function() {
            this.m.total_bytes_received.should.equal(0);
            var b = new Buffer.alloc(16);
            b.fill("h");
            this.m.pushBuffer(b);
            this.m.total_bytes_received.should.equal(16);
        });

        it("appends data to its local buffer", function() {
            this.m.buf.length.should.equal(0);
            var b = new Buffer.alloc(16);
            b.fill("h");
            this.m.pushBuffer(b);
            this.m.buf.should.eql(b); // eql = wiggly equality
        });
    });

    describe("prefix decoder", function() {
 
        it("consumes, unretrievably, the first byte of the buffer, if it's a bad prefix", function() {

            var b = new Buffer.from([1, 253]);
            this.m.pushBuffer(b);
            
            // eat the exception here.
            try {
                this.m.parsePrefix();
            } catch (e) {
                this.m.buf.length.should.equal(1);
                this.m.buf[0].should.equal(253);
            }
        
        });

        it("throws an exception if a malformed prefix is encountered", function() {

            var b = new Buffer.from([15, 253, 1, 7, 7]); // borked system status packet, invalid
            this.m.pushBuffer(b);
            var m = this.m;
            (function() { m.parsePrefix(); }).should.throw('Bad prefix (15)');

        });

    });

    describe("length decoder", function() {
        it("updates the expected length to the size of the expected full message", function() {
            this.m.expected_length.should.equal(10); // default, header size
            var b = new Buffer.from([253, 1, 1]); // packet length = 1
            this.m.pushBuffer(b);
            this.m.parseLength();
            this.m.expected_length.should.equal(13); // 1+12 bytes for the message header
        });
    });

    describe("payload decoder", function() {

        it("resets the expected length of the next packet to 10 (header)", function() {
            this.m.pushBuffer(this.heartbeatPayload);
            this.m.parseLength(); // expected length should now be 9 (message) + 8 bytes (header) = 17
            this.m.expected_length.should.equal(21);
            this.m.parsePayload();
            this.m.expected_length.should.equal(6);
        });

        it("submits a candidate message to the mavlink decode function", function() {
            
            var spy = sinon.spy(this.m, 'decode');
        
            this.m.pushBuffer(this.heartbeatPayload);
            this.m.parseLength();
            this.m.parsePayload();

            // could improve this to check the args more closely.
            // It'd be better but tricky because the type comparison doesn't quite work.
            spy.called.should.be.true;

        });

        // invalid data should return bad_data message
        it("parsePayload throws exception if a borked message is encountered", function() {
            var b = new Buffer.from([3, 0, 1, 2, 3, 4, 5]); // invalid message
            this.m.pushBuffer(b);
            var message;
            (function(){
                message = this.m.parsePayload();
            }).should.throw();
        });

        it("returns a valid mavlink packet if everything is OK", function() {
            this.m.pushBuffer(this.heartbeatPayload);
            this.m.parseLength();
            var message = this.m.parsePayload();
            message.should.be.an.instanceof(mavlink20.messages.heartbeat);
        });

        it("increments the total packets received if a good packet is decoded", function() {
            this.m.total_packets_received.should.equal(0);
            this.m.pushBuffer(this.heartbeatPayload);
            this.m.parseLength();
            var message = this.m.parsePayload();
            this.m.total_packets_received.should.equal(1);
        });

        

    });

});


describe("MAVLink 2.0 CRC-16/MCRF4XX Decoder", function() {

    beforeEach(function() {
        // Message header + payload, lacks initial MAVLink flag (FE) and CRC.
        this.heartbeatMessage = new Buffer.from([0x09, 0x03, 0xff , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x06 , 0x08 , 0x00 , 0x00 , 0x03]);

    });

    // This test matches the output directly taken by inspecting what the Python implementation
    // generated for the above packet.
    it('implements CRC-16/MCRF4XX function', function() {
            mavlink20.x25Crc(this.heartbeatMessage).should.equal(27276);
    });

    // Heartbeat crc_extra value is 50.
    it('can accumulate further bytes as needed (crc_extra)', function() {
            var crc = mavlink20.x25Crc(this.heartbeatMessage);
            crc = mavlink20.x25Crc([50], crc);
            crc.should.eql(23711)
    });

});
