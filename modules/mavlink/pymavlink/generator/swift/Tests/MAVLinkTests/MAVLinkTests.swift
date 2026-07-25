//
//  MAVLinkTests.swift
//  MAVLinkTests
//
//  Created by Max Odnovolyk on 10/6/16.
//  Copyright © 2016 Build Apps. All rights reserved.
//

import Foundation
import XCTest
@testable import MAVLink

class MAVLinkTests: XCTestCase {
    override func setUp() {
        super.setUp()
        
        continueAfterFailure = false
    }
    
    // MARK: - Parsing tests
    
    func testParseDidParseMessageThatStartsRightAfterCorruptedMessageIdByte() {
        let corruptedByte = UInt8(0xC7)
        
        var data = Data(testHeartbeatData.prefix(upTo: 5))
        data.append(corruptedByte)
        data.append(testStatustextData)

        var callsCount = 0
        
        let delegate = Delegate(didParse: { message, _, _, _ in
            XCTAssert(message is Statustext, "Expects to get instance of Statustext from provided data")
            
            callsCount += 1
        })
        
        let mavLink = MAVLink()
        mavLink.delegate = delegate
        mavLink.parse(data: data, channel: 0)
        
        XCTAssert(callsCount == 1, "MAVLink instance should parse exactly one message from provided data")
    }
    
    func testParseDidParseMessageThatStartsRightAfterCorruptedCRCByte() {
        let corruptedByte = UInt8(0x00)
        
        var data = testHeartbeatData
        data.removeLast(2)
        data.append(corruptedByte)
        data.append(testStatustextData)
        
        var callsCount = 0
        
        let delegate = Delegate(didParse: { message, _, _, _ in
            XCTAssert(message is Statustext, "Expects to get instance of Statustext from provided data")
            
            callsCount += 1
        })
        
        let mavLink = MAVLink()
        mavLink.delegate = delegate
        mavLink.parse(data: data, channel: 0)
        
        XCTAssert(callsCount == 1, "MAVLink instance should parse exactly one message from provided data")
    }
    
    // MARK: - Dispatching tests
    
    func testDispatchDidPutProperMessageId() {
        var callsCount = 0
        
        let delegate = Delegate(didFinalize: { _, _, data, _, _ in
            XCTAssert(data[5] == Heartbeat.id, "Sixth byte of MAVLink packet should be message id (in this specific case \(Heartbeat.id))")
            
            callsCount += 1
        })
        
        let mavLink = MAVLink()
        mavLink.delegate = delegate
        try! mavLink.dispatch(message: testHeartbeatMessage, systemId: 0, componentId: 0, channel: 0)
        
        XCTAssert(callsCount == 1, "MAVLink instance should return exactly one finalized packet from provided message")
    }
    
    func testDispatchDidPutProperSystemId() {
        var callsCount = 0
        let systemId = UInt8(0xFF)
        
        let delegate = Delegate(didFinalize: { _, _, data, _, _ in
            XCTAssert(data[3] == systemId, "Fourth byte of MAVLink packet should be system id (\(systemId))")
            
            callsCount += 1
        })
        
        let mavLink = MAVLink()
        mavLink.delegate = delegate
        try! mavLink.dispatch(message: testHeartbeatMessage, systemId: systemId, componentId: 0, channel: 0)
        
        XCTAssert(callsCount == 1, "MAVLink instance should return exactly one finalized packet from provided message")
    }
    
    func testDispatchDidPutProperComponentId() {
        var callsCount = 0
        let componentId = UInt8(0xFF)
        
        let delegate = Delegate(didFinalize: { _, _, data, _, _ in
            XCTAssert(data[4] == componentId, "Fifth byte of generated MAVLink packet should contain component id (\(componentId))")
            
            callsCount += 1
        })
        
        let mavLink = MAVLink()
        mavLink.delegate = delegate
        try! mavLink.dispatch(message: testHeartbeatMessage, systemId: 0, componentId: componentId, channel: 0)
        
        XCTAssert(callsCount == 1, "MAVLink instance should return exactly one finalized packet from provided message")
    }
    
    func testDispatchDidPutProperCRC() {
        var callsCount = 0
        
        let delegate = Delegate(didFinalize: { [unowned self] _, _, data, _, _ in
            let expectedData = self.testHeartbeatData
            XCTAssert(data == expectedData, "Test message`s bytes should match expected constant test data (including CRC)")
            
            callsCount += 1
        })
        
        let mavLink = MAVLink()
        mavLink.delegate = delegate
        try! mavLink.dispatch(message: testHeartbeatMessage, systemId: 0xFF, componentId: 0, channel: 0)
        
        XCTAssert(callsCount == 1, "MAVLink instance should return exactly one finalized packet from provided message")
    }
    
    func testDispatchRethrowsDataExtensionsErrors() {
        let mavLink = MAVLink()
        let message = Statustext(severity: MAVSeverity.notice, text:"💩")
        
        XCTAssertThrowsError(try mavLink.dispatch(message: message, systemId: 0, componentId: 0, channel: 0)) { error in
            switch error {
            case let PackError.invalidStringEncoding(offset, string) where offset == 1 && string == "💩":
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
}
