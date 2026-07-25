//
//  Testdata.swift
//  MAVLink
//
//  Created by Max Odnovolyk on 10/31/16.
//  Copyright © 2016 Build Apps. All rights reserved.
//

import Foundation
import XCTest
@testable import MAVLink

extension XCTestCase {
    
    /// Loads data from test tlog file
    var testTlogData: Data {
        func bundledPath() -> URL? {
            let bundle = Bundle(for: MAVLinkTests.self)
            return bundle.url(forResource: "flight", withExtension: "tlog")
        }
        
        func relativePath() -> URL {
            let packagePath = URL(fileURLWithPath: FileManager.default.currentDirectoryPath, isDirectory: true).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
            return packagePath.appendingPathComponent("Tests/MAVLinkTests/Testdata/flight.tlog")
        }
        
        let logPath: URL!
        
        #if os(macOS) || os(iOS) || os(watchOS) || os(tvOS)
            if let path = bundledPath() {
                logPath = path
            } else {
                logPath = relativePath()
            }
        #elseif os(Linux)
            logPath = relativePath()
        #else
            XCTFail("Unsupported target OS")
        #endif
        
        return try! Data(contentsOf: logPath)
    }
    
    /**
     ATTITUDE: The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
     Fields:
         timeBootMs = 72062 : Timestamp (milliseconds since system boot)
         roll = -0.0695389 : Roll angle (rad, -pi..+pi)
         pitch = 0.0272282 : Pitch angle (rad, -pi..+pi)
         yaw = 1.11595 : Yaw angle (rad, -pi..+pi)
         rollspeed = -0.0208449 : Roll angular speed (rad/s)
         pitchspeed = 0.00913008 : Pitch angular speed (rad/s)
         yawspeed = 0.00166465 : Yaw angular speed (rad/s)
    */
    var testAttitudeData: Data {
        return Data(bytes: [0xFE, 0x1C, 0x00, 0x01, 0x01, 0x1E, 0x7E, 0x19, 0x01, 0x00, 0x64, 0x6A, 0x8E, 0xBD, 0xB2, 0x0D, 0xDF, 0x3C, 0x5B, 0xD7, 0x8E, 0x3F, 0xEA, 0xC2, 0xAA, 0xBC, 0x56, 0x96, 0x15, 0x3C, 0x51, 0x30, 0xDA, 0x3A, 0x12, 0xAB])
    }
    
    var testHeartbeatMessage: Heartbeat {
        return Heartbeat(type: 6, autopilot: 8, baseMode: 0, customMode: 0, systemStatus: 0, mavlinkVersion: 3)
    }
    
    var testHeartbeatData: Data {
        return Data(bytes: [0xFE,0x09,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x08,0x00,0x00,0x03,0xA1,0xDF])
    }
    
    var testStatustextData: Data {
        return Data(bytes: [0xFE,0x33,0x00,0x01,0x01,0xFD,0x01,0x41,0x50,0x4D,0x3A,0x43,0x6F,0x70,0x74,0x65,0x72,0x20,0x56,0x33,0x2E,0x34,0x2D,0x64,0x65,0x76,0x20,0x28,0x62,0x64,0x64,0x64,0x66,0x61,0x65,0x35,0x29,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x90,0x07])
    }
}
