//
//  MAVLinkTests.swift
//  MAVLinkTests
//
//  Created by Max Odnovolyk on 9/28/16.
//  Copyright © 2016 Build Apps. All rights reserved.
//

import Foundation
import XCTest
@testable import MAVLink

class DataExtensionsTests: XCTestCase {
    
    // MARK: - Test number<T: MAVLinkNumber>(at offset: Data.Index, byteOrder: CFByteOrder) throws -> T
    
    func testGetNumberCannotGetTooLargeNumber() {
        let data = Data(count: 4)
        
        let test: () throws -> Void = {
            let _: Int32 = try data.number(at: 1)
        }
        
        XCTAssertThrowsError(try test()) { error in
            switch error {
            case let ParseError.valueSizeOutOfBounds(offset, size, upperBound) where offset + size > upperBound:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testGetNumberDidGetNumberAtOffset() {
        let data = Data(bytes: [0x00, 0x00, 0x01, 0x02, 0x03, 0x04])
        let expectedNumber: UInt32 = 0x01_02_03_04
        let receivedNumber: UInt32 = try! data.number(at: 2, byteOrder: .bigEndian)
        
        XCTAssert(receivedNumber == expectedNumber, "Method should return number value available at specified offset in data")
    }
    
    func testGetNumberByteOrderIsSwappedOnLittleEndianSystem() {
        let memoryBytes: [UInt8] = [0x04, 0x03, 0x02, 0x01]
        let expectedNumber: UInt32 = 0x01_02_03_04
        
        let data = Data(bytes: memoryBytes)
        let receivedNumber: UInt32 = try! data.number(at: 0, byteOrder: .littleEndian)
        
        XCTAssert(receivedNumber == expectedNumber, "Method expects swapped bytes in memory on little-endian system (most significant digit byte at the end)")
    }
    
    func testGetNumberByteOrderRemainsSameOnBigEndianSystem() {
        let memoryBytes: [UInt8] = [0x01, 0x02, 0x03, 0x04]
        let expectedNumber: UInt32 = 0x01_02_03_04
        
        let data = Data(bytes: memoryBytes)
        let receivedNumber: UInt32 = try! data.number(at: 0, byteOrder: .bigEndian)
        
        XCTAssert(receivedNumber == expectedNumber, "Method expects less significant digit byte at the end of value's memory chunk")
    }
    
    // MARK: Test array<T: MAVLinkNumber>(at offset: Data.Index, capacity: Int) throws -> [T]
    
    func testGetArrayCannotAccessBytesOutOfDataUpperBound() {
        let data = Data(count: 4)
        
        let test: () throws -> Void = {
            let _: [UInt8] = try data.array(at: 1, capacity: 4)
        }
        
        XCTAssertThrowsError(try test()) { error in
            switch error {
            case let ParseError.valueSizeOutOfBounds(offset, size, upperBound) where offset + size > upperBound:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testGetArrayDidGetArrayAtOffset() {
        let data = Data(bytes: [0x00, 0x00, 0x01, 0x02, 0x03, 0x00])
        let expectedArray: [UInt8] = [0x01, 0x02, 0x03]
        let receivedArray: [UInt8] = try! data.array(at: 2, capacity: 3)
        
        XCTAssert(receivedArray == expectedArray, "Expect array values at specified offset in data")
    }
    
    func testGetArrayDidGetRequestedNumberOfItems() {
        let data = Data(count: 4)
        let itemsCountToGet = 3
        
        let receivedArray: [UInt8] = try! data.array(at: 1, capacity: itemsCountToGet)
        XCTAssert(receivedArray.count == itemsCountToGet, "Expect requested capacity of array to be equal to received array's count")
    }
    
    // MARK: Test string(at offset: Data.Index, length: Int) throws -> String
    
    func testGetStringCannotAccessBytesOutOfDataUpperBound() {
        let data = Data(count: 4)
        
        XCTAssertThrowsError(try data.string(at: 1, length: 4)) { error in
            switch error {
            case let ParseError.valueSizeOutOfBounds(offset, size, upperBound) where offset + size > upperBound:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }

    func testGetStringCannotReadNonASCIISEncodedString() {
        // Check that invalidStringEncoding is thrown in case of non ASCII compatible strings.
        // It looks like all strings are ASCII compatible for String(bytes: encoding:) method.
    }
    
    func testGetStringDidReadASCIIEncodedStringAtSpecifiedOffset() {
        let data = Data(bytes: [0x60, 0x61, 0x62, 0x63, 0x64, 0x65]) // "`abcde"
        let string = try! data.string(at: 3, length: 3)
        
        XCTAssert(string == "cde", "Expect string at offset 3 (cde)")
    }
    
    func testGetStringDidReadEmptyNullTerminatedASCIIEncodedString() {
        let data = Data(bytes: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]) // "\0\0\0\0\0\0"
        let string = try! data.string(at: 1, length: 4)
        
        XCTAssert(string == "", "Expect to get empty string from zeroed data bytes")
    }
    
    func testGetStringDidReadNullTerminatedASCIIEncodedString() {
        let data = Data(bytes: [0x60, 0x61, 0x62, 0x0, 0x64, 0x65]) // "`ab\0de"
        let string = try! data.string(at: 1, length: 4)
        
        XCTAssert(string == "ab", "Expect cut off by null-termination string")
    }
    
    func testGetStringDidReadExactlyLengthSizedASCIIEncodedStringWithoutNullTermination() {
        let data = Data(bytes: [0x60, 0x61, 0x62, 0x63, 0x64, 0x65]) // "`abcde"
        let string = try! data.string(at: 1, length: 3)
        
        XCTAssert(string == "abc", "Expect cut off by length limit string")
    }
    
    // MARK: Test enumeration<T: Enumeration>(at offset: Data.Index) throws -> T where T.RawValue: MAVLinkNumber
    
    func testGetEnumerationCannotInitWithUnknownValue() {
        let data = Data(bytes: [UInt8(ADSBAltitudeType.enumEnd)])
        
        let test: () throws -> Void = {
            let _: ADSBAltitudeType = try data.enumeration(at: 0)
        }
        
        XCTAssertThrowsError(try test()) { error in
            switch error {
            case ParseEnumError<ADSBAltitudeType>.unknownValue:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testGetEnumerationDidGetProperCaseAtOffset() {
        let data = Data(bytes: [0x00, 0x00, ADSBAltitudeType.geometric.rawValue])
        let adsbAltitudeType: ADSBAltitudeType = try! data.enumeration(at: 2)
        
        XCTAssert(adsbAltitudeType == .geometric, "Expected adsbAltitudeType is .geometric")
    }
    
    // MARK: - Test set<T: MAVLinkNumber>(_ number: T, at offset: Data.Index, byteOrder: CFByteOrder) throws -> Void
    
    func testSetNumberCannotSetTooLargeNumber() {
        var data = Data(count: 4)
        
        XCTAssertThrowsError(try data.set(Int32(1), at: 1)) { error in
            switch error {
            case let PackError.valueSizeOutOfBounds(offset, size, upperBound) where offset + size > upperBound:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testSetNumberDidSetNumberAtOffset() {
        var data = Data(count: 6)
        let number: UInt32 = 0x01_02_03_04
        let expectedData = Data(bytes: [0x00, 0x00, 0x01, 0x02, 0x03, 0x04])

        try! data.set(number, at: 2, byteOrder: .bigEndian)
        
        XCTAssert(data == expectedData, "Method should set number at specified offset in data memory")
    }
    
    func testSetNumberByteOrderIsSwappedOnLittleEndianSystem() {
        var data = Data(count: 4)
        let number: UInt32 = 0x01_02_03_04
        let expectedData = Data(bytes: [0x04, 0x03, 0x02, 0x01])
        
        try! data.set(number, at: 0, byteOrder: .littleEndian)
        
        XCTAssert(data == expectedData, "Method should write swapped bytes in memory on little-endian system (most significant digit byte at the end)")
    }
    
    func testSetNumberByteOrderRemainsSameOnBigEndianSystem() {
        var data = Data(count: 4)
        let number: UInt32 = 0x01_02_03_04
        let expectedData = Data(bytes: [0x01, 0x02, 0x03, 0x04])
        
        try! data.set(number, at: 0, byteOrder: .bigEndian)
        
        XCTAssert(data == expectedData, "Method should write less significant digit byte at the end of data's memory chunk")
    }
    
    // MARK: Test set<T: MAVLinkNumber>(_ array: [T], at offset: Data.Index, capacity: Int) throws
    
    func testSetArrayCannotSetArrayWithLessCapacityThanArrayCount() {
        var data = Data(count: 6)
        let array = Array<UInt8>(repeating: 0, count: 4)
        
        XCTAssertThrowsError(try data.set(array, at: 0, capacity: 3)) { error in
            switch error {
            case let PackError.invalidValueLength(offset, providedValueLength, allowedLength) where offset == 0 && providedValueLength > allowedLength:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testSetArrayCannotSetValuesOutOfDataUpperBound() {
        var data = Data(count: 10)
        let array = Array<UInt64>(repeating: 0, count: 2)
        
        XCTAssertThrowsError(try data.set(array, at: 0, capacity: 16)) { error in
            switch error {
            case let PackError.valueSizeOutOfBounds(offset, size, upperBound) where offset == 0 && offset + size > upperBound:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testSetArrayDidSetValuesAtOffset() {
        var data = Data(count: 6)
        let array: [UInt8] = [0x01, 0x02, 0x03, 0x04]
        let expectedData = Data(bytes: [0x00, 0x01, 0x02, 0x03, 0x04, 0x00])
        
        try! data.set(array, at: 1, capacity: 4)
        
        XCTAssert(data == expectedData, "Method should write array values at specified offset")
    }
    
    // MARK: Test set(_ string: String, at offset: Data.Index, length: Int) throws
    
    func testSetStringCannotSetNonASCIIString() {
        var data = Data(count: 10)
        
        XCTAssertThrowsError(try data.set("💩", at: 0, length: 4)) { error in
            switch error {
            case let PackError.invalidStringEncoding(offset, string) where offset == 0 && string == "💩":
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testSetStringCanntSetTooLongString() {
        var data = Data(count: 5)
        
        XCTAssertThrowsError(try data.set("abcdef", at: 0, length: 6)) { error in
            switch error {
            case let PackError.valueSizeOutOfBounds(offset, size, upperBound) where offset == 0 && offset + size > upperBound:
                break
            default:
                XCTFail("Unexpected error thrown")
            }
        }
    }
    
    func testSetStringDidSetNullTerminatedStringAtOffset() {
        var data = Data(bytes: Array<UInt8>(repeating: 0xFF, count: 5))
        let expectedData = Data(bytes: [0xFF, 0x61, 0x62, 0x63, 0x00])
        
        try! data.set("abc", at: 1, length: 4)
        
        XCTAssert(data == expectedData, "Method should write ASCII string bytes at specified offset with null byte at the end")
    }
    
    func testSetStringDidSetEmptyString() {
        var data = Data(bytes: Array<UInt8>(repeating: 0xFF, count: 5))
        let expectedData = Data(bytes: [0xFF, 0xFF, 0x00, 0xFF, 0xFF])
        
        try! data.set("", at: 2, length: 3)
        
        XCTAssert(data == expectedData, "Method should accept empty string as valid ASCII encoded string and add null termination")
    }
    
    // MARK: Test set<T: Enumeration>(_ enumeration: T, at offset: Data.Index) throws
    
    func testSetEnumerationDidSetProperRawValueAtOffset() {
        var data = Data(bytes: Array<UInt8>(repeating: 0x00, count: 4))
        let expectedData = Data(bytes: [0x00, 0x00, 0x00, MAVVTOLState.fw.rawValue])
        
        try! data.set(MAVVTOLState.fw, at: 3)
        
        XCTAssert(data == expectedData, "Method should set appropriate case raw value at specified offset")
    }
    
    // MARK: - Test write-read methods calls return consistent values
    
    func testGetNumberDidGetSameValuePreviouslySetWithSetNumberCall() {
        var data = Data(count: 10)
        let offset = 1
        let number = UInt64(0x01_02_03_04_05_06_07_08)
        
        try! data.set(number, at: offset)
        let receivedNumber: UInt64 = try! data.number(at: offset)
        
        XCTAssert(receivedNumber == number)
    }
    
    func testGetArrayDidGetSameValuePreviouslySetWithSetArrayCall() {
        var data = Data(count: 10)
        let offset = 1
        let array: [UInt8] = [0x00, 0x01, 0x02, 0x03, 0x04]
        
        try! data.set(array, at: offset, capacity: 5)
        let receivedArray: [UInt8] = try! data.array(at: offset, capacity: 5)
        
        XCTAssert(receivedArray == array)
    }
    
    func testGetStringDidGetSameValuePreviouslySetWithSetStringCall() {
        var data = Data(count: 10)
        let offset = 1
        let string = "test"
        
        try! data.set(string, at: offset, length: 9)
        let receivedString = try! data.string(at: offset, length: 9)
        
        XCTAssert(receivedString == string)
    }
    
    func testGetEnumerationDidGetSameValuePreviouslySetWithSetEnumerationCall() {
        var data = Data(count: 10)
        let offset = 1
        let mavvtolState = MAVVTOLState.fw
        
        try! data.set(mavvtolState, at: offset)
        let receivedMavvtolState: MAVVTOLState = try! data.enumeration(at: offset)
        
        XCTAssert(receivedMavvtolState == mavvtolState)
    }
}
