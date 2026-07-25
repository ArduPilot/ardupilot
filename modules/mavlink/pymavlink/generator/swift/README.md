## MAVLink Communication in Swift

This folder contains Swift files used to generate MAVLink Swift Library.

Generated code allows to decode and encode MAVLink Messages into/from specially auto-generated Swift structs and enums according to specific MAVLink XML definition file. It was built from the ground up in Swift utilizing type safety, generics, throwing and other language features to make library more reliable.

Current Swift implementation supports only first version of MAVLink protocol.

## Features

- [x] Swift 3.0 Implementation
- [x] Message Decoding and Encoding Support
- [x] Payload Length and CRC Extra Checks
- [x] Strongly Typed Messages and Enums
- [x] Precise Parsing Error Reporting
- [x] Rich Metadata Information
- [x] Inline Documentation

## Requirements

- Swift 3.0.1

## Generating Platform-specific Source Files

To generate MAVLink Swift Library for specific flight controller or specific firmware version you need to follow [this](https://github.com/ArduPilot/mavlink#generating-language-specific-source-files) instructions.

## Installation

MAVLink Swift Library doesn't contain any external dependencies.

#### Manually
1. Download all files from `Sources` subfolder
2. Drop downloaded files into your project
3. Congratulations!  

#### Swift Package Manager (From Pre-generated Repository)
You can use Swift Package Manager to install MAVLink Swift Library by adding it to your `Package.swift` file:

```swift
import PackageDescription

let package = Package(
    name: "GCS",
    dependencies: [.Package(url: "https://github.com/modnovolyk/MAVLinkSwift", majorVersion: 0)]
)
```

```
$ swift build
```

Tested with swift build --version: 3.0.2 (swiftpm-11750)

## Usage Example

```swift
import Foundation
import MAVLink

let data = Data(bytes: [0xFE, 0x1C, 0x00, 0x01, 0x01, 0x1E, 0x7E, 0x19, 0x01, 0x00, 0x64, 0x6A, 0x8E, 0xBD, 0xB2, 0x0D, 0xDF, 0x3C, 0x5B, 0xD7, 0x8E, 0x3F, 0xEA, 0xC2, 0xAA, 0xBC, 0x56, 0x96, 0x15, 0x3C, 0x51, 0x30, 0xDA, 0x3A, 0x12, 0xAB])

let mavLink = MAVLink()

mavLink.parse(data: data, channel: 0) { message, _ in
    print(message.debugDescription)
}
```

Output:

```
ATTITUDE: The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
Fields:
	timeBootMs = 72062 : Timestamp (milliseconds since system boot)
	roll = -0.0695389 : Roll angle (rad, -pi..+pi)
	pitch = 0.0272282 : Pitch angle (rad, -pi..+pi)
	yaw = 1.11595 : Yaw angle (rad, -pi..+pi)
	rollspeed = -0.0208449 : Roll angular speed (rad/s)
	pitchspeed = 0.00913008 : Pitch angular speed (rad/s)
	yawspeed = 0.00166465 : Yaw angular speed (rad/s)
```

## Testing

`Tests` folder includes XCode project that you should run to tests generated Swift code. Before running tests you need to execute `./ardugen.sh` script to generate Swift and C code into right subfolders. C code is used to compare output results and ensure that Swift implementation interprets data in the same way. `ardugen.sh` uses `ardupilotmega.xml` definition file from `Tests/MAVLinkTests/Testdata`.

Unit tests cover conversion of raw data to/from Swift types. Also there are several integration tests that compare work of Swift code and C implementation on real-world tlog file.