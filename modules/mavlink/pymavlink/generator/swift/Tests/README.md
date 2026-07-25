## Swift MAVLink library test project ##

`MAVLinkTests` target covers `DataExtensions` with unit tests (conversion between raw data and typed values). Also it contains some integration tests for `MAVLink` class.

To quickly generate MAVLink classes and enums for Swift you can use following script:

```bash
./ardugen.sh
```
It will generate Swift files for `ardupilotmega.xml` definitions into `MAVLink/MAVLink/Swift` folder and C headers for the same xml file into `MAVLinkTest/MAVLink/C` folder. C headers are used to check Swift library compatibility with C/C++ implementations.

Please keep in mind that you will probably need to re-add generated MAVLink swift files to the project as complete list of files depends on specific declarations in xml file. C headers will be compiled as Clang module, so there is no need to add them into Xcode project.

Test suite consists of:
- `DataExtensionsTests.swift` contains unit tests that cover raw-to-typed values transformations and vice versa (with informative erroring)
- `MAVLinkTests.swift` covers some edge cases of parsing state machine 
- `CompareWithCImpTests.swift` tests which compare output of Swift library with C on the same raw data input (attached tlog file)

Test tlog file was downloaded from [DroneKit LA testdata](https://github.com/dronekit/dronekit-la-testdata) repository.
