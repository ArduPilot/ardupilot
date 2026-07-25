# Libcanard Drivers

This directory contains implementations of platform-specific components for Libcanard.
Each sub-directory contains (or should contain) a dedicated README file with driver specific documentation.

## Porting Guide

Existing drivers should be used as a reference for implementation of one's own custom drivers.
Libcanard does not interact with the underlying platform drivers directly,
but does so via the application.
Therefore, there is no need in a dedicated porting guide.
This is unlike Libuavcan, which is more complex and does have a well-defined interface between
the library and the platform.

![libuavcan vs libcanard arch](libuavcan_vs_libcanard_arch.png)
