# Custom Fast Task for ArduPilot

This library provides a framework for integrating a custom FAST_TASK into ArduPilot. The FAST_TASK runs at the main loop frequency (400Hz for Copter), allowing users to execute high-frequency logic during flight.

## Purpose

The custom FAST task enables developers to insert their own initialization and update logic into the ArduPilot main loop. This is useful for tasks that require high-frequency execution, such as real-time monitoring, control or custom data logging.

## Usage

### Option 1: Use the Provided Empty Class

An empty class `AC_Custom_FastTask_Empty` is provided as a template. Users can insert their custom logic into the `init()` and `update()` methods of this class:

- `init()` is called during Copter initialization.
- `update()` is called at 400Hz during the main loop.
Within these methods, users can access singleton instances such as AHRS, the attitude controller, and other system components to read/write and monitor required values.

## Example
An example implementation is provided in the `examples` folder to demonstrate how inserrt code to the AC_Custom_FastTask_Empty class.

### Option 2: Create a Custom Class

Users can define their own class that inherits from `AC_Custom_FastTask_Base`. To register this class, implement a factory class `CustomFT_SubFactory` with a static method `createInstance()` that returns a handle to your custom class.
