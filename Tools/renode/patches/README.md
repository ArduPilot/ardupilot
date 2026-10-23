# Local Renode patches

`usbip-device-state.patch` supports firmware-driven USB disconnect/reconnect.
`usbip-unlink.patch` applies on top of it and adds USB/IP URB cancellation and
ordered endpoint reads, needed when GDB closes and reopens the CDC port.

`cortex-m-debug-monitor.patch` adds guest DebugMonitor enable, BKPT status,
MON_STEP, FPB instruction matching, and corrects SysTick wakeup when the timer
was disabled before sleep. These are independent of Renode's host GDB server.
The FPB register frontend is `../peripherals/cpu/AP_CortexM_FPB.cs`.

`cortex-m-usb-debug-extensions.patch` applies after the DebugMonitor patch. It
corrects fixed exception priorities and masked-fault escalation, and adds guest
DWT memory-access notifications and post-instruction debug events. The existing
`AP_DWT.cs` frontend exposes four comparators when these CPU APIs are present;
unpatched Renode retains its cycle counter with no advertised comparators.

Apply these from the Renode Infrastructure directory (including its tlib
checkout), then rebuild both native ARM-M and managed components. The patches
were developed against Infrastructure `add012af003a0f620d3da52828262676f374d121`
and tlib `d50c868f33fb42954d85b8d06ce6f7d5bac23dd4`.

```sh
patch -p1 < /path/to/ardupilot/Tools/renode/patches/usbip-device-state.patch
patch -p1 < /path/to/ardupilot/Tools/renode/patches/usbip-unlink.patch
patch -p1 < /path/to/ardupilot/Tools/renode/patches/cortex-m-debug-monitor.patch
patch -p1 < /path/to/ardupilot/Tools/renode/patches/cortex-m-usb-debug-extensions.patch
```

Skip patches already applied by your Renode build setup. From the Renode root:

```sh
./build.sh --skip-fetch --external-lib-arch arm-m.le
dotnet build Renode_NET.sln -c Release -p:NET=true --no-restore -m:1
```

Run the CPU regression tests from the ArduPilot root:

```sh
python3 -m pytest -q Tools/renode/tests/test_usb_debug_cpu.py
```

These CPU tests require the patched CPU APIs. They report a skip with an
explanation on an unpatched Renode, including the package currently pinned by
CI. Once those APIs are present, all behavior assertions run; a failing
DebugMonitor, fault-delivery, DWT or FPB implementation is not treated as a missing dependency.

For a firmware USB debugging session, add the FPB overlay to a normal `--usb`
launch. Create `fpb.repl` containing:

```text
fpb: Miscellaneous.AP_CortexM_FPB @ sysbus 0xE0002000
    cpu: cpu
```

Then launch a matching USB-debug-enabled firmware ELF:

```sh
Tools/renode/run.py CubeOrangePlus --usb --elf /path/to/arduplane \
    --exec 'include @/path/to/ardupilot/Tools/renode/peripherals/cpu/AP_CortexM_FPB.cs' \
    --exec 'machine LoadPlatformDescription @/path/to/fpb.repl'
```

Attach with `usbip_attach.py`, then use `Tools/debug/gdb_usb.py` against the
emulated device's second CDC interface. These patches cover the monitor
operations used by the USB debugger; they are not a complete CoreSight model
and do not replace testing on an STM32H7.

For the opt-in firmware tests, launch a fresh CubeOrangePlus Plane instance as
above, with USB/IP on port 23451 and the monitor on port 23465. Wait for the
launcher's initialization/reset commands to finish before connecting. These
tests use USB/IP directly in userspace, so do not attach a kernel USB/IP client
to the same instance. The ELF must include debug symbols and `--enable-USB-debug`.

```sh
USB_DEBUG_ELF=/path/to/arduplane USB_DEBUG_USBIP_PORT=23451 \
    USB_DEBUG_MONITOR_PORT=23465 \
    python3 -m pytest -q Tools/renode/tests/test_usb_debug_firmware.py
```

To test the initial setup breakpoint as well, build with
`--enable-USB-debug-startup-wait`, launch that ELF in a fresh instance, and add
`USB_DEBUG_STARTUP_WAIT=1` to the test environment. The suite exercises RAM
assignment, current and suspended registers, floating-point execution,
write/read/access watchpoints, RAM breakpoint restoration and step-over,
fault repair, MPU write-fault recovery, and a real GDB `AP_HAL::millis()` call.
It also tests priority-zero USB interruption of a busy thread with BASEPRI
masking all nonzero priorities, interruption of a self-pending lower-priority
ISR, and concurrent application SysTick operation. With the monitor port set,
SysTick is enabled before attachment and its configuration is checked again
after detach, along with restoration of the original vector table and USB IRQ
priority. It checks a GCS heartbeat after detach. Without `USB_DEBUG_ELF` these tests skip;
without `USB_DEBUG_MONITOR_PORT` only the MPU fault-injection test skips.

The CubeOrangePlus validation used assertions enabled and a Renode hook returning
false from `sdcard_init()` to bypass an unrelated SD crash-dump initialization
assertion. This exercises USB/debugger behavior without SD-card initialization;
it does not validate that path or hardware cache coherency.
