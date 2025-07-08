# ESP32 's3' Debugging Guide

The ESP32 's3' variant has some built-in debug capabilities for using 'gdb' but has a few quirks.

This document is written for a "dev" machine running Ubuntu or similar Linux, and makes no attempt to describe how to install any prerequisites.

## Setup

1. Open **four terminal windows**. You will use all four. Make "ardupilot" the current directory in all four.

2. In any of the windows, run the following commands to install the ESP-IDF:

    ```bash
    cd modules/esp_idf
    ./install.sh
    cd ../..
    ```

3. In **all the windows**, run the following command to export the ESP-IDF path:

    ```bash
    source modules/esp_idf/export.sh
    ```

## Usage

We'll be using the 4 windows for the following tasks:

- **Window A** - Recompiling the firmware and flashing the target device:

    ```bash
    ./waf configure --debug --board=esp32s3empty
    ./waf copter --upload
    ```

- **Window B** - Running openocd to connect to the target device to support debugging it:

    ```bash
    cp ./Tools/debug/gdb-openocd-esp32.init .gdbinit
    openocd -f board/esp32s3-builtin.cfg
    ```

- **Window C** - Running gdb to debug the target device:

    ```bash
    xtensa-esp32s3-elf-gdb ./build/esp32s3empty/esp-idf_build/ardupilot.elf
    ```

- **Window D** - Running "mavproxy" or a similar tool to monitor the console output of the target device and see what it's "doing":

    ```bash
    mavproxy.py --setup --master /dev/ttyACM0
    ```

# Debugging Details for Each Window

## Window A - Recompiling/Flashing/Uploading the Firmware

In this window, run the commands to recompile, flash, and upload the firmware. This is referred to as doing a `--upload`. 

If you forget the `--upload`, you can and will be re-running it multiple times, which is okay. If the `--upload` part fails, or gets halfway through and gives up, then you need to put the device into "BOOTLOADER" mode and try again. There are "Boot" and "Reset" buttons on nearly all of these boards for a reason. Press and hold the boot button while short-pressing reset, then release the boot button, and try the `--upload` again. If you have `mavproxy` running in "Window D", you'll see information from the bootloader at this point, but that's not essential.

## Window B - Running OpenOCD

In this window, run the above `openocd` command to connect to the target device. 

`OpenOCD` needs to be running before you start `gdb`, and it needs to be running the whole time you are using `gdb`. In my experience, you can leave `openocd` running and just ignore it, it will be fine, but if you stop it, you will need to restart it before you can use `gdb` again. I found that I can even leave it running while uploading with `--upload` in window A, but if you have problems, try stopping `openocd` and restarting it after the upload. You can type `Ctrl-C` to stop it.

## Window C - Running GDB

In this window, run the `gdb` command to connect to the target device. 

You will see a `(gdb)` prompt, and you can type "help" to see the commands `gdb` understands. You can type "run" to start the firmware running [from the beginning], or 'c' to continue running from where you left off, and "quit" to exit `gdb`. If the prompt has "gone away", it means the target IS RUNNING. You can type `Ctrl-C` to stop it, and then type "run" to start it again. You can set breakpoints, and single-step, and examine variables, and so on. You can also type "monitor reset halt" to stop the firmware running, and "monitor resume" to start it running again. You can also type "monitor reset" to reset the device, and "monitor shutdown" to stop `openocd` running. ESP32 specific `gdb`-stuff: 'thb' is short for 'temporary-hardware-breakpoint', and THERE ARE ONLY TWO OF THESE. Power GDB users, sorry. See [ESP32 API Guides](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/tips-and-quirks.html) for more details. Type 'c' to continue running from where you left off. Type 'q' at the `(gdb)` prompt to exit `gdb`. Power-users also pay attention to the contents of the "./Tools/debug/gdb-openocd-esp32.init" that we copied to `.gdbinit` for this to work. Type 'i threads' to see the threads that are running, and the core/s that they are running on. Type 'bt' to see the backtrace of the current thread. Type 'thread 2' to switch to thread 2, and 'thread 1' to switch back to thread 1. (do this before a 'bt' to see the backtrace of the other thread) Type 'thread apply all bt full' to see a backtrace of all threads. Other things you can type at the `(gdb)` prompt can be found with Google.

## Window D - Running Mavproxy

In this window, run the above-mentioned `mavproxy` command to monitor the console output of the target device. 

You should look to get hints about what the board is doing in this screen. It shows you bootup messages being emitted from `ardupilot` if you start or restart it *immediately* after the `--upload` command finishes in window A. If you wait a few seconds, you will miss the bootup messages, and you will need to stop and restart the firmware to see them again. If it says this... then it's telling you that the BOOTLOADER is running, not the main firmware:

```plaintext
rst:0x15 (USB_UART_CHIP_RESET),boot:0x0 (DOWNLOAD(USB/UART0))
Saved PC:0x40048ee5
waiting for download
```
IIf you get the above 'bootloader' info, it's because you pressed the 'reset/boot' buttons as per above on the board, and you need to **stop** mavproxy in 'Window D' and do a `--upload` in window A to get the main firmware running again.

## Tips

Although the device will almost always show up as `/dev/ttyACM0` when plugged in, occasionally, if you are rebooting it a lot, it might reconnect as `/dev/ttyACM1` instead of `/dev/ttyACM0` (i.e., one-1 instead of zero-0). You can either change the above commands to match the new name, or better still, just unplug the USB cable to the device for a little more than 5 seconds, and then plug it back in, and it will be `/dev/ttyACM0` again. This is a quirk of Linux/USB when many disconnect/reconnect events occur in a short time, and 5 seconds is the magic number to reset the "counter" that Linux uses to assign the device name.

## Ordering

Edit some source-code in your text editor if you want, then:

1. Stop mavproxy in window D - with `ctrl-c`.
2. Run the `--upload` in window A.
3. Restart mavproxy in window D [hit up-arrow, then enter].
4. Run openocd in window B if not already running.
5. Run gdb in window C - if it was still running from before, you **must** stop and then restart it for it to have the newest info.