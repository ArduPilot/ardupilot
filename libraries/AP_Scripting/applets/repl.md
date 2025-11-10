# Lua REPL

This script implements an interactive REPL (read-evaluate-print-loop) for the
Lua scripting engine accessible over serial, with line editing, history, and
output formatting.

The script can also act as a client for QGroundControl's MAVLink Console
functionality (within the Analyze view), subject to limitations detailed
below.

### Basic Usage
* Configure a serial port (e.g. `SERIALn_PROTOCOL`) to protocol 28 (Scripting).
    * By default the first such port is used; this can be adjusted in the script
      text.
    * `SERIAL6` is the alternate USB serial port on Cube Orange, and convenient
      for bench testing. CAN and network serial ports will also work.
* Load the `repl.lua` script onto the autopilot.
* Connect a terminal emulator to the port and enter Lua statements/expressions
  at the `> ` prompt, then press Enter to execute. Results and errors will be
  printed back.
    * A `>> ` prompt indicates that more input is needed to complete the
      statement.
    * You can use the arrow keys to edit the current and previous inputs.
    * Press ESC twice to clear the input and any incomplete statement then
      return to an empty prompt.

### Autopilot Connection
* On Linux a convenient command is e.g. `minicom -w -D /dev/ttyACM1 -b 115200`,
  assuming you have the minicom terminal emulator installed.
* Any terminal emulator on any platform should work; see notes below about
  control codes and other configuration.

### SITL Connection
* Start SITL with a command like `Tools/autotest/sim_vehicle.py -A --serialN=tcp:9995:wait` to allow connection to the selected serial port.
* Connect a terminal emulator to localhost TCP port 9995
    * On Linux a convenient command is `stty -icanon -echo -icrnl && netcat localhost 9995`.
    * Note that you must execute `reset` to turn echo back on once disconnected.
    * Scripting must be restarted after a TCP reconnection.

### MAVLink Connection
* Requires at least Ardupilot 4.6.
* Set the port in the script text to `nil` to enable.
* In addition to `repl.lua`, copy the `mavport.lua` file and `MAVLink` directory
  from `AP_Scripting/modules` to `APM/SCRIPTS/MODULES` on your autopilot.
* The ESC key is not supported; cause a syntax error to reset the prompt.
* The experience over a radio link might be sub-par due to lack of any sort of
  packet loss tracking or retransmission.

### Notes and Limitations
* Statements like `local x = 3` create a variable which immediately goes out of
  scope once evaluated. Names must be global to survive to the next prompt.
* There is currently no facility for installing periodic update callbacks.
* While theoretically impossible to accidentally crash the autopilot software,
  certain scripting APIs can cause damage to you or your vehicle if used
  improperly. Use extreme caution with an armed vehicle!
* The script expects Enter to be `\r`, `\r\n`, or `\n`. It prints `\r\n` for a
  new line, and uses ANSI cursor control codes for line editing and history.
  Check your terminal configuration if Enter doesn't work or you see garbage
  characters. Lines longer than the terminal width likely won't edit properly.
* Evaluating complex statements or printing complex results can cause an
  `exceeded time limit` error, stopping the script and losing variables and
  history. Increasing the vehicle's `SCR_VM_I_COUNT` parameter reduces the
  chance of this occurring.
