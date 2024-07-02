# Lua REPL

This script implements an interactive REPL (read-evaluate-print-loop) for the
Lua scripting engine accessible over serial, with line editing, history, and
output formatting.

### Basic Usage
* Configure a serial port (e.g. `SERIALn_PROTOCOL`) to protocol 28 (Scripting).
    * By default the first such port is used; this can be adjusted in the script
      text.
    * `SERIAL6` is the alternate USB serial port on Cube Orange, and convenient
      for bench testing.
* Remove propellers (safety first!) and load the script onto the autopilot.
* Connect a terminal emulator to the port and enter Lua statements/expressions
  at the `> ` prompt, then press Enter to execute. Results and errors will be
  printed back.
    * On Linux a convenient command is e.g.
      `minicom -w -D /dev/ttyACM1 -b 115200`.
    * A `>> ` prompt indicates that more input is needed to complete the
      statement.
    * You can use the arrow keys to edit the current and previous inputs.
    * Press ESC twice to clear the input and any incomplete statement then
      return to an empty prompt.

### SITL Setup
* Configure SITL `SERIALn_PROTOCOL` as above.
* Start SITL with a command like `Tools/autotest/sim_vehicle.py -A --serialN=tcp:9995:wait`
* Connect a terminal emulator to localhost TCP port 9995
    * On Linux a convenient command is `stty -icanon -echo -icrnl && netcat localhost 9995`.
    * Note that you must execute `reset` to turn echo back on once disconnected.
    * Scripting must be restarted after a TCP reconnection.

### Notes
* Statements like `local x = 3` create a variable which immediately goes out of
  scope once evaluated. Names must be global to survive to the next prompt.
* There is currently no facility for installing periodic update callbacks.
* While theoretically impossible to crash the autopilot software, certain
  scripting APIs can cause damage to you or your vehicle if used improperly.
  Use extreme caution with an armed vehicle, and simulate first!
* The script expects Enter to include `\r` (`\n` is ignored), prints `\r\n` for
  a new line, and uses ANSI cursor control codes for line editing and history.
  Check your terminal configuration if Enter doesn't work or you see garbage
  characters.
* Evaluating complex statements or printing complex results can cause an
  `exceeded time limit` error and stop the script. Increasing the vehicle's
  `SCR_VM_I_COUNT` parameter reduces the chance of this occurring.
