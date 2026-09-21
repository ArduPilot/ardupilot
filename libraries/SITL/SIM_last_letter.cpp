/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for the last_letter_lib flight dynamics library
*/

#include "SIM_config.h"

#if AP_SIM_LAST_LETTER_ENABLED

#include "SIM_last_letter.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// The default aircraft when --model is given without a ":<aircraft>" suffix.
static const char *default_model_name = "ardupilot_plane";

// The simulator process, so it can be reaped when SITL exits. SITL::Aircraft has
// no destructor to hook into, hence the file-static plus atexit().
static pid_t last_letter_pid = -1;

// A SITL reboot is an execv() of ourselves, which discards both the atexit()
// handler and the pid above while leaving the child running. execv() does keep
// the environment, so the pid is handed to the next incarnation through it, the
// same way SITL already passes SITL_WATCHDOG_RESET across a reboot.
static const char *child_pid_env = "LAST_LETTER_CHILD_PID";

/*
  Stop a last_letter child and wait for it to go away, so that it has released
  the UDP port before we return. Bounded, so a wedged child cannot hold up
  SITL indefinitely.
 */
static void stop_and_reap(pid_t pid, uint16_t timeout_ms)
{
    if (pid <= 0) {
        return;
    }
    kill(pid, SIGTERM);
    for (uint16_t i=0; i<timeout_ms/10; i++) {
        const pid_t ret = waitpid(pid, nullptr, WNOHANG);
        if (ret == pid) {
            return;
        }
        if (ret < 0 && errno == ECHILD) {
            // not ours to reap; watch it instead
            if (kill(pid, 0) < 0) {
                return;
            }
        }
        usleep(10000);
    }
    printf("last_letter: child %d will not exit, killing it\n", int(pid));
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
}

static void stop_last_letter(void)
{
    stop_and_reap(last_letter_pid, 1000);
    last_letter_pid = -1;
}

last_letter::last_letter(const char *_frame_str) :
    // We deviate from the JSON model convention by interpreting the frame
    // string as "last_letter:<model_name>" instead of "last_letter:<sim_ip>".
    // It is not used by JSON anyways, it is an API for its subclasses.
    // The constraint is that last_letter will be launched on localhost.
    JSON("last_letter"),
    sim_port(0),
    want_log(false)
{
    const char *colon = strchr(_frame_str, ':');
    const char *name = (colon != nullptr && colon[1] != '\0') ? colon+1 : default_model_name;
    strncpy(model_name, name, sizeof(model_name)-1);
    model_name[sizeof(model_name)-1] = '\0';

    printf("Starting SITL: last_letter, aircraft '%s'\n", model_name);
}

/*
  Create the socket and record the port the child must bind. port_out is the
  port ArduPilot sends servo packets to.
 */
void last_letter::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    JSON::set_interface_ports(address, port_in, port_out);
    sim_port = (uint16_t)port_out;
}

/*
  Options are a comma-separated list of flags:

      log     record an MCAP log, named after the date and time

  Unknown flags are reported and ignored.
 */
void last_letter::set_config(const char *config)
{
    Aircraft::set_config(config);

    // strtok_r() writes into its input, and config points into argv.
    char options[256];
    strncpy(options, config, sizeof(options)-1);
    options[sizeof(options)-1] = '\0';

    char *saveptr = nullptr;
    for (char *tok = strtok_r(options, ",", &saveptr);
         tok != nullptr;
         tok = strtok_r(nullptr, ",", &saveptr)) {
        if (strcmp(tok, "log") == 0) {
            want_log = true;
        } else {
            printf("last_letter: ignoring unknown config option '%s'\n", tok);
        }
    }
}

/*
  Start the last_letter child process. The base class calls this once the model
  is fully configured, to pass the config to the command line.
 */
void last_letter::launch_external_sim(void)
{
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%u", unsigned(sim_port));

    const char *bin = getenv("LAST_LETTER_SITL_BIN");
    if (bin == nullptr) {
        bin = "last_letter_ardupilot";
    }

    const char *args[8];
    uint8_t argc = 0;
    args[argc++] = bin;
    args[argc++] = "--model";
    args[argc++] = model_name;
    args[argc++] = "--port";
    args[argc++] = port_str;
    if (want_log) {
        args[argc++] = "--log";
    }
    args[argc++] = nullptr;

    // Stop the child from before a SITL reboot. Without this it keeps running
    // and holding the UDP port, the child forked below dies on bind, and the
    // pre-reboot simulator serves the new incarnation instead -- which works,
    // but leaves one simulator spanning several boots. Wait for it to be gone
    // before forking, or the new child races it for the port.
    const char *prev = getenv(child_pid_env);
    if (prev != nullptr) {
        stop_and_reap((pid_t)atoi(prev), 3000);
        unsetenv(child_pid_env);
    }

    pid_t child_pid = fork();
    if (child_pid == 0) {
        // in child
        // Close stdin and set /dev/null to sit on it.
        close(0);
        if (open("/dev/null", O_RDONLY|O_CLOEXEC) == -1) {
            perror("/dev/null");
        }
        // Close all the other descriptors from the child side, so the child
        // cannot keep ArduPilot's sockets or log files alive.
        // Closing up to 100 descriptors.
        for (uint8_t i=3; i<100; i++) {
            close(i);
        }
        execvp(bin, const_cast<char *const *>(args));
        // only reached if the exec failed
        perror(bin);
        _exit(1);
    }

    if (child_pid < 0) {
        perror("fork");
        return;
    }

    last_letter_pid = child_pid;

    // Hand the pid to the incarnation on the far side of the next reboot. Set
    // after the fork(), so the child does not inherit its own pid in its
    // environment.
    char pid_str[16];
    snprintf(pid_str, sizeof(pid_str), "%d", int(child_pid));
    setenv(child_pid_env, pid_str, 1);

    atexit(stop_last_letter);
}

} // namespace SITL

#endif  // AP_SIM_LAST_LETTER_ENABLED
