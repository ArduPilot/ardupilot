# Security Policy

ArduPilot controls real vehicles, so we take security reports seriously. This
page says how to report an issue, where ArduPilot's security boundary sits and
what counts as a vulnerability. The full background is on the
[security landing page](https://ardupilot.org/dev/docs/security-landing-page.html).

## Reporting an issue

Open an issue in the [ArduPilot issue tracker](https://github.com/ArduPilot/ardupilot/issues).
You can also post a link to it in the `code_review` channel of the
[ArduPilot Discord server](https://ardupilot.org/discord) so the developers see
it quickly.

Issues in other ArduPilot projects belong in their own repositories, for
example [pymavlink](https://github.com/ArduPilot/pymavlink),
[MAVProxy](https://github.com/ArduPilot/MAVProxy) and
[Mission Planner](https://github.com/ArduPilot/MissionPlanner). Issues in
third-party ground control stations, such as QGroundControl, go to those
projects.

A useful report contains:

- the vehicle firmware and version, or the git commit, and the board
- the interface used (MAVLink over which link, DroneCAN, a serial protocol,
  Ethernet, SD card, ...)
- steps to reproduce, ideally in SITL, with the parameters that differ from
  defaults
- a `.bin` log, crash dump or backtrace when there is one
- which documented protection the issue defeats, if any (see below)

If you used a fuzzer, a static analyser or an AI assistant to find the issue,
say so, and confirm the problem reproduces on current `master` before
reporting. Reports that do not reproduce will be closed.

## The security boundary

ArduPilot's security boundary extends up to, but does not include, the physical
communication interface: the radio link, network cable, USB cable and so on.
Securing the command and control (C2) link and any network on the vehicle is
the responsibility of the user or integrator.

Every interface is trusted by default. In particular, the ground control
station is trusted: anyone who can send MAVLink to the vehicle over an
unsigned link can command it, change its parameters and upload firmware. That
is the design, not a vulnerability.

## What is a vulnerability

An issue is a vulnerability when it defeats one of the protections ArduPilot
documents for users who choose to enable them:

- [MAVLink2 signing](https://ardupilot.org/dev/docs/common-MAVLink2-signing.html):
  once signing is enabled on a link, unsigned or wrongly signed packets must be
  ignored. A packet that is not correctly signed but still changes the
  vehicle's behaviour is in scope.
- [Secure firmware](https://ardupilot.org/dev/docs/secure-firmware.html):
  a board running a signed bootloader must refuse to load and run firmware
  that is not signed with one of its accepted keys.
- [Remote ID](https://ardupilot.org/dev/docs/common-remoteid.html) tamper
  protection: an autopilot built with it must refuse non-compliant firmware
  sent from a ground station or a normal uploader. Issues in the ArduRemoteID
  module firmware belong in the
  [ArduRemoteID repository](https://github.com/ArduPilot/ArduRemoteID).

## Robustness bugs

Corrupted or malformed data received on any interface must not halt or crash
the software. A crash, hang or lockup caused by malformed data is a
correctness bug, whichever interface it arrives on, and we want to hear about
it through the same issue tracker. It is not a security vulnerability unless it
also defeats one of the protections above, for example a malformed but
unsigned packet that crashes a vehicle with signing enabled.

## Not vulnerabilities

These come up often and are not treated as security issues:

- Crashes or unexpected behaviour found by changing parameters in SITL, or by
  sending invalid MAVLink from a trusted ground station. Report a crash from
  malformed data as a robustness bug as described above.
- Commanding, reconfiguring or reflashing a vehicle over an unsigned link, or
  over an interface you already have access to.
- Getting round
  [parameter lockdown](https://ardupilot.org/dev/docs/common-parameter-lockdown.html).
  It is a Lua script that guards against accidental changes, and a ground
  station can stop it.
- Attacks that need physical access to the vehicle, such as cutting wires,
  breaking propellers or interfering with sensors. Pre-arm checks catch some
  of these, but they cannot protect against all of them.
- The security of the radio link, network or companion computer, which is
  outside ArduPilot's boundary.

## Supported versions

Fixes are made on `master` first. Whether a fix is also backported to a stable
release is decided case by case by the development team. Please check that an
issue still occurs on the latest stable release or on `master` before
reporting it.
