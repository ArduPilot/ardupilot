# Build tool dependency policy

To ensure the broadest compatibility when building the benchmark library, but
still allow forward progress, we require any build tooling to be available for:

* Debian stable _and_
* The last two Ubuntu LTS releases

Currently, this means using build tool versions that are available for Ubuntu
Ubuntu 20.04 (Focal Fossa), Ubuntu 22.04 (Jammy Jellyfish) and Debian 11.4 (bullseye).

_Note, CI also runs ubuntu-18.04 to attempt best effort support for older versions._

## cmake
The current supported version is cmake 3.16.3 as of 2022-08-10.

* _3.10.2 (ubuntu 18.04)_
* 3.16.3 (ubuntu 20.04)
* 3.18.4 (debian 11.4)
* 3.22.1 (ubuntu 22.04)

