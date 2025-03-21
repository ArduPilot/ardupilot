inputs@{ flake-utils, ... }:
flake-utils.lib.meld inputs [ ./ardupilot-firmware.nix ./ardupilot-sim.nix] 
