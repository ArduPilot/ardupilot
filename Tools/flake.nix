{
  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs = inputs@{ flake-utils, ... }:
    flake-utils.lib.meld inputs [
      ./nix
    ];
}