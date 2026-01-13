{ self, flake-utils, nixpkgs, ... }:
  let supported-systems = with flake-utils.lib.system; [
    x86_64-linux
    aarch64-linux
    aarch64-darwin
  ];
in flake-utils.lib.eachSystem supported-systems (system:
    let
        pkgs = import nixpkgs { inherit system; };

    nixpkgs_22_05 = import (builtins.fetchTarball {
      url    = "https://github.com/NixOS/nixpkgs/archive/refs/tags/22.05.tar.gz";
      sha256 = "0d643wp3l77hv2pmg2fi7vyxn4rwy0iyr8djcw1h5x72315ck9ik";
    }) { inherit system; };

    python = nixpkgs_22_05.python310.withPackages (ps: with ps; [
      ps.setuptools
      ps.future
      ps.pexpect
      ps.empy     
    ]);

  in {
    packages = flake-utils.lib.flattenTree {
      ardusim = pkgs.stdenv.mkDerivation {
        name = "ardusim";
        src = builtins.fetchGit {
          url = "https://github.com/ArduPilot/ardupilot.git";
          rev = "9cc2b9d59a54868be94e53bd45fc4279e33f6b9f";
          ref = "Rover-4.6";
          submodules = true;
        };
        buildInputs = [ python pkgs.git pkgs.rsync pkgs.gcc ];

        nativeBuildInputs = [ pkgs.wafHook ];
        patches = [
          ./patches/remove_gcc_Werror.patch
          ./patches/fake_git_rev.patch
        ];
        wafPath = "modules/waf/waf-light";
        wafConfigureFlags = [ "--board sitl" ];
        wafFlags = [ "rover" ];
        postInstall =
          "	mv Tools/autotest/default_params/rover.parm $out/.;\n	mv Tools/mavproxy_modules/ $out/.;\n";
        system = builtins.currentSystem;
      };
    };
  })