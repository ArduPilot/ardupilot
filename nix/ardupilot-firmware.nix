{ self, flake-utils, nixpkgs, ... }:
let supported-systems = with flake-utils.lib.system; [ x86_64-linux ];
in flake-utils.lib.eachSystem supported-systems (system:
  let
    pkgs = import nixpkgs { inherit system; };
    python = (pkgs.python39.withPackages
      (ps: with ps; [ setuptools future pexpect empy ]));
  in {
    packages.ardupilot = pkgs.pkgsCross.arm-embedded.stdenv.mkDerivation {
      name = "ardupilot";
      src = builtins.fetchGit {
        url = "https://github.com/ArduPilot/ardupilot.git";
        rev = "df2be63e21217b626be53c7c9e98f37410b12126";
        ref = "Rover-4.4";
        submodules = true;
      };
      hardeningDisable = [ "all" ];
      buildInputs = [ python pkgs.git pkgs.rsync pkgs.gcc ];
      nativeBuildInputs = [
        python
        pkgs.git
        pkgs.rsync
        pkgs.gcc-arm-embedded
        pkgs.gcc_multi
        pkgs.wafHook
      ];
      patches = [
        ./patches/ardusim_patches/runnable_status.patch
        ./patches/ardusim_patches/remove_gcc_Werror.patch
        ./patches/ardusim_patches/fake_git_rev.patch
      ];
      wafPath = "modules/waf/waf-light";
      wafConfigureFlags = [ "--board CubeOrangePlus" ];
      wafFlags = [ "rover" ];
      postInstall = ''
          cp -r build/CubeOrangePlus/bin/* $out/bin/.
          '';
      system = builtins.currentSystem;
    };
    packages.ardupilot-fw-flasher = pkgs.stdenv.mkDerivation {
      name = "ardupilot-fw-flasher";
      src = builtins.fetchGit {
        url = "https://github.com/ArduPilot/ardupilot.git";
        rev = "df2be63e21217b626be53c7c9e98f37410b12126";
        ref = "Rover-4.4";
        submodules = true;
      };
      dontBuild = true;

      buildInputs = [ python ];

      installPhase = ''
        mkdir -p $out/bin
        cp Tools/scripts/uploader.py $out/bin/ardupilot-fw-flasher
      '';
    };
  })