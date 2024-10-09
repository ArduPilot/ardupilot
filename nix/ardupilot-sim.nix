{ self, flake-utils, nixpkgs, ... }:
  let supported-systems = with flake-utils.lib.system; [
    x86_64-linux
    aarch64-linux
    aarch64-darwin
  ];
in flake-utils.lib.eachSystem supported-systems (system:
    let
        pkgs = import nixpkgs { inherit system; };
        inherit (pkgs) lib;
        python = (pkgs.python39.withPackages
        (ps: with ps; [ setuptools future pexpect empy ]));
  in {
    packages = flake-utils.lib.flattenTree {
      ardusim = pkgs.stdenv.mkDerivation {
        name = "ardusim";
        src = builtins.fetchGit {
          url = "https://github.com/ArduPilot/ardupilot.git";
          rev = "df2be63e21217b626be53c7c9e98f37410b12126";
          ref = "Rover-4.4";
          submodules = true;
        };
        buildInputs = [ python pkgs.git pkgs.rsync pkgs.gcc ];

        nativeBuildInputs = [ pkgs.wafHook ];
        patches = [
          ./patches/ardusim_patches/runnable_status.patch
          ./patches/ardusim_patches/remove_gcc_Werror.patch
          ./patches/ardusim_patches/fake_git_rev.patch
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
