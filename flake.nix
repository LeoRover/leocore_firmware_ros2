{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    poetry2nix = {
      url = "github:nix-community/poetry2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs = { self, nixpkgs, flake-utils, poetry2nix }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = (import nixpkgs { system = system; });

        inherit (poetry2nix.lib.mkPoetry2Nix { inherit pkgs; }) mkPoetryEnv;

        # poetry-env = mkPoetryEnv {
        #   projectDir = ./.;
        #   python = pkgs.python312;
        # };

        openocd-stm = pkgs.openocd.overrideAttrs (old: {
          src = pkgs.fetchFromGitHub {
            owner = "STMicroelectronics";
            repo = "OpenOCD";
            rev = "26301c4";
            sha256 = "sha256-7REQi9pcT6pn8yiAMpQpRQ+0ouMQelcciMAHyUonkVA=";
          };
          nativeBuildInputs = with pkgs; [
            autoconf
            automake
            pkg-config
            libtool
            texinfo
            which
          ];
          preConfigure = ''
            ./bootstrap nosubmodule
          '';
        });

      in {
        devShells = {
          buildenv = pkgs.mkShellNoCC {
            # PYTHONPATH = "${poetry-env}/lib/python3.12/site-packages";

            packages = with pkgs; [
            #   poetry-env

              # Our cross compilation toolchain
              gcc-arm-embedded-13

              # Other build tools
              cmake
              gcc
              git
              ninja
              clang-tools_18 # for clang-tidy
            ];
          };

          default = pkgs.mkShellNoCC {
            # PYTHONPATH = "${self.devShells.${system}.buildenv.PYTHONPATH}";

            packages = (with pkgs; [
              # GDB from gcc-arm-embedded is broken so we include this one
              pkgsCross.arm-embedded.buildPackages.gdb

              # For ST-Link development
              openocd-stm
            ]) ++ self.devShells.${system}.buildenv.nativeBuildInputs;
          };

          cubemx = pkgs.mkShellNoCC {
            nativeBuildInputs = with pkgs; [ stm32cubemx ];
          };
        };
        formatter = pkgs.nixfmt-classic;
      });
}
