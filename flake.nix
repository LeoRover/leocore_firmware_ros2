{
  inputs = {
    self.submodules = true;
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    micro_ros_cmake.url = "path:./external/micro_ros_cmake";
  };
  outputs = { self, nixpkgs, flake-utils, micro_ros_cmake }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = (import nixpkgs { system = system; });

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
            PYTHONPATH = micro_ros_cmake.devShells.${system}.default.PYTHONPATH;

            packages =
              micro_ros_cmake.devShells.${system}.default.nativeBuildInputs
              ++ (with pkgs; [
                # Our cross compilation toolchain
                gcc-arm-embedded-13

                # Other build tools
                cmake
                gcc
                git
                ninja
                clang-tools_18 # for clang-tidy
              ]);
          };

          default = pkgs.mkShellNoCC {
            PYTHONPATH = "${self.devShells.${system}.buildenv.PYTHONPATH}";

            packages = self.devShells.${system}.buildenv.nativeBuildInputs
              ++ (with pkgs; [
                # GDB from gcc-arm-embedded is broken so we include this one
                pkgsCross.arm-embedded.buildPackages.gdb

                # For ST-Link development
                openocd-stm
              ]);
          };

          cubemx = pkgs.mkShellNoCC {
            nativeBuildInputs = with pkgs; [ stm32cubemx ];
          };
        };
        formatter = pkgs.nixfmt-classic;
      });
}
