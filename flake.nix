{
  inputs = {
    self.submodules = true;
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    micro_ros_cmake.url = "path:./external/micro_ros_cmake";
  };
  outputs = { self, nixpkgs, flake-utils, micro_ros_cmake }:
    flake-utils.lib.eachDefaultSystem (system:
      let pkgs = (import nixpkgs { system = system; });
      in {
        devShells = {
          buildenv = pkgs.mkShellNoCC {
            packages = (with pkgs; [
              # Our cross compilation toolchain
              gcc-arm-embedded-13

              # Other build tools
              cmake
              gcc
              git
              ninja
              llvmPackages_20.clang-tools # for clang-tidy
            ]) ++ micro_ros_cmake.devShells.${system}.default.nativeBuildInputs;
          };

          default = pkgs.mkShellNoCC {
            packages = (with pkgs; [
              # GDB from gcc-arm-embedded is broken so we include this one
              pkgsCross.arm-embedded.buildPackages.gdb

              # For ST-Link development
              openocd
            ]) ++ self.devShells.${system}.buildenv.nativeBuildInputs;
          };

          cubemx = pkgs.mkShellNoCC {
            nativeBuildInputs = with pkgs; [ stm32cubemx ];
          };
        };
        formatter = pkgs.nixfmt-classic;
      });
}
