{
  description = "QMI8658 6-axis IMU driver for embedded-hal (sync + async, I2C + SPI)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      rust-overlay,
    }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forEachSystem = nixpkgs.lib.genAttrs systems;
      cargoToml = builtins.fromTOML (builtins.readFile ./Cargo.toml);
      version = cargoToml.package.version;

      pkgsFor =
        system:
        import nixpkgs {
          inherit system;
          overlays = [ rust-overlay.overlays.default ];
        };

      toolchainFor =
        pkgs:
        pkgs.rust-bin.stable.latest.default.override {
          extensions = [
            "clippy"
            "rustfmt"
            "rust-analyzer"
            "rust-src"
          ];
          targets = [
            "thumbv7em-none-eabihf"
            "thumbv7m-none-eabi"
            "riscv32imc-unknown-none-elf"
          ];
        };

      # The only sandbox-able gate: host tests with all features.
      # Cross-target checks run via `just ci` inside the devShell.
      hostTestPackageFor =
        system:
        let
          pkgs = pkgsFor system;
          rustToolchain = toolchainFor pkgs;
          rustPlatform = pkgs.makeRustPlatform {
            cargo = rustToolchain;
            rustc = rustToolchain;
          };
        in
        rustPlatform.buildRustPackage {
          pname = "qmi8658-host-tests";
          inherit version;
          src = self;
          cargoLock.lockFile = ./Cargo.lock;
          cargoBuildFlags = [ "--all-features" ];
          cargoTestFlags = [ "--all-features" ];
          # Library-only check package; nothing to install.
          doInstall = false;
          meta = {
            description = "Host-side tests for the qmi8658 driver crate";
            license = with pkgs.lib.licenses; [
              mit
              asl20
            ];
          };
        };
    in
    {
      packages = forEachSystem (system: {
        default = hostTestPackageFor system;
      });

      checks = forEachSystem (system: {
        host-tests = hostTestPackageFor system;
      });

      devShells = forEachSystem (
        system:
        let
          pkgs = pkgsFor system;
        in
        {
          default = pkgs.mkShell {
            name = "qmi8658-dev";
            packages = [
              (toolchainFor pkgs)
              pkgs.just
              pkgs.cargo-hack
              pkgs.cargo-readme
              pkgs.nixfmt
              pkgs.git
            ];
          };
        }
      );

      formatter = forEachSystem (system: (pkgsFor system).nixfmt);
    };
}
