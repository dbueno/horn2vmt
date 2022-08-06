{
  description = "A very basic flake";
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    flake-compat = {
      url = "github:edolstra/flake-compat";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, flake-compat }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        horn2vmt = pkgs.callPackage ./default.nix {};
      in {
        packages = {
          inherit horn2vmt;
        };
        defaultPackage = horn2vmt;
        devShell =
          pkgs.mkShell {
            inputsFrom = [ horn2vmt ];
            packages = with pkgs; [ creduce ];
            inherit (horn2vmt) cmakeFlags;
            hardeningDisable = [ "all" ];
            CXXFLAGS = "-Werror";
          };
      });
}
