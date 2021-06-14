{ boost17x, gmp
, cmake, fetchFromGitHub, z3
, pkgs, stdenv
, llvmPackages
, debugVersion ? false
}:

  stdenv.mkDerivation rec {
    pname = "horn2vmt";
    version = "1.0";
    
    src = fetchFromGitHub {
      owner = "dbueno";
      repo = "${pname}";
      rev = "dev";
      sha256 = "1i4bpvican8v6l5xw25m52n3fg45srwbaqa32z9izvw8v4naviw5";
    };

    fmt = stdenv.mkDerivation {
      pname = "fmtlib";
      version = "cd4af11efc9c622896a3e4cb599fa28668ca3d05";
      src = fetchFromGitHub {
        owner = "fmtlib";
        repo = "fmt";
        rev = "cd4af11efc9c622896a3e4cb599fa28668ca3d05";
        sha256 = "17q2fdzakk5p0s3fx3724gs5k2b5ylp8f1d6j2m3wgvlfldx9k9a";
      };
      buildInputs = [ cmake ];
      cmakeFlags = [ "-DCMAKE_POSITION_INDEPENDENT_CODE=TRUE" ];
      doCheck = true;
      checkTarget = "test";
    };

    nativeBuildInputs = [ cmake ];

    buildInputs = [ llvmPackages.llvm z3 boost17x gmp ];

    VERBOSE="1";

    cmakeFlags = [
      "-DLLVM_DIR=${llvmPackages.libllvm.dev}/lib/cmake/llvm"
      "-DZ3_DIR=${z3.lib}"
      "-Dfmt_DIR=${fmt}/lib/cmake/fmt"
    ];
  }

