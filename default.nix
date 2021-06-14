{ ninja, boost17x
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
      sha256 = "0br7yak293djswbk0qw87qid5yn4ysa3ai6qh0036k19r5hb9jdm";
    };

    fmtSrc = fetchFromGitHub {
      owner = "fmtlib";
      repo = "fmt";
      rev = "cd4af11efc9c622896a3e4cb599fa28668ca3d05";
      sha256 = "17q2fdzakk5p0s3fx3724gs5k2b5ylp8f1d6j2m3wgvlfldx9k9a";
    };

    nativeBuildInputs = [ cmake ];

    buildInputs = [ llvmPackages.llvm z3 boost17x ];

    cmakeFlags = [
      "-DLLVM_DIR=${llvmPackages.libllvm.out}"
      "-DZ3_DIR=${z3.out}"
      "-DFMT_DIR=${fmtSrc}"
    ];
  }

