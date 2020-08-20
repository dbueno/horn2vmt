# Horn2VMT

## Dependencies
Z3 - 4.8.7 is tested but earlier should work
LLVM - 5 & 10 tested but in between should work
CMake 3.14
Boost 1.67
fmtlib - in a submodule of this repo

## install recipe

    git submodule init
    git submodule update
    cmake -DZ3_DIR=/opt/z3-4.8.7/ \
        -DLLVM_DIR=/opt/clang+llvm-10.0.0-x86_64-apple-darwin/ ..

## usage

    horn2vmt file.smt2 > file.vmt
