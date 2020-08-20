# Horn2VMT

This is a proof of concept implementation of the paper, "Horn2VMT: Translating
Horn Reachability into Transition Systems," available online at
http://eptcs.web.cse.unsw.edu.au/content.cgi?VPTHCVS2020#EPTCS320.13.

This tool converts linear Horn clauses into a transition system format, VMT,
described at the [nuXmv
site](https://es-static.fbk.eu/tools/nuxmv/index.php?n=Languages.VMT). VMT is
SMT-LIBv2 compatible.

## Dependencies
+ Z3 - 4.8.7 is tested but earlier should work
+ LLVM - 5 & 10 tested but in between should work
+ CMake 3.14
+ Boost 1.67
+ fmtlib - in a submodule of this repo

## install recipe

    git submodule init
    git submodule update
    cmake -DZ3_DIR=/opt/z3-4.8.7/ \
        -DLLVM_DIR=/opt/clang+llvm-10.0.0-x86_64-apple-darwin/ ..

## usage

    horn2vmt file.smt2 > file.vmt
