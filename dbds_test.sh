#!/bin/bash

# Enables address sanitizer
# export LD_PRELOAD=/opt/llvm/release_70/lib/clang/7.0.1/lib/linux/libclang_rt.asan-x86_64.so


PROJECT_DIR=$(pwd)

if [ "$1" == "stats" ]
then
    opt -debug-pass=Structure -S -o /dev/null -load ../build/lib/LLVMBlockDuplicator.so -O3 -simulator -simplifycfg -time-passes -stats < test/Transforms/Duplicate/memcpy_full_redundancies_O0.ll 2> out > /dev/null

    if [ "$2" == "grep" ]
    then
	cat out | grep Simulator
    fi
else
    mkdir ./test_result
    rm -f ./test_result/*

    cd ./test/Transforms/Duplicate
    for test_file in *.ll; do
	opt -S \
	    -o $PROJECT_DIR/test_result/dbds_$test_file \
	    -load $PROJECT_DIR/../build/lib/LLVMBlockDuplicator.so \
	    -O3 -simulator -simplifycfg -dot-dom -dot-cfg \
	    < $test_file \
	    > /dev/null
    done

    mkdir $PROJECT_DIR/graphs
    for dot_file in *.dot; do
	[ -f "$dot_file" ] || break
	dot -Tps $dot_file -o $dot_file.ps
	mv $dot_file.ps $PROJECT_DIR/graphs/
    done
fi
