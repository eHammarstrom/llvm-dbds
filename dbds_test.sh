#!/bin/bash

# Enables address sanitizer
# export LD_PRELOAD=/opt/llvm/release_70/lib/clang/7.0.1/lib/linux/libclang_rt.asan-x86_64.so


PROJECT_DIR=$(pwd)
TEST_DIR=$PROJECT_DIR/test/Transforms/Duplicate

if [ "$1" == "stats" ]
then
    opt -debug-pass=Structure -S -o /dev/null -load ../build/lib/LLVMBlockDuplicator.so -O3 -simulator -simplifycfg -time-passes -stats < test/Transforms/Duplicate/memcpy_full_redundancies_O0.ll 2> out > /dev/null

    if [ "$2" == "grep" ]
    then
	cat out | grep Simulator
    fi
else
    # generate LLVM IR from test_programs
    cd $PROJECT_DIR/test_programs
    for test_prog in *.c; do
	test_no_ext=${test_prog%%.*}
	clang -S -O0 -Xclang -disable-O0-optnone -emit-llvm \
	      -fno-inline-functions \
	      $test_prog -o $TEST_DIR/$test_no_ext.ll
    done

    cd $PROJECT_DIR

    mkdir test_result
    rm -f test_result/*

    # optimize all test IR
    cd $TEST_DIR

    for test_file in *.ll; do
	# test_no_ext=${test_file%%.*}
	# OUT_FILE=$PROJECT_DIR/test_result/out_$test_no_ext.txt

	opt -S \
	    -o $PROJECT_DIR/test_result/dbds_$test_file \
	    -load $PROJECT_DIR/../build/lib/LLVMBlockDuplicator.so \
	    -O3 -simulator -simplifycfg -dot-dom -dot-cfg \
	    < $test_file #&> OUT_FILE # redirect stdin and stderr to OUT_FILE
    done

    # produce graphs for all tests
    mkdir $PROJECT_DIR/graphs
    for dot_file in *.dot; do
	[ -f "$dot_file" ] || break
	dot -Tps $dot_file -o $dot_file.ps
	mv $dot_file.ps $PROJECT_DIR/graphs/
	rm $dot_file
    done

    # compile and run tests
    cd $PROJECT_DIR/test_result

    echo "10" > input
    for test_file in *.ll; do
	test_no_ext=${test_file%%.*}
	llc $test_no_ext.ll
	clang $test_no_ext.s -o $test_no_ext.e
	./$test_no_ext.e < input > $test_no_ext.output
    done

fi
