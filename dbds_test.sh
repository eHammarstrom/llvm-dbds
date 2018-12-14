#!/bin/bash

# Enables address sanitizer
# export LD_PRELOAD=/opt/llvm/release_70/lib/clang/7.0.1/lib/linux/libclang_rt.asan-x86_64.so


PROJECT_DIR=$(pwd)
TEST_DIR=$PROJECT_DIR/test/Transforms/Duplicate

# use built opt because of our lib mods
OPT=$PROJECT_DIR/../build/bin/opt
CLANG=$PROJECT_DIR/../build/bin/clang
LLC=$PROJECT_DIR/../build/bin/llc

# generate LLVM IR from test_programs
rm -f test/Transforms/Duplicate/*

cd $PROJECT_DIR/test_programs

for test_prog in *.c; do
    test_no_ext=${test_prog%%.*}
    $CLANG -S -O0 -Xclang -disable-O0-optnone -emit-llvm \
	  -fno-inline-functions \
	  $test_prog -o $TEST_DIR/$test_no_ext.ll
done

cd $PROJECT_DIR


if [ "$1" == "stats" ]
then
    # print stats for all tests
    cd $TEST_DIR

    for test_file in *.ll; do
	# test_no_ext=${test_file%%.*}
	# OUT_FILE=$PROJECT_DIR/test_result/out_$test_no_ext.txt

	if [ "$2" == "grep" ]
	then
	    $OPT -debug-pass=Structure -S -o /dev/null \
		 -O3 -time-passes -stats \
		 < $test_file 2> out > /dev/null

		 # -load $PROJECT_DIR/../build/lib/LLVMBlockDuplicator.so \

	    cat out | grep $3
	else
	    $OPT -debug-pass=Structure -S -o /dev/null \
		 -O3 \
		 -time-passes -stats \
		 < $test_file > /dev/null

		 # -load $PROJECT_DIR/../build/lib/LLVMBlockDuplicator.so \
	fi
    done

    rm -f out

else
    mkdir test_result
    rm -f test_result/*

    # optimize all test IR
    cd $TEST_DIR

    mkdir $PROJECT_DIR/graphs

    for test_file in *.ll; do
	test_no_ext=${test_file%%.*}
	# OUT_FILE=$PROJECT_DIR/test_result/out_$test_no_ext.txt

	echo ""
	echo "*********** OPTIMIZING $test_no_ext ***********"
	echo ""

	$OPT -S \
	     -o $PROJECT_DIR/test_result/$test_file \
	     -O3 -dot-dom -dot-cfg \
	     -debug-only simulator \
	     < $test_file #&> OUT_FILE # redirect stdin and stderr to OUT_FILE

	     # -load $PROJECT_DIR/../build/lib/LLVMBlockDuplicator.so \

	# produce AFTER graphs
	for dot_file in *.dot; do
	    [ -f "$dot_file" ] || break
	    dot -Tps $dot_file -o AFTER.$test_no_ext$dot_file.ps
	    mv AFTER.$test_no_ext$dot_file.ps $PROJECT_DIR/graphs/
	    rm $dot_file
	done

	$OPT -S \
	     -o /dev/null \
	     -dot-dom -dot-cfg \
	     < $test_file #&> OUT_FILE # redirect stdin and stderr to OUT_FILE

	# produce BEFORE graphs
	for dot_file in *.dot; do
	    [ -f "$dot_file" ] || break
	    dot -Tps $dot_file -o BEFORE.$test_no_ext$dot_file.ps
	    mv BEFORE.$test_no_ext$dot_file.ps $PROJECT_DIR/graphs/
	    rm $dot_file
	done
    done



    # compile and run tests
    cd $PROJECT_DIR/test_result

    echo "10" > input10
    echo "0" > input0
    echo "1023456789ABCDEFG" >> input10
    echo "1023456789ABCDEFG" >> input0
    for test_file in *.ll; do
	test_no_ext=${test_file%%.*}
	echo ""
	echo "*********** TESTING: $test_no_ext ***********"
	echo ""
	$LLC $test_no_ext.ll
	$CLANG $test_no_ext.s -o $test_no_ext.e
	$CLANG -O3 $PROJECT_DIR/test_programs/$test_no_ext.c
	./$test_no_ext.e < input10 > $test_no_ext.output10
	./a.out < input10 > $test_no_ext.correct10
	diff $test_no_ext.output10 $test_no_ext.correct10
	./$test_no_ext.e < input0 > $test_no_ext.output0
	./a.out < input0 > $test_no_ext.correct0
	diff $test_no_ext.output0 $test_no_ext.correct0
    done

fi
