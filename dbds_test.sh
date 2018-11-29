#!/bin/bash

#if [[ "$1" -eq "" ]]; then
    #echo "Must pass an LLVM IR .ll test file."
    #exit 1
#fi

rm -f test.ll
opt -debug-pass=Structure -S -o test.ll -load ../build/lib/LLVMBlockDuplicator.so -O3 -simulator -simplifycfg -dot-dom -dot-cfg < test/Transforms/Duplicate/memcpy_full_redundancies_O0.ll > /dev/null

for dot_file in *.dot; do
    [ -f "$dot_file" ] || break

    dot -Tps $dot_file -o $dot_file.ps
done
