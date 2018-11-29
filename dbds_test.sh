#!/bin/bash

rm -f test.ll

if [ "$1" == "stats" ]
then
  opt  -S -o test.ll -load ../build/lib/LLVMBlockDuplicator.so -O3 -simulator -simplifycfg -time-passes -stats < test/Transforms/Duplicate/memcpy_full_redundancies_O0.ll 2> out > /dev/null
  cat out | grep Simulator
else
  opt -debug-pass=Structure -S -o test.ll -load ../build/lib/LLVMBlockDuplicator.so -O3 -simulator -simplifycfg -dot-dom -dot-cfg < test/Transforms/Duplicate/memcpy_full_redundancies_O0.ll > /dev/null

  for dot_file in *.dot; do
    [ -f "$dot_file" ] || break

    dot -Tps $dot_file -o $dot_file.ps
  done
fi

