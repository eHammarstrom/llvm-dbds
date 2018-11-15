#!/bin/bash

opt -load ../build/lib/LLVMBlockDuplicator.so -simulator < test/Transforms/GVN/2007-07-25-DominatedLoop.ll > /dev/null
