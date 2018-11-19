#!/bin/bash

git clone --single-branch -b release_70 https://github.com/llvm-mirror/clang tools/clang

git clone --single-branch -b release_70 https://github.com/llvm-mirror/compiler-rt projects/compiler-rt
