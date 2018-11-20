# DBDS info

## IMPORTANT NOTES

### Preparing tests

Do not forget to remove the `optnone` attribute in the .ll produced by `clang`. This will make opt skip its opts.

Beware that there is also an attribute `noinline` which one may want to remove at a later stage. Post DBDS implementation.

### Runtime errors

If you use analysis in runOnModule, `getAnalysis<AnalysisType>(Function)`

If you use analysis in runOnFunction, `getAnalysis<AnalysisType>(void)`


## debugging with lldb

1. `lldb opt`
2. `process launch -o /dev/null -i test/Transforms/GVN/2007-07-25-DominatedLoop.ll -- -load ../build/lib/LLVMBlockDuplicator.so -simulator`

	2.1. `-i <llvm IR file>`

	2.2. `-o /dev/null` to suppress ll output
