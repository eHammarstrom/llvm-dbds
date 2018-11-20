# DBDS info

## IMPORTANT NOTES

### Runtime errors

If you use analysis in runOnModule, `getAnalysis<AnalysisType>(Function)`

If you use analysis in runOnFunction, `getAnalysis<AnalysisType>(void)`


## debugging with lldb

1. `lldb opt`
2. `process launch -o /dev/null -i test/Transforms/GVN/2007-07-25-DominatedLoop.ll -- -load ../build/lib/LLVMBlockDuplicator.so -simulator`

	2.1. `-i <llvm IR file>`

	2.2. `-o /dev/null` to suppress ll output
