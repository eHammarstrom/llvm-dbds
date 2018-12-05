//===- DeadStoreElimination.h - Fast Dead Store Elimination -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements a trivial dead store elimination that only considers
// basic-block local redundant stores.
//
// FIXME: This should eventually be extended to be a post-dominator tree
// traversal.  Doing so would be pretty trivial.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TRANSFORMS_SCALAR_DEADSTOREELIMINATION_H
#define LLVM_TRANSFORMS_SCALAR_DEADSTOREELIMINATION_H

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/MemoryLocation.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/IR/PassManager.h"

namespace llvm {

class Function;

/// This class implements a trivial dead store elimination. We consider
/// only the redundant stores that are local to a single Basic Block.
class DSEPass : public PassInfoMixin<DSEPass> {
public:
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &FAM);
};

namespace dse {

using OverlapIntervalsTy = std::map<int64_t, int64_t>;
using InstOverlapIntervalsTy = DenseMap<Instruction *, OverlapIntervalsTy>;

enum OverwriteResult {
  OW_Begin,
  OW_Complete,
  OW_End,
  OW_PartialEarlierWithFullLater,
  OW_Unknown
};

uint64_t getPointerSize(const Value *V, const DataLayout &DL,
                        const TargetLibraryInfo &TLI, const Function *F);
bool hasAnalyzableMemoryWrite(Instruction *I, const TargetLibraryInfo &TLI);
MemoryLocation getLocForWrite(Instruction *Inst);
bool isRemovable(Instruction *I);
bool isPossibleSelfRead(Instruction *Inst, const MemoryLocation &InstStoreLoc,
                        Instruction *DepWrite, const TargetLibraryInfo &TLI,
                        AliasAnalysis &AA);
OverwriteResult isOverwrite(const MemoryLocation &Later,
                            const MemoryLocation &Earlier, const DataLayout &DL,
                            const TargetLibraryInfo &TLI, int64_t &EarlierOff,
                            int64_t &LaterOff, Instruction *DepWrite,
                            InstOverlapIntervalsTy &IOL, AliasAnalysis &AA,
                            const Function *F);

} // namespace dse

} // end namespace llvm

#endif // LLVM_TRANSFORMS_SCALAR_DEADSTOREELIMINATION_H
