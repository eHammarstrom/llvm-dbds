//===- BlockDuplicator.cpp - Block duplication pass ------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements dominance-based duplication simulation.
//
//
// Some notation used:
// bp = a Basic Block Predecessor of bm
// bm = a Merge Basic Block, successor of bp
//
//
// See the paper(s):
//
// "Dominance-Based Duplication Simulation (DBDS)", by D. Leopoldseder,
// L. Stadler, T. Würthinger, J. Eisl, D. Simon, H. Mössenböck
//
//
//===----------------------------------------------------------------------===//

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/CaptureTracking.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/Analysis/MemoryBuiltins.h"
#include "llvm/Analysis/MemoryDependenceAnalysis.h"
#include "llvm/Analysis/MemoryLocation.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Type.h"
#include "llvm/Pass.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Duplicate/BlockDuplicator.h"
#include "llvm/Transforms/Scalar/DeadStoreElimination.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/Cloning.h"

using namespace llvm;
using namespace llvm::blockduplicator;

#define DEBUG_TYPE "simulator"

STATISTIC(FunctionCounter, "Counts number of functions entered");
// STATISTIC(SimulationCounter, "Counts number of possible simulation points");

namespace {
//===--------------------------------------------------------------------===//
// DBDuplicationSimulation pass implementation
//

using namespace std;

struct DBDuplicationSimulation : public FunctionPass {
  static char ID; // Pass identification, replacement for typeid
  DBDuplicationSimulation() : FunctionPass(ID) {}

  bool runOnFunction(Function &F) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    FunctionPass::getAnalysisUsage(AU);

    // required - need before our pass
    AU.addRequired<DominatorTreeWrapperPass>();
    AU.addRequired<TargetTransformInfoWrapperPass>();
    AU.addRequired<TargetLibraryInfoWrapperPass>();
    AU.addRequired<TargetLibraryInfoWrapperPass>();

    AU.addRequired<MemoryDependenceWrapperPass>();
    // AU.addPreserved<MemoryDependenceWrapperPass>(); // do we actually
    // preserve?

    AU.addRequired<AAResultsWrapperPass>();
    // AU.addPreserved<GlobalsAAWrapperPass>(); // do we actually preserve?

    // maybe?
    // AU.addRequired<DependenceAnalysisWrapperPass>();
  }
};
} // namespace

char DBDuplicationSimulation::ID = 0;
static RegisterPass<DBDuplicationSimulation> X("simulator",
                                               "Duplication Simulator Pass");

bool DBDuplicationSimulation::runOnFunction(Function &F) {
  if (skipFunction(F))
    return false;

#ifdef PRINTS
  errs() << "simulator: ";
  errs().write_escaped(F.getName()) << '\n';
#endif
  ++FunctionCounter;

  // domtree<node<basicblock>>> of function F
  auto &DT = getAnalysis<DominatorTreeWrapperPass>().getDomTree();
  auto &TTI = getAnalysis<TargetTransformInfoWrapperPass>().getTTI(F);
  auto &TLI = getAnalysis<TargetLibraryInfoWrapperPass>().getTLI();
  auto &MD = getAnalysis<MemoryDependenceWrapperPass>().getMemDep();
  auto &AA = getAnalysis<AAResultsWrapperPass>().getAAResults();

  DT.print(errs());

  vector<DomTreeNodeBase<BasicBlock> *> WorkList;
  WorkList.push_back(DT.getRootNode());

  // Merge simulations performed on the DT
  vector<Simulation *> Simulations;

  // A DFS of the domtree(function F)
  do {
    // Pop DFS child off worklist
    DomTreeNodeBase<BasicBlock> *Node = WorkList.back();
    WorkList.pop_back();

    BasicBlock *BB = Node->getBlock();
#ifdef PRINTS
    errs().write_escaped(BB->getName()) << '\n';
#endif

    for (BasicBlock *BBSuccessor : successors(BB)) {
      if (BlockIsIfMergePoint(BBSuccessor)) {
        // BB          = b_pi  in paper [0]
        // BBSuccessor = b_m   in paper [0]

#ifdef PRINTS
        errs() << '\t' << "has successor merge point " << BBSuccessor->getName()
               << '\n';
        errs() << '\t' << "running simulation\n";
#endif

        /* Test, prints cost of all instructions in succ(BB)
        for (auto II = BBSuccessor->begin(); II != BBSuccessor->end(); ++II) {
          unsigned Cost = TTI.getInstructionCost(
              &(*II), TargetTransformInfo::TCK_RecipThroughput);
          errs() << '\t';
          (*II).print(errs());
          errs() << " = " << Cost << '\n';
        }
        */

        // Simulate duplication
        Simulation *S =
            new Simulation(&TTI, &TLI, &MD, &AA, &F, BB, BBSuccessor);
        S->run();

        // Collect all simulations
        Simulations.push_back(S);
      }
    }

    // add children of node to worklist
    for (auto &Child : Node->getChildren()) {
      WorkList.push_back(Child);
    }
  } while (!WorkList.empty());

  bool Changed = false;

  // Sort simulations by benefit/cost
  std::sort(Simulations.begin(), Simulations.end(),
            [](Simulation *S1, Simulation *S2) -> bool {
              return S1->simulationBenefit() > S2->simulationBenefit();
            });

  const int BenefitThreshold = 0;

  // Apply simulations if passing benefit/cost threshold
#ifdef PRINTS
  int i = 0;
#endif
  for (Simulation *S : Simulations) {
#ifdef PRINTS
    errs() << "Simulation(" << ++i
           << ") has Benefit = " << S->simulationBenefit() << '\n';
#endif
    if (S->simulationBenefit() > BenefitThreshold) {
      Changed |= S->apply();
#ifdef PRINTS
      errs() << "\tApplied simulation! (" << Changed << ")\n";
#endif
    }
  }

  return Changed;
}

void appendInstructions(vector<Instruction *> &Instructions, BasicBlock *BB) {
  for (auto II = BB->begin(); II != BB->end(); ++II) {
    Instruction *I = cast<Instruction>(II);

    if (isa<PHINode>(I) || isa<BranchInst>(I))
      continue;

    Instructions.push_back(I);
  }
}

int SimulationAction::getBenefit() {
  const int BenefitScaleFactor = 256;

  return Benefit * BenefitScaleFactor;
}

int SimulationAction::getCost() { return Cost; }

AddAction::AddAction(const TargetTransformInfo *TTI,
                     pair<Instruction *, Instruction *> P)
    : ActionInst(P) {
  // TODO: calculate benefit/cost
  // in this case it is only a cost
}

bool AddAction::apply(BasicBlock *NewBlock, InstructionMap IMap) {
  // Translate instruction pointer to duplicated instruction pointer
  // Do this in NewBlock
  Instruction *Reference = IMap.at(ActionInst.first);
  Instruction *Addition = ActionInst.second;
  Reference->insertAfter(Addition);
  return true; // there may be cases where we cannot apply an action
}

RemoveAction::RemoveAction(const TargetTransformInfo *TTI, Instruction *I)
    : ActionInst(I) {
  // TODO: calculate benefit/cost
  // in this case it is only a benefit
  unsigned InstCost =
      TTI->getInstructionCost(I, TargetTransformInfo::TCK_RecipThroughput);
#ifdef PRINTS
  errs() << "\tA simulated removal will save = " << InstCost << '\n';
#endif
  Benefit += InstCost;
}

bool RemoveAction::apply(BasicBlock *NewBlock, InstructionMap IMap) {
  // Translate instruction pointer to duplicated instruction pointer
  // Do this in NewBlock
  IMap.at(ActionInst)->eraseFromParent();
  return true;
}

ReplaceAction::ReplaceAction(const TargetTransformInfo *TTI,
                             pair<Instruction *, Instruction *> P)
    : ActionInst(P) {
  // TODO: calculate benefit/cost
  // in this case we both benefit and cost
}

bool ReplaceAction::apply(BasicBlock *NewBlock, InstructionMap IMap) {
  // Translate instruction pointer to duplicated instruction pointer
  // Do this in NewBlock
  Instruction *Replacee = IMap.at(ActionInst.first);
  Instruction *Replacer = ActionInst.second;
  ReplaceInstWithInst(Replacee, Replacer);
  return true;
}

Simulation::Simulation(const TargetTransformInfo *TTI,
                       const TargetLibraryInfo *TLI,
                       MemoryDependenceResults *MD, AliasAnalysis *AA,
                       Function *F, BasicBlock *bp, BasicBlock *bm)
    : TTI(TTI), TLI(TLI), MD(MD), AA(AA), BP(bp), BM(bm) {
  Module *Mod = BP->getModule();

  AC.push_back(new MemCpyApplicabilityCheck(TTI, TLI, MD, AA, Mod, F));
  AC.push_back(new DeadStoreApplicabilityCheck(TTI, TLI, MD, AA, Mod, F));

  for (BasicBlock::iterator I = BM->begin(); isa<PHINode>(I); ++I) {
    PHINode *PN = cast<PHINode>(I);

    const int BPIndex = PN->getBasicBlockIndex(BP);

    Value *PredValue = PN->getIncomingValue(BPIndex);

    PHITranslation.insert(pair<Value *, Value *>(PN, PredValue));
  }

  appendInstructions(Instructions, BP);
  appendInstructions(Instructions, BM);
}

void Simulation::run() {
  for (auto &check : AC) {
    // Simulate an optimization using an AC
    vector<SimulationAction *> SimActions =
        check->simulate(PHITranslation, Instructions);
    // Save the generated actions taken by optimization
    Actions.insert(Actions.end(), SimActions.begin(), SimActions.end());
  }
}

int Simulation::simulationBenefit() {
  int Benefit = 0;
  int Cost = 0;

  for (auto A : Actions) {
    Benefit += A->getBenefit();
    Cost += A->getCost();
  }

  // TODO: evaluate the benefit function

  return Benefit - Cost;
}

InstructionMap Simulation::mergeBlocks() {
  InstructionMap IMap;

  // Remove branch to BM in BP
  BP->getInstList().pop_back();

  // Provide a mapping from all instructions to themselves
  // in the predecessor block.
  for (auto II = BP->begin(); II != BP->end(); ++II) {
    Instruction *I = cast<Instruction>(II);
    IMap.insert(pair<Instruction *, Instruction *>(I, I));
  }

  // Provide a mapping from all instructions to themselves
  // in the merge block. Move them to the predecessor block.
  // And reduce their PHI-usages.
  for (auto II = BM->begin(); II != BM->end(); ++II) {
    Instruction *I = cast<Instruction>(II);

    // Here we peek into the map using the old instruction I
    // because the PHITranslation was done in the original block
    PHINode *PNC = dyn_cast<PHINode>(I);
    if (PNC)
      continue;

    // Clone instruction in merge block
    Instruction *ClonedInstruction = I->clone();

    // Create a synonym from old merge block instruction to newly cloned
    // instruction
    IMap.insert(pair<Instruction *, Instruction *>(I, ClonedInstruction));

    // Insert the newly cloned instruction at the end of predecessor block
    BP->getInstList().insert(BP->end(), ClonedInstruction);

    // Replace all PHI uses with the Value mappings in the PHITranslation map
    for (unsigned IOP = 0; IOP < ClonedInstruction->getNumOperands(); ++IOP) {
      Value *OP = ClonedInstruction->getOperand(IOP);

      // If we have no translation, continue
      if (PHITranslation.find(OP) == PHITranslation.end())
        continue;

      // Replace PHI usage with concrete Value translation
      ClonedInstruction->replaceUsesOfWith(OP, PHITranslation[OP]);
    }
  }

  return IMap;
}

bool Simulation::apply() {
  InstructionMap IMap = mergeBlocks();

  bool Changed = false;

  for (auto SA : Actions) {
    Changed |= SA->apply(BP, IMap);
  }

  return Changed;
}

vector<SimulationAction *>
MemCpyApplicabilityCheck::simulate(SymbolMap Map,
                                   const vector<Instruction *> Instructions) {
  vector<SimulationAction *> SimActions;

  errs() << "Performing MemCpy AC!\n";

  // Reverse iterator to only save last memcpy
  for (auto IIA = Instructions.rbegin(); IIA != Instructions.rend(); ++IIA) {
    MemCpyInst *MemCpyA;
    if (!(MemCpyA = dyn_cast<MemCpyInst>(*IIA)))
      continue;

    ConstantInt *CpySizeA = dyn_cast<ConstantInt>(MemCpyA->getLength());

    // Reverse iterate after IIA and remove redundant memcpy
    for (auto IIB = IIA + 1; IIB != Instructions.rend(); ++IIB) {
      Instruction *IB = *IIB;

      MemCpyInst *MemCpyB;
      if (!(MemCpyB = dyn_cast<MemCpyInst>(IB)))
        continue;

      // We cannot optimize volatile memory
      if (MemCpyB->isVolatile())
        break;

      ConstantInt *CpySizeB = dyn_cast<ConstantInt>(MemCpyB->getLength());

      // B = memcpy(b <- a, x)
      // ..
      // unchanged(b)
      // A = memcpy(c <- b, y)
      // transform(A, memcpy(c <- a, y))
      if (MemCpyA->getSource() == MemCpyB->getDest()) {
        // TODO: see comment above
      }

      // Optimization after this point needs both copy lengths
      if (!CpySizeA || !CpySizeB)
        continue;

      // Optimization after this point require src(A) == src(B)
      if (MemCpyA->getSource() != MemCpyB->getSource())
        continue;

      // Optimization after this point require dest(A) == dest(B)
      if (MemCpyA->getDest() != MemCpyB->getDest())
        continue;

      /* This should be handled by Dead Store Elimination AC
      // B = memcpy(b <- a, 10)
      // A = memcpy(b <- a, 12)
      // remove(B)
      if (CpySizeA->getZExtValue() >= CpySizeB->getZExtValue()) {
        RemoveAction *SA = new RemoveAction(TTI, *IIB);
        SimActions.push_back(SA);
      }
      */

      // B = memcpy(b <- a, 12)
      // A = memcpy(b <- a, 10)
      // transform(B, memcpy((b+10) <- (a+10), 2))
      if (CpySizeA->getZExtValue() < CpySizeB->getZExtValue()) {
        // TODO: see comment above
      }
    }
  }

  return SimActions;
}

MemCpyApplicabilityCheck::~MemCpyApplicabilityCheck() {}

/**
 *
 * This implementation of the DSE AC is using the
 * same logic as DeadStoreElimination.cpp:eliminateDeadStores
 *
 */
vector<SimulationAction *>
DeadStoreApplicabilityCheck::simulate(SymbolMap Map,
                                      vector<Instruction *> Instructions) {
  errs() << "Performing DSE AC!\n";

  vector<SimulationAction *> SimActions;

  unordered_set<Instruction *> RemovedInst;

  // For DSE memory dependence analysis.
  size_t LastThrowingInstIndex = 0;
  DenseMap<Instruction *, size_t> InstrOrdering;
  size_t InstrIndex = 1;

  // Mod is the current compilation module
  const DataLayout &DL = Mod->getDataLayout();

  // A map of interval maps representing partially-overwritten value parts.
  dse::InstOverlapIntervalsTy IOL;

  for (auto IIA = Instructions.rbegin(); IIA != Instructions.rend(); ++IIA) {
    Instruction *IA = *IIA;

    // We don't handle free calls, DSE does,
    // see DeadStoreElimination.cpp:eliminateDeadStores.
    if (isFreeCall(IA, TLI)) {
      continue;
    }

    size_t CurInstNumber = InstrIndex++;
    InstrOrdering.insert(std::make_pair(IA, CurInstNumber));
    if (IA->mayThrow()) {
      LastThrowingInstIndex = CurInstNumber;
      continue;
    }

    // Check to see if Inst writes to memory.  If not, continue.
    if (!dse::hasAnalyzableMemoryWrite(IA, *TLI))
      continue;

    errs() << "AnalyzableMemWrite\n";
    errs() << "here: ";
    IA->print(errs());
    errs() << '\n';

    // If we find something that writes memory, get its memory dependence.
    // MemDepResult InstDep = MD->getDependency(IA);

    // Ignore any store where we can't find a local dependence.
    // FIXME: cross-block DSE would be fun. :)
    /*
    if (!InstDep.isDef() && !InstDep.isClobber())
      continue;
    */

    // Figure out what location is being stored to.
    MemoryLocation Loc = dse::getLocForWrite(IA);

    // If we didn't get a useful location, fail.
    if (!Loc.Ptr)
      continue;

    errs() << "Before while\n";

    // Loop until we find a store we can eliminate
    for (auto IIB = IIA + 1; IIB != Instructions.rend(); ++IIB) {

      // Get the memory clobbered by the instruction we depend on.  MemDep will
      // skip any instructions that 'Loc' clearly doesn't interact with.  If we
      // end up depending on a may- or must-aliased load, then we can't optimize
      // away the store and we bail out.  However, if we depend on something
      // that overwrites the memory location we *can* potentially optimize it.

      // Find out what memory location the dependent instruction stores.
      // Instruction *DepWrite = InstDep.getInst();
      Instruction *DepWrite = *IIB;

      // Can't look past this instruction if it might read 'Loc'.
      if (isRefSet(AA->getModRefInfo(DepWrite, Loc))) {
        errs() << "\tInst: ";
        DepWrite->print(errs());
        errs() << "\n\tMight read our mem loc\n";
        break;
      }

      if (!dse::hasAnalyzableMemoryWrite(DepWrite, *TLI))
        continue;

      errs() << "Before while2\n";
      MemoryLocation DepLoc = dse::getLocForWrite(DepWrite);
      // If we didn't get a useful location, or if it isn't a size, bail out.
      if (!DepLoc.Ptr)
        continue;
      errs() << "Before while3\n";

      // Make sure we don't look past a call which might throw. This is an
      // issue because MemoryDependenceAnalysis works in the wrong direction:
      // it finds instructions which dominate the current instruction, rather
      // than instructions which are post-dominated by the current instruction.
      //
      // If the underlying object is a non-escaping memory allocation, any store
      // to it is dead along the unwind edge. Otherwise, we need to preserve
      // the store.
      /*
      size_t DepIndex = InstrOrdering.lookup(DepWrite);
      assert(DepIndex && "Unexpected instruction");
      if (DepIndex <= LastThrowingInstIndex) {
        const Value *Underlying = GetUnderlyingObject(DepLoc.Ptr, DL);
        bool IsStoreDeadOnUnwind = isa<AllocaInst>(Underlying);
        if (!IsStoreDeadOnUnwind) {
          // We're looking for a call to an allocation function
          // where the allocation doesn't escape before the last
          // throwing instruction; PointerMayBeCaptured
          // reasonably fast approximation.
          IsStoreDeadOnUnwind = isAllocLikeFn(Underlying, TLI) &&
                                !PointerMayBeCaptured(Underlying, false, true);
        }
        if (!IsStoreDeadOnUnwind)
          continue;
      }
      */

      // If we find a write that is a) removable (i.e., non-volatile), b) is
      // completely obliterated by the store to 'Loc', and c) which we know that
      // 'Inst' doesn't load from, then we can remove it.
      // Also try to merge two stores if a later one only touches memory written
      // to by the earlier one.
      if (dse::isRemovable(DepWrite) &&
          !dse::isPossibleSelfRead(IA, Loc, DepWrite, *TLI, *AA)) {

        if (RemovedInst.find(DepWrite) != RemovedInst.end())
          continue;

        errs() << "\tDSE AC: may remove,\n";
        errs() << "\tthis: ";
        DepWrite->print(errs());
        errs() << '\n';
        errs() << "\tbecause of this: ";
        IA->print(errs());
        errs() << '\n';

        int64_t InstWriteOffset, DepWriteOffset;

        dse::OverwriteResult OR =
            dse::isOverwrite(Loc, DepLoc, DL, *TLI, DepWriteOffset,
                             InstWriteOffset, DepWrite, IOL, *AA, F);

        if (OR == dse::OW_Complete) {
          errs() << "OW_Complete\n";
          // Delete the store.
          SimActions.push_back(new RemoveAction(TTI, DepWrite));
          RemovedInst.insert(DepWrite);
        } else if (OR == dse::OW_Begin) {
          errs() << "OW_Begin\n";
          // TODO
        } else if (OR == dse::OW_End) {
          errs() << "OW_End\n";
          // TODO
        } else if (OR == dse::OW_PartialEarlierWithFullLater) {
          errs() << "OW_PartialEarlierWithFullLater\n";
          // TODO
        }
      }
    }
  }

  return SimActions;
}

DeadStoreApplicabilityCheck::~DeadStoreApplicabilityCheck() {}
