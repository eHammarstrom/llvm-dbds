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
#include <vector>

#include "llvm/ADT/Statistic.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/Analysis/TargetTransformInfo.h"
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
        Simulation *S = new Simulation(&TTI, &TLI, BB, BBSuccessor);
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

AddAction::AddAction(TargetTransformInfo *TTI,
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

RemoveAction::RemoveAction(TargetTransformInfo *TTI, Instruction *I)
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

ReplaceAction::ReplaceAction(TargetTransformInfo *TTI,
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
                       const TargetLibraryInfo *TLI, BasicBlock *bp,
                       BasicBlock *bm)
    : TTI(TTI), TLI(TLI), BP(bp), BM(bm) {

  AC.push_back(new MemCpyApplicabilityCheck(TTI, TLI));
  AC.push_back(new DeadStoreApplicabilityCheck(TTI, TLI));

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

  // Add BM DUPLICATED instructions to BP if
  // there exist no SimulationAction for it
  // else use the SimulationAction
  // And for all instructions -- provide a mapping
  // even if it is to itself.
  for (auto II = BP->begin(); II != BP->end(); ++II) {
    Instruction *I = cast<Instruction>(II);
    IMap.insert(pair<Instruction *, Instruction *>(I, I));
  }

  for (auto II = BM->begin(); II != BM->end(); ++II) {
    Instruction *I = cast<Instruction>(II);
    Instruction *ClonedInstruction = I->clone();

    IMap.insert(pair<Instruction *, Instruction *>(I, ClonedInstruction));

    PHINode *PNC = dyn_cast<PHINode>(ClonedInstruction);
    if (PNC) {
      // Only insert PHI-function if we don't have a mapping to a concrete Value
      if (PHITranslation.find(PNC) == PHITranslation.end())
        BP->getInstList().insert(BP->begin(), ClonedInstruction);
    } else {
      BP->getInstList().insert(BP->end(), ClonedInstruction);
    }
  }

  // Replace all uses of the removed PHI-functions
  // with the new concrete Value.
  for (auto &KV : PHITranslation) {
    KV.first->replaceAllUsesWith(KV.second);
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

vector<SimulationAction *>
DeadStoreApplicabilityCheck::simulate(SymbolMap Map,
                                      vector<Instruction *> Instructions) {
  vector<SimulationAction *> SimActions;

  errs() << "Performing DSE AC!\n";

  // Reverse iterator to only save last memcpy
  for (auto II = Instructions.begin(); II != Instructions.end(); ++II) {
    Instruction *I = *II;

    if (!hasAnalyzableMemoryWrite(I, *TLI))
      continue;

    errs() << "AnalyzableMemWrite\n";
  }

  return SimActions;
}

DeadStoreApplicabilityCheck::~DeadStoreApplicabilityCheck() {}

//===- BlockDuplicator helpers -===//

/// Does this instruction write some memory?  This only returns true for things
/// that we can analyze with other helpers below.
bool
blockduplicator::hasAnalyzableMemoryWrite(Instruction *I,
                         const TargetLibraryInfo &TLI) {
  if (isa<StoreInst>(I))
    return true;
  if (IntrinsicInst *II = dyn_cast<IntrinsicInst>(I)) {
    switch (II->getIntrinsicID()) {
    default:
      return false;
    case Intrinsic::memset:
    case Intrinsic::memmove:
    case Intrinsic::memcpy:
    case Intrinsic::memcpy_element_unordered_atomic:
    case Intrinsic::memmove_element_unordered_atomic:
    case Intrinsic::memset_element_unordered_atomic:
    case Intrinsic::init_trampoline:
    case Intrinsic::lifetime_end:
      return true;
    }
  }
  if (auto CS = CallSite(I)) {
    if (Function *F = CS.getCalledFunction()) {
      StringRef FnName = F->getName();
      if (TLI.has(LibFunc_strcpy) && FnName == TLI.getName(LibFunc_strcpy))
        return true;
      if (TLI.has(LibFunc_strncpy) && FnName == TLI.getName(LibFunc_strncpy))
        return true;
      if (TLI.has(LibFunc_strcat) && FnName == TLI.getName(LibFunc_strcat))
        return true;
      if (TLI.has(LibFunc_strncat) && FnName == TLI.getName(LibFunc_strncat))
        return true;
    }
  }
  return false;
}
