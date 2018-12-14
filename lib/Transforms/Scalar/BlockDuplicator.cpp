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
#include "llvm/Transforms/Scalar/BlockDuplicator.h"
#include "llvm/Transforms/Scalar/DeadStoreElimination.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/InitializePasses.h"

using namespace llvm;
using namespace llvm::blockduplicator;

#define DEBUG_TYPE "simulator"

STATISTIC(FunctionCounter, "Counts number of functions entered");
STATISTIC(DuplicationCounter, "Counts number of simulations duplicated");

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
INITIALIZE_PASS_BEGIN(DBDuplicationSimulation, "simulator", "Simulate optimizations and duplicate", false,
                      false)
INITIALIZE_PASS_DEPENDENCY(DominatorTreeWrapperPass)
INITIALIZE_PASS_DEPENDENCY(TargetTransformInfoWrapperPass)
INITIALIZE_PASS_DEPENDENCY(TargetLibraryInfoWrapperPass)
INITIALIZE_PASS_DEPENDENCY(MemoryDependenceWrapperPass)
INITIALIZE_PASS_DEPENDENCY(AAResultsWrapperPass)
INITIALIZE_PASS_END(DBDuplicationSimulation, "simulator", "Simulate optimizations and duplicate", false,
                    false)

/*
static RegisterPass<DBDuplicationSimulation> X("simulator",
                                               "Duplication Simulator Pass");
*/

// Public interface to the Duplication Simulation Pass.
FunctionPass* llvm::createDuplicationSimulationPass() {
  return new DBDuplicationSimulation();
}

bool DBDuplicationSimulation::runOnFunction(Function &F) {
  if (skipFunction(F))
    return false;

  ++FunctionCounter;

  // domtree<node<basicblock>>> of function F
  auto &DT = getAnalysis<DominatorTreeWrapperPass>().getDomTree();
  auto &TTI = getAnalysis<TargetTransformInfoWrapperPass>().getTTI(F);
  auto &TLI = getAnalysis<TargetLibraryInfoWrapperPass>().getTLI();
  auto &MD = getAnalysis<MemoryDependenceWrapperPass>().getMemDep();
  auto &AA = getAnalysis<AAResultsWrapperPass>().getAAResults();

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

    for (BasicBlock *BBSuccessor : successors(BB)) {
      if (BlockIsIfMergePoint(BBSuccessor)) {
        // BB          = b_pi  in paper [0]
        // BBSuccessor = b_m   in paper [0]

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
        Simulation *S = new Simulation(&TTI, &TLI, &MD, &AA, &F, BB, BBSuccessor);
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
  for (Simulation *S : Simulations) {
    // S->print(errs());
    if (S->simulationBenefit() > BenefitThreshold) {
      Changed |= S->apply();
      ++DuplicationCounter;
    }

    delete S;
  }

  // may use assert(verifyModule()) to verify IR after pass

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

SimulationAction::~SimulationAction() {};

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
  Benefit = 1;
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
    : BP(bp), BM(bm) {
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

void Simulation::cleanUpPHINodes() {
  for (auto I = BM->begin(); isa<PHINode>(I); ++I) {
    PHINode *PN = cast<PHINode>(I);
    // Remove BP Value from PHI, without removing the instruction.
    // If it becomes empty, the block will be destroyed later.
    if (PN->getIncomingValueForBlock(BP) != PN)
      PN->removeIncomingValue(BP, false);
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

    if (isa<PHINode>(I))
      continue;

    Instruction *ClonedInstruction = I->clone();

    // Create a synonym from old merge block instruction to newly cloned
    // instruction
    IMap.insert(pair<Instruction *, Instruction *>(I, ClonedInstruction));

    // Insert the newly cloned instruction at the end of predecessor block
    BP->getInstList().push_back(ClonedInstruction);

    LLVM_DEBUG(dbgs() << "Clone(" << *I << "->" << *ClonedInstruction
                      << "  )\n");

    // Replace all PHI uses with the Value mappings in the PHITranslation map
    // Also replace all old instruction uses with the new ones in the IMap
    for (unsigned IOP = 0; IOP < ClonedInstruction->getNumOperands(); ++IOP) {
      Value *OP = ClonedInstruction->getOperand(IOP);
      Instruction *OPInst = dyn_cast<Instruction>(OP);

      if (PHITranslation.find(OP) != PHITranslation.end()) {
        // If we have a PHI-translation, translate

        Value *PHIVal = PHITranslation[OP];

        LLVM_DEBUG(dbgs() << "PHITranslation Replacing: " << *OP << " <- "
                          << *PHIVal << '\n');

        ClonedInstruction->replaceUsesOfWith(OP, PHIVal);
      }

      if (IMap.find(OPInst) != IMap.end()) {
        // If we have an IMap translation, translate
        LLVM_DEBUG(dbgs() << "IMap Replacing: " << *OP << " <- "
                          << *IMap[OPInst] << '\n');
        ClonedInstruction->replaceUsesOfWith(OP, IMap[OPInst]);
      }
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

  // Clean up phi values after merge and optimizations
  cleanUpPHINodes();

  return Changed;
}

ApplicabilityCheck::~ApplicabilityCheck() {};

vector<SimulationAction *>
MemCpyApplicabilityCheck::simulate(SymbolMap Map,
                                   const vector<Instruction *> Instructions) {
  vector<SimulationAction *> SimActions;

  // Reverse iterator to only save last memcpy
  for (auto IIA = Instructions.rbegin(); IIA != Instructions.rend(); ++IIA) {

    Instruction *IA = *IIA;

    MemCpyInst *MemCpyA;
    if (!(MemCpyA = dyn_cast<MemCpyInst>(IA)))
      continue;

    MemoryLocation LocA = dse::getLocForWrite(MemCpyA);

    // Reverse iterate after IIA and remove redundant memcpy
    for (auto IIB = IIA + 1; IIB != Instructions.rend(); ++IIB) {

      Instruction *IB = *IIB;

      if (!dse::hasAnalyzableMemoryWrite(IB, *TLI))
        continue;

      // Check if an instruction, that is not a memcpy,
      // writes to MemCpyA memory location, if so we cannot
      // use MemCpyA source for coming optimizations
      if (isModSet(AA->getModRefInfo(IB, LocA)) && !isa<MemCpyInst>(IB)) {
        LLVM_DEBUG(dbgs() << "WritesTo (" << *IB << "," << *IA << " )\n");
        break;
      }

      MemCpyInst *MemCpyB;
      if (!(MemCpyB = dyn_cast<MemCpyInst>(IB)))
        continue;

      // We cannot optimize volatile memory
      if (MemCpyB->isVolatile())
        break;

      // B = memcpy(b <- a, x)
      // ..
      // unchanged(b)
      // A = memcpy(c <- b, y)
      // transform(A, memcpy(c <- a, y))
      if (MemCpyA->getSource() == MemCpyB->getDest()) {
        // Cloning since we only want to change Source Value
        Instruction *I = MemCpyA->clone();
        MemCpyInst *MemCpyI = dyn_cast<MemCpyInst>(I);

        MemCpyI->setSource(MemCpyB->getRawSource());

        LLVM_DEBUG(dbgs() << "Replacing (" << *IA << "," << *I << " )\n");

        ReplaceAction *RA = new ReplaceAction(
            TTI, std::pair<Instruction *, Instruction *>(MemCpyA, MemCpyI));
        SimActions.push_back(RA);
      }
    }
  }

  return SimActions;
}

/**
 *
 * This implementation of the DSE AC is using the
 * same logic as DeadStoreElimination.cpp:eliminateDeadStores
 *
 */
vector<SimulationAction *>
DeadStoreApplicabilityCheck::simulate(SymbolMap Map,
                                      vector<Instruction *> Instructions) {
  vector<SimulationAction *> SimActions;

  unordered_set<Instruction *> RemovedInst;

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

    // Check to see if Inst writes to memory.  If not, continue.
    if (!dse::hasAnalyzableMemoryWrite(IA, *TLI))
      continue;

    // Figure out what location is being stored to.
    MemoryLocation Loc = dse::getLocForWrite(IA);

    // If we didn't get a useful location, fail.
    if (!Loc.Ptr)
      continue;

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
      if (isRefSet(AA->getModRefInfo(DepWrite, Loc)))
        break;

      if (!dse::hasAnalyzableMemoryWrite(DepWrite, *TLI))
        continue;

      MemoryLocation DepLoc = dse::getLocForWrite(DepWrite);
      // If we didn't get a useful location, or if it isn't a size, bail out.
      if (!DepLoc.Ptr)
        continue;

      // If we find a write that is a) removable (i.e., non-volatile), b) is
      // completely obliterated by the store to 'Loc', and c) which we know that
      // 'Inst' doesn't load from, then we can remove it.
      // Also try to merge two stores if a later one only touches memory written
      // to by the earlier one.
      if (dse::isRemovable(DepWrite) &&
          !dse::isPossibleSelfRead(IA, Loc, DepWrite, *TLI, *AA)) {

        if (RemovedInst.find(DepWrite) != RemovedInst.end())
          continue;


        LLVM_DEBUG(dbgs() << "DSE AC(" << *DepWrite << " given " << *IA << " )\n");

        int64_t InstWriteOffset, DepWriteOffset;

        dse::OverwriteResult OR =
            dse::isOverwrite(Loc, DepLoc, DL, *TLI, DepWriteOffset,
                             InstWriteOffset, DepWrite, IOL, *AA, F);

        if (OR == dse::OW_Complete) {
          // Delete the store.
          LLVM_DEBUG(dbgs() << "Removing (" << *DepWrite << " )\n");
          SimActions.push_back(new RemoveAction(TTI, DepWrite));
          RemovedInst.insert(DepWrite);
        } else if ((OR == dse::OW_End &&
                    dse::isShortenableAtTheEnd(DepWrite)) ||
                   ((OR == dse::OW_Begin &&
                     dse::isShortenableAtTheBeginning(DepWrite)))) {

          int64_t EarlierSize = DepLoc.Size;
          int64_t LaterSize = Loc.Size;
          bool IsOverwriteEnd = (OR == dse::OW_End);
          Instruction *EarlierWrite = DepWrite;
          int64_t EarlierOffset = DepWriteOffset;
          int64_t LaterOffset = InstWriteOffset;

          auto *EarlierIntrinsic = cast<AnyMemIntrinsic>(EarlierWrite);
          unsigned EarlierWriteAlign = EarlierIntrinsic->getDestAlignment();
          if (!IsOverwriteEnd)
            LaterOffset = int64_t(LaterOffset + LaterSize);

          if (!(isPowerOf2_64(LaterOffset) &&
                EarlierWriteAlign <= LaterOffset) &&
              !((EarlierWriteAlign != 0) &&
                LaterOffset % EarlierWriteAlign == 0))
            continue;

          int64_t NewLength = IsOverwriteEnd
                                  ? LaterOffset - EarlierOffset
                                  : EarlierSize - (LaterOffset - EarlierOffset);

          if (auto *AMI = dyn_cast<AtomicMemIntrinsic>(EarlierWrite)) {
            // When shortening an atomic memory intrinsic, the newly shortened
            // length must remain an integer multiple of the element size.
            const uint32_t ElementSize = AMI->getElementSizeInBytes();
            if (0 != NewLength % ElementSize)
              continue;
          }

          Value *EarlierWriteLength = EarlierIntrinsic->getLength();
          Value *TrimmedLength =
              ConstantInt::get(EarlierWriteLength->getType(), NewLength);

          auto NewInst = EarlierWrite->clone();
          auto *NewInstIntrinsic = cast<AnyMemIntrinsic>(NewInst);
          NewInstIntrinsic->setLength(TrimmedLength);
          // EarlierIntrinsic->setLength(TrimmedLength);

          LLVM_DEBUG(dbgs() << "Replacing (" << *EarlierWrite << "," << *NewInst << " )\n");
          ReplaceAction *RA = new ReplaceAction(
              TTI,
              std::pair<Instruction *, Instruction *>(EarlierWrite, NewInst));

          SimActions.push_back(RA);

          EarlierSize = NewLength;
          if (!IsOverwriteEnd) {
            int64_t OffsetMoved = (LaterOffset - EarlierOffset);
            Value *Indices[1] = {
                ConstantInt::get(EarlierWriteLength->getType(), OffsetMoved)};
            GetElementPtrInst *NewDestGEP = GetElementPtrInst::CreateInBounds(
                EarlierIntrinsic->getRawDest(), Indices, "", EarlierWrite);

            NewInstIntrinsic->setDest(NewDestGEP);
            // EarlierIntrinsic->setDest(NewDestGEP);
            EarlierOffset = EarlierOffset + OffsetMoved;
          }
        }
      }
    }
  }

  return SimActions;
}
