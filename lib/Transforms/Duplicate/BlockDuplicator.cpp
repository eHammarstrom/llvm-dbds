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

#include <vector>
#include <algorithm>

#include "llvm/ADT/Statistic.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
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

    /*
      AU.addRequiredID(LoopSimplifyID);
      AU.addRequiredID(LCSSAID);
    */
    /*
      AU.addRequired<LoopInfo>();
      AU.addRequired<DependenceAnalysis>();
      AU.addRequired<ScalarEvolution>();
      AU.addRequired<TargetTransformInfo>();
    */

    // preserved - not invalidated
    /*
      AU.addPreserved<DominatorTree>();
      AU.addPreserved<DependenceAnalysis>();
    */
  }
};
} // namespace

char DBDuplicationSimulation::ID = 0;
static RegisterPass<DBDuplicationSimulation> X("simulator",
                                               "Duplication Simulator Pass");

bool DBDuplicationSimulation::runOnFunction(Function &F) {
  if (skipFunction(F))
    return false;

  errs() << "simulator: ";
  errs().write_escaped(F.getName()) << '\n';
  ++FunctionCounter;

  // domtree<node<basicblock>>> of function F
  DominatorTree &DT = getAnalysis<DominatorTreeWrapperPass>().getDomTree();

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
    errs().write_escaped(BB->getName()) << '\n';

    // TODO: perform algorithm

    for (BasicBlock *BBSuccessor : successors(BB)) {
      if (BlockIsIfMergePoint(BBSuccessor)) {
        // BB          = b_pi  in paper [0]
        // BBSuccessor = b_m   in paper [0]

        errs() << '\t' << "has successor merge point " << BBSuccessor->getName()
               << '\n';
        errs() << '\t' << "running simulation\n";

        // Simulate duplication
        Simulation *S = new Simulation(BB, BBSuccessor);
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

  const int BenefitThreshold = 10;

  // Apply simulations if passing benefit/cost threshold
  for (Simulation *S : Simulations) {
    if (S->simulationBenefit() > BenefitThreshold) {
      Changed |= S->apply();
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

bool SimulationAction::apply(BasicBlock *NewBlock, InstructionMap IMap) {
  bool Changed = false;

  switch (Type) {
  case Add:
    for (auto P : u.AddInstructions) {
      // Translate instruction pointer to duplicated instruction pointer
      Instruction *Reference = IMap.at(P.first);
      Instruction *Addition = P.second;

      Reference->insertAfter(Addition);
      Changed |= true;
    }
    break;
  case Remove:
    for (auto I : u.RemoveInstructions) {
      // Translate instruction pointer to duplicated instruction pointer
      IMap.at(I)->eraseFromParent();
      Changed |= true;
    }
    break;
  case Replace:
    for (auto P : u.ReplaceInstructions) {
      // Translate instruction pointer to duplicated instruction pointer
      Instruction *Replacee = IMap.at(P.first);
      Instruction *Replacer = P.second;

      ReplaceInstWithInst(Replacee, Replacer);
      Changed |= true;
    }
    break;
  }

  return Changed;
}

Simulation::Simulation(BasicBlock *bp, BasicBlock *bm) : BP(bp), BM(bm) {

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
    SimulationAction *action = check->simulate(PHITranslation, Instructions);
    Actions.push_back(action);
  }
}

int Simulation::simulationBenefit() {
  int Benefit = 0;
  int Cost = 0;

  for (auto A : Actions) {
    Benefit += A->getBenefit();
    Cost += A->getCost();
  }

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

    if (isa<PHINode>(I)) {
      BP->getInstList().insert(BP->begin(), ClonedInstruction);
    } else {
      BP->getInstList().insert(BP->end(), ClonedInstruction);
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

SimulationAction *
MemCpyApplicabilityCheck::simulate(SymbolMap Map,
                                   vector<Instruction *> Instrs) {
  return NULL;
}

MemCpyApplicabilityCheck::~MemCpyApplicabilityCheck() {}
