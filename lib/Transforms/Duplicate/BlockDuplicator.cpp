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

#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Casting.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Duplicate/BlockDuplicator.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/CFG.h"
#include "llvm/Pass.h"

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
}

char DBDuplicationSimulation::ID = 0;
static RegisterPass<DBDuplicationSimulation> X("simulator", "Duplication Simulator Pass");

bool DBDuplicationSimulation::runOnFunction(Function &F) {
  if (skipFunction(F))
    return false;

  errs() << "simulator: "; errs().write_escaped(F.getName()) << '\n';
  ++FunctionCounter;

  // domtree<node<basicblock>>> of function F
  DominatorTree &DT = getAnalysis<DominatorTreeWrapperPass>().getDomTree();

  DT.print(errs());

  vector<DomTreeNodeBase<BasicBlock>*> WorkList;
  WorkList.push_back(DT.getRootNode());

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

        // benefit, cost = simulateMerge(BB, BBSuccessor)

        errs() << '\t' << "has successor merge point " << BBSuccessor->getName() << '\n';
      }
    }

    // add children of node to worklist
    for (auto &Child : Node->getChildren()) {
      WorkList.push_back(Child);
    }
  } while (!WorkList.empty());

  bool Changed = false;

  /*
   * Choose simulations to apply
   */

  return Changed;
}

void appendInstructions(vector<Instruction*> &instr, BasicBlock *bb) {
	for (auto i = bb->begin(); i != bb.end(); ++i) {
		Instruction *I = cast<Instruction>(i);

		if (isa<PHINode>(I) || isa<BranchInst>(I))
			continue;

		instr.push_back(I);
	}
}

Simulation::Simulation(BasicBlock* bp,
                       BasicBlock* bm)
    : BP(bp), BM(bm) {

  for (BasicBlock::iterator I = BM->begin(); isa<PHINode>(I); ++I) {
    PHINode *PN = cast<PHINode>(I);

    const int BPIndex = PN->getBasicBlockIndex(BP);

    Value *PredValue = PN->getIncomingValue(BPIndex);

    PHITranslation.insert(
        pair<Value*, Value*>(PN, PredValue));
  }

	appendInstructions(Instructions, BP);
	appendInstructions(Instructions, BM);

	}

void Simulation::run() {
	for (auto& check: AC) {
		check->simulate(PHITranslation, Instructions);
	}
}


BasicBlock* Simulation::apply() {
}


int MemCpyApplicabilityCheck::simulate(SymbolMap Map,
                                   vector<Instruction*> Instrs) {
  vector<SimulationAction*> Actions;

  return 0;
}

MemCpyApplicabilityCheck::~MemCpyApplicabilityCheck() {}
