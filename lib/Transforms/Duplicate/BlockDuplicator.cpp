//===- Hello.cpp - Example code from "Writing an LLVM Pass" ---------------===//
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
// See the paper(s):
//
// "Dominance-Based Duplication Simulation (DBDS)", by D. Leopoldseder,
// L. Stadler, T. Würthinger, J. Eisl, D. Simon, H. Mössenböck
//
//
//===----------------------------------------------------------------------===//

#include <vector>

#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/Pass.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/CFG.h"
using namespace llvm;

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

  bool runOnFunction(Function &F) {
    vector<DomTreeNodeBase<BasicBlock>*> WorkList;
    DomTreeNodeBase<BasicBlock> *Node;
    BasicBlock *BB;

    errs() << "simulator: "; errs().write_escaped(F.getName()) << '\n';
    ++FunctionCounter;

    // domtree<node<basicblock>>> of function F
    DominatorTree &DT = getAnalysis<DominatorTreeWrapperPass>().getDomTree();

    DT.print(errs());

    WorkList.push_back(DT.getRootNode());

    // a DFS of the domtree(function F)
    do {
      // pop DFS child off worklist
      Node = WorkList.back();
      WorkList.pop_back();

      BB = Node->getBlock();
      errs().write_escaped(BB->getName()) << '\n';

      // TODO: perform algorithm

      // check if basicblock is a mergepoint for each pred
      for (auto Predecessor : predecessors(BB)) {
        if (!DT.dominates(Predecessor, BB)) {

          // benefit, cost = simulateMerge(Predecessor, BB)

          errs() << '\t' << "is a merge point with " << Predecessor->getName() << '\n';
        }
      }

      // add children of node to worklist
      for (auto &Child : Node->getChildren()) {
        WorkList.push_back(Child);
      }
    } while (!WorkList.empty());

    return false;
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    FunctionPass::getAnalysisUsage(AU);
    AU.addRequired<DominatorTreeWrapperPass>();

    // required - need before our pass
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
