//===- BlockDuplicator.h - Block duplication pass ---------------*- C++ -*-===//

//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
// This file implements dominance-based duplication simulation.
//
//
// See the paper(s):
//
// "Dominance-Based Duplication Simulation (DBDS)", by D. Leopoldseder,
// L. Stadler, T. Würthinger, J. Eisl, D. Simon, H. Mössenböck
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TRANSFORMS_DUPLICATE_BLOCKDUPLICATOR_H
#define LLVM_TRANSFORMS_DUPLICATE_BLOCKDUPLICATOR_H

namespace llvm {
namespace blockduplicator {

using namespace std;

// Left Value is a PHI-instr
// Right Value is PHI.getOperand(bp) where bp is the predecessor of bm
typedef map<Value *, Value *> SymbolMap;
typedef map<Instruction *, Instruction *> InstructionMap;

class SimulationAction {
public:
  SimulationAction() {}
  // Calculates the benefit of an action taken
  virtual bool apply(BasicBlock *, InstructionMap) = 0;
  int getBenefit();
  int getCost();

protected:
  // Benefit of duplication simulation
  int Benefit = 0;
  // Cost of duplication simulation
  int Cost = 0;
};

class AddAction : public SimulationAction {
public:
  AddAction(TargetTransformInfo *, pair<Instruction *, Instruction *>);
  bool apply(BasicBlock *, InstructionMap);

private:
  pair<Instruction *, Instruction *> ActionInst;
};

class RemoveAction : public SimulationAction {
public:
  RemoveAction(TargetTransformInfo *, Instruction *);
  bool apply(BasicBlock *, InstructionMap);

private:
  Instruction *ActionInst;
};

class ReplaceAction : public SimulationAction {
public:
  ReplaceAction(TargetTransformInfo *, pair<Instruction *, Instruction *>);
  bool apply(BasicBlock *, InstructionMap);

private:
  pair<Instruction *, Instruction *> ActionInst;
};

// Interface to be implemented by an optimization
class ApplicabilityCheck {
public:
  ApplicabilityCheck(const TargetTransformInfo *TTI,
                     const TargetLibraryInfo *TLI)
      : TTI(TTI), TLI(TLI) {}
  virtual ~ApplicabilityCheck() = 0;
  // Returns the actions that should be taken to apply an optimization
  virtual vector<SimulationAction *> simulate(const SymbolMap,
                                              const vector<Instruction *>) = 0;

private:
  const TargetTransformInfo *TTI;
  const TargetLibraryInfo *TLI;
};

// MemCpyOptimizer-like optimizations
class MemCpyApplicabilityCheck : public ApplicabilityCheck {
public:
  using ApplicabilityCheck::ApplicabilityCheck;
  ~MemCpyApplicabilityCheck();
  vector<SimulationAction *> simulate(const SymbolMap, const vector<Instruction *>);
};

// DeadStoreElimination-like optimizations
class DeadStoreApplicabilityCheck : public ApplicabilityCheck {
public:
  using ApplicabilityCheck::ApplicabilityCheck;
  ~DeadStoreApplicabilityCheck();
  vector<SimulationAction *> simulate(const SymbolMap, const vector<Instruction *>);
};

class Simulation {
public:
  Simulation(const TargetTransformInfo *TTI, const TargetLibraryInfo *TLI,
             BasicBlock *bp, BasicBlock *bm);
  ~Simulation();
  void run();
  // The simulation benefit accounts for the cost of the duplication
  int simulationBenefit();
  // Performs duplication, returning the merged BasicBlock with the new
  // instructions. This will replace bp in the CFG.
  bool apply();

private:
  // Instruction cost model
  const TargetTransformInfo *TTI;
  // Function usage info and more
  const TargetLibraryInfo *TLI;
  // Duplicate BM, and merge inte BP
  InstructionMap mergeBlocks();
  // ApplicabilityChecks
  vector<ApplicabilityCheck *> AC;
  // Predecessor basic block
  BasicBlock *BP;
  // Successor merge basic block
  BasicBlock *BM;
  // Maps a merge basic block phi-function to the value
  // defined in the predecessor block bm.
  SymbolMap PHITranslation;
  // Vector over the instructions in the simulation
  vector<Instruction *> Instructions;
  // The actions to be taken to transform the duplication block
  // into its optimized equivalent.
  vector<SimulationAction *> Actions;
};

} // namespace blockduplicator
} // namespace llvm

#endif
