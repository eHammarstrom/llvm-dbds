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

private:
  // Benefit of duplication simulation
  int Benefit;
  // Cost of duplication simulation
  int Cost;
};

class AddAction : public SimulationAction {
public:
  AddAction(pair<Instruction *, Instruction *>);
  bool apply(BasicBlock *, InstructionMap);

private:
  pair<Instruction *, Instruction *> ActionInst;
};

class RemoveAction : public SimulationAction {
public:
  RemoveAction(Instruction *);
  bool apply(BasicBlock *, InstructionMap);

private:
  Instruction *ActionInst;
};

class ReplaceAction : public SimulationAction {
public:
  ReplaceAction(pair<Instruction *, Instruction *>);
  bool apply(BasicBlock *, InstructionMap);

private:
  pair<Instruction *, Instruction *> ActionInst;
};

// Interface to be implemented by an optimization
class ApplicabilityCheck {
public:
  ApplicabilityCheck() {}
  virtual ~ApplicabilityCheck() = 0;
  // Returns the actions that should be taken to apply an optimization
  virtual vector<SimulationAction *> simulate(SymbolMap,
                                              vector<Instruction *>) = 0;
};

// MemCpyOptimizer-like optimizations
class MemCpyApplicabilityCheck : public ApplicabilityCheck {
public:
  MemCpyApplicabilityCheck() {}
  ~MemCpyApplicabilityCheck();
  vector<SimulationAction *> simulate(SymbolMap, vector<Instruction *>);
};

class Simulation {
public:
  Simulation(BasicBlock *BP, BasicBlock *BM);
  ~Simulation();
  void run();
  int simulationBenefit();
  // Performs duplication, returning the merged BasicBlock with the new
  // instructions. This will replace bp in the CFG.
  bool apply();

private:
  // Duplicate BM, and merge inte BP
  InstructionMap mergeBlocks();
  // ApplicabilityChecks
  vector<ApplicabilityCheck *> AC = {new MemCpyApplicabilityCheck()};
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
