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
typedef map<Value*, Value*> SymbolMap;
typedef map<Instruction*, Instruction*> InstructionMap;

enum SimulationActionType {
                           Add, // Action adds instruction
                           Remove, // Action removes instruction(s)
                           Replace, // Action replaces instruction(s)
};

class SimulationAction {
public:
  // Calculates the benefit of an action taken
  int getBenefit();
  int getCost();
  bool apply(BasicBlock*, InstructionMap);
private:
  // Benefit of duplication simulation
  int Benefit;
  // Cost of duplication simulation
  int Cost;
  // Type of action
  SimulationActionType Type;

  union {
    // Instruction to be added by action, <Reference, Addition>
    vector<pair<Instruction*, Instruction *>> AddInstructions;
    // Instructions to be removed by action
    vector<Instruction *> RemoveInstructions;
    // Instruction pair to be replaced, <Replacee, Replacer>
    vector<pair<Instruction *, Instruction *>> ReplaceInstructions;
  } u;
};

// Interface to be implemented by an optimization
class ApplicabilityCheck {
public:
  ApplicabilityCheck() {}
  virtual ~ApplicabilityCheck() = 0;
  // Returns the actions that should be taken to apply an optimization
  virtual SimulationAction* simulate(SymbolMap, vector<Instruction*>) = 0;
};

// MemCpyOptimizer-like optimizations
class MemCpyApplicabilityCheck : public ApplicabilityCheck {
public:
  MemCpyApplicabilityCheck() {}
  ~MemCpyApplicabilityCheck();
  SimulationAction* simulate(SymbolMap, vector<Instruction*>);
};

class Simulation {
public:
  Simulation(BasicBlock* BP, BasicBlock* BM);
  ~Simulation();
  void run();
  int simulationBenefit();
  // Performs duplication, returning the merged BasicBlock with the new instructions.
  // This will replace bp in the CFG.
  bool apply();
private:
  // Duplicate BM, and merge inte BP
  InstructionMap mergeBlocks();
  // ApplicabilityChecks
  vector<ApplicabilityCheck*> AC = {new MemCpyApplicabilityCheck()};
  // Predecessor basic block
  BasicBlock* BP;
  // Successor merge basic block
  BasicBlock* BM;
  // Maps a merge basic block phi-function to the value
  // defined in the predecessor block bm.
  SymbolMap PHITranslation;
  // Vector over the instructions in the simulation
  vector<Instruction*> Instructions;
  // The actions to be taken to transform the duplication block
  // into its optimized equivalent.
  vector<SimulationAction*> Actions;
};

}
}

#endif
