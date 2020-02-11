#ifndef TRUTHTABLE_H
#define TRUTHTABLE_H

// {{SMILE_PUBLIC_HEADER}}

#ifndef DEFCPT_H
#include "defcpt.h"
#endif

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
// TRUTHTABLE
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

class DSL_truthTable : public DSL_cpt
{
 // represents a discrete & deterministic chance node.
 // It's a CPT with only 1's and 0's

 // ENTRY VALUES:
 // This node is expecting DISCRETE entries from it's parents
 // wich means that all it's parents should be discrete (or 
 // discretized if they are continuous)

 public:
  // METHODS FOR USER INTERACTION
  // creation/destruction
  DSL_truthTable(int myHandle, DSL_network *theNetwork);
  DSL_truthTable(DSL_nodeDefinition &likeThisOne);
  virtual ~DSL_truthTable() {CleanUp();};

  int operator =(DSL_nodeDefinition &likeThisOne) {return(DSL_cpt::operator =(likeThisOne));};

  // methods to deal with changes in the definition of my parents
  int DaddyGetsBigger(int daddy, int thisPosition);

  // setting and retrieving data
  int GetType() const { return DSL_TRUTHTABLE; }
  const char *GetTypeName() const { return "TRUTHTABLE"; }

  int SetProbability(DSL_intArray &thisCoordinates, double theProbability);
  int SetResultingState(DSL_intArray &thisCoordinates);
  int SetResultingStates(DSL_stringArray &theStates);
  int SetResultingStates(const DSL_intArray &theStates);
  int SetResultingStates(const int* states);

  int GetResultingState(DSL_intArray &thisCoordinates) const;
  int GetResultingStates(DSL_intArray &here) const;
  int GetResultingStates(DSL_stringArray &here) const;

  int GetTemporalResultingStates(int order, DSL_intArray &states) const;
  int SetTemporalResultingStates(int order, const DSL_intArray &states);
  int SetTemporalResultingStates(int order, const int* states);
  
  void CheckConsistency(int deep = 0);

  int Clone(DSL_nodeDefinition &thisGuy);
  
  // get/set definitions in several ways
  int SetDefinition(DSL_intArray &withThis) {return(SetResultingStates(withThis));};
  int SetDefinition(DSL_stringArray &withThis) {return(SetResultingStates(withThis));};
  int MakeUniform() { return DSL_WRONG_NODE_TYPE; }
 
protected:
  int SetResultingState(DSL_intArray &thisCoordinates, DSL_intArray &theStates);
  int MakeConsistent();
};

#endif
