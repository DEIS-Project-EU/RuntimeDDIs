#ifndef DSL_DEFCPT_H
#define DSL_DEFCPT_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodedef.h"
#include "dmatrix.h"
#include "idarray.h"

//-------------------------------------------------------------------------
// CPT
//-------------------------------------------------------------------------

// minimum number of states
#define DSL_CPT_MIN_STATES 2 

#define DSL_CPT_EPSILON    0.000005 

class DSL_cpt : public DSL_nodeDefinition
{
 // represents a discrete & non-deterministic chance node.

 // ENTRY VALUES:
 // This node is expecting DISCRETE entries from it's parents
 // wich means that all it's parents should be discrete (or 
 // discretized if they are continuous)
 public:
  DSL_cpt(int myHandle, DSL_network *theNetwork);
  DSL_cpt(DSL_nodeDefinition &likeThisOne);
  virtual ~DSL_cpt() { CleanUp(); }

  int operator =(DSL_nodeDefinition &likeThisOne);
  int GetType() const { return DSL_CPT; }
  const char *GetTypeName() const { return "CPT"; }

  // access methods
  DSL_idArray   &GetStatesNames()   {return(nameStates);}
  DSL_Dmatrix   &GetProbabilities() {return(table);}

  const DSL_Dmatrix* GetTemporalProbabilities(int order) const;
  int SetTemporalProbabilities(int order, const DSL_Dmatrix &probs);
  int SetTemporalProbabilities(int order, const DSL_doubleArray &probs);

  int GetSize() {return(table.GetSize());}
  int GetNumberOfStates() const { return nameStates.NumItems(); }
  int SetProbabilities(DSL_doubleArray &theProbabilities);
  int SetProbability(DSL_intArray &theCoordinates, double theProbability);
  int GetProbability(DSL_intArray &theCoordinates, double &theProbability);
  int RenameState(int stateNumber, const char *newName);
  int RenameStates(DSL_stringArray &theStateNames);
  int NextCoordinates(DSL_intArray &theCoordinates);
  int PrevCoordinates(DSL_intArray &theCoordinates);
  // methods to deal with changes in my definition
  int AddState(const char *thisName);
  int InsertState(int here, const char *thisName);
  int RemoveState(int stateNumber);
  int SetNumberOfStates(int aNumber);
  int SetNumberOfStates(DSL_stringArray &theStateNames);
  int ChangeOrderOfStates(DSL_intArray &newOrder);
  
  // methods to deal with changes in my relationships
  int AddParent(int theParent);
  int AbsorbEvidenceFromParent(int theParent);
  int MarginalizeParent(int theParent);
  int RemoveParent(int theParent);
  // methods to deal with changes in the definition of my parents
  int DaddyGetsBigger(int daddy, int thisPosition);
  int DaddyGetsSmaller(int daddy, int thisPosition);
  int DaddyChangedOrderOfOutcomes(int daddy, DSL_intArray &newOrder);
  int OrderOfParentsGetsChanged(DSL_intArray &newOrder);
  
  // methods for internal use (no checking at all)
  void SetProbability(int thisOne, double theProbability) {table[thisOne] = theProbability;}
  double GetProbability(int thisOne) {return(table[thisOne]);}

  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);
  void CheckConsistency(int deep = 0);

  int CreateUniqueStateName(char *here);

  int ReCreateFromNetworkStructure();
  int Clone(DSL_nodeDefinition &thisGuy);

  int CheckParentsStructure();

  
  
 
  // common interface
   int AddOutcome(const char *thisName)                            {return(AddState(thisName));}
  int InsertOutcome(int here, const char *thisName)               {return(InsertState(here,thisName));}
  int RemoveOutcome(int outcomeNumber)                      {return(RemoveState(outcomeNumber));}
  int GetNumberOfOutcomes() const { return GetNumberOfStates(); }
  int RenameOutcome(int outcomeNumber, const char *newName)       {return(RenameState(outcomeNumber,newName));}
  int RenameOutcomes(DSL_stringArray &theOutcomeNames)      {return(RenameStates(theOutcomeNames));}
  DSL_idArray *GetOutcomesNames()                       {return(&GetStatesNames());}
  int SetNumberOfOutcomes(int aNumber)                      {return(SetNumberOfStates(aNumber));}
  int SetNumberOfOutcomes(DSL_stringArray &theOutcomeNames) {return(SetNumberOfStates(theOutcomeNames));}
  int ChangeOrderOfOutcomes(DSL_intArray &newOrder)         {return(ChangeOrderOfStates(newOrder));}
  
  // get/set definitions in several ways
  int GetDefinition(DSL_Dmatrix **here) {*here = &GetProbabilities(); return(DSL_OKAY);}
  int GetDefinition(DSL_doubleArray **here) {*here = &GetProbabilities().GetItems(); return(DSL_OKAY);}
  int SetDefinition(DSL_Dmatrix &withThis) 
  {
	  assert(table.CompatibleWith(withThis));
	  table = withThis; 
	  return DSL_OKAY;
  }
  int SetDefinition(DSL_doubleArray &withThis) {table.GetItems().FillFrom(withThis); return(DSL_OKAY);}

  int MakeUniform();

protected:
  void DoCopyParameters(DSL_nodeDefinition &target) const;
    
  DSL_idArray  nameStates; // names (labels) for all the states of the node 
  DSL_Dmatrix  table;      // the table of probabilities
};

#endif
