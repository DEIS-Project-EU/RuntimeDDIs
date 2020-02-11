#ifndef DEFLIST_H
#define DEFLIST_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodedef.h"
#include "idarray.h"
#include "doublearray.h"

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//   LIST
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

// minimun number of choices
#define DSL_LIST_MIN_CHOICES 2 

class DSL_list : public DSL_nodeDefinition
{
 // represents a discrete & deterministic decision node.
 protected:
  int             numChoices;   // number of possible choices for the node
  DSL_idArray     nameChoices;  // names (labels) for all the posible choices
  DSL_doubleArray valueChoices; // numerical value of the choices (for continuous children)

 public:
  // creation/destruction
  DSL_list(int myHandle, DSL_network *theNetwork);
  DSL_list(DSL_nodeDefinition &likeThisOne);
  virtual ~DSL_list() {CleanUp();};

  int operator=(DSL_nodeDefinition &likeThisOne);

  int GetType() const { return DSL_LIST; }
  const char *GetTypeName() const { return "LIST"; }

 // access methods
  DSL_idArray     &GetChoicesNames(void) {return(nameChoices);};
  DSL_doubleArray &GetChoicesValues(void) {return(valueChoices);};

  int GetSize(void) {return(numChoices);};
  int GetNumberOfChoices() const { return numChoices; }
  int RenameChoice(int choiceNumber, const char *theChoice);
  int RenameChoices(DSL_stringArray &theChoicesNames);
  // valueChoices
  int ChangeChoiceValue(int choiceNumber, double theValue);
  int ChangeChoicesValues(DSL_doubleArray &theChoicesValues);
  // methods to deal with changes in my definition
  int AddChoice(const char *theChoice);
  int InsertChoice(int here, const char *theChoice);
  int RemoveChoice(int choiceNumber);
  int SetNumberOfChoices(int aNumber);
  int SetNumberOfChoices(DSL_stringArray &theChoicesNames);
  int ChangeOrderOfChoices(DSL_intArray &newOrder);

  // methods to deal with changes in my relationships
  int AddParent(int theParent);
  int RemoveParent(int theParent) {return(DSL_OKAY);};
  // methods to deal with changes in the definition of my parents
  int DaddyGetsBigger(int daddy, int thisPosition) {return(DSL_OKAY);};
  int DaddyGetsSmaller(int daddy, int thisPosition) {return(DSL_OKAY);};
  int DaddyChangedOrderOfOutcomes(int daddy, DSL_intArray &newOrder) {return(DSL_OKAY);};

  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);
  void CheckConsistency(int deep = 0);

  int CreateUniqueChoiceName(char *here);

  int Clone(DSL_nodeDefinition &thisGuy);
  int ReCreateFromNetworkStructure(void);

  int CheckParentsStructure(void);

 protected:
  int DSL_stringToDouble(char *theString, double &theValue);
  int iRenameChoice(int here, char *theChoice);
  int NumerizeChoice(int thisChoice);

 // common interface
 public:
  int AddOutcome(const char *thisName)                            {return(AddChoice(thisName));}
  int InsertOutcome(int here, const char *thisName)               {return(InsertChoice(here,thisName));}
  int RemoveOutcome(int outcomeNumber)                      {return(RemoveChoice(outcomeNumber));}
  int GetNumberOfOutcomes() const { return GetNumberOfChoices(); } 
  int RenameOutcome(int outcomeNumber, const char *newName)       {return(RenameChoice(outcomeNumber,newName));}
  int RenameOutcomes(DSL_stringArray &theOutcomeNames)      {return(RenameChoices(theOutcomeNames));}
  DSL_idArray *GetOutcomesNames(void)                       {return(&GetChoicesNames());}
  int SetNumberOfOutcomes(int aNumber)                      {return(SetNumberOfChoices(aNumber));}
  int SetNumberOfOutcomes(DSL_stringArray &theOutcomeNames) {return(SetNumberOfChoices(theOutcomeNames));}
  int ChangeOrderOfOutcomes(DSL_intArray &newOrder)         {return(ChangeOrderOfChoices(newOrder));}
  
  int GetDefinition(DSL_stringArray **here) {*here = &GetChoicesNames(); return(DSL_OKAY);}
  int GetDefinition(DSL_doubleArray **here) {*here = &GetChoicesValues(); return(DSL_OKAY);}
  int SetDefinition(DSL_doubleArray &withThis) {GetChoicesValues() = withThis; return(DSL_OKAY);}
  int SetDefinition(DSL_stringArray &withThis) {GetChoicesNames() = withThis; return(DSL_OKAY);}
};


#endif



