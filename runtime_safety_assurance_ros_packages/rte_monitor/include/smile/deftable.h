#ifndef DEFTABLE_H
#define DEFTABLE_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodedef.h"
#include "dmatrix.h"

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//   TABLE
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

class DSL_table : public DSL_nodeDefinition
{
 // represents a discrete & deterministic utility node.

 // ENTRY VALUES:
 // This node is expecting DISCRETE entries from it's parents
 // wich means that all it's parents should be discrete (or 
 // discretized if they are continuous)
 protected:
  DSL_Dmatrix      table;      // the table with the utilities

 public:
  // creation/destruction
  DSL_table(int myHandle, DSL_network *theNetwork);
  DSL_table(DSL_nodeDefinition &likeThisOne);
  virtual ~DSL_table() {CleanUp();};

  int operator =(DSL_nodeDefinition &likeThisOne);

  int GetType() const { return DSL_TABLE; }
  const char* GetTypeName() const { return "TABLE"; }

  // access methods
  DSL_Dmatrix     &GetUtilities(void)       {return(table);};

  int GetSize(void) {return(table.GetSize());};
  int SetUtilities(DSL_doubleArray &theUtilities);
  int SetUtility(DSL_intArray &thisCoordinates, double theUtility);
  int GetUtility(DSL_intArray &thisCoordinates, double &theUtility);

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
  
  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);

  int Clone(DSL_nodeDefinition &thisGuy);
  int ReCreateFromNetworkStructure(void);

  int CheckParentsStructure(void);

  int NormalizeExpectedUtilities(double origMinVal, double origMaxVal, double destMinVal, double destMaxVal);

  double GetMinimumUtility() const;
  double GetMaximumUtility() const ;

  int IsParentNecessary(int parentIndex, double epsilon, bool &necessary);

public:
  int GetDefinition(DSL_Dmatrix **here) {*here = &GetUtilities(); return(DSL_OKAY);};
  int GetDefinition(DSL_doubleArray **here) {*here = &GetUtilities().GetItems(); return(DSL_OKAY);};
  int SetDefinition(DSL_Dmatrix &withThis) {table = withThis; return(DSL_OKAY);};
  int SetDefinition(DSL_doubleArray &withThis) {table.GetItems().FillFrom(withThis); return(DSL_OKAY);};
};

#endif
