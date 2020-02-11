#ifndef NODECOST_H
#define NODECOST_H

// {{SMILE_PUBLIC_HEADER}}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//   NODE COST 
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

#include "dslobject.h"
#include "dmatrix.h"

class DSL_network;

class DSL_nodeCost : public DSL_object
{
 // represents the cost of doing something with a node
 // It can be used for observation costs, manipulations costs,
 // etc

 // ENTRY VALUES:
 // This node is expecting DISCRETE entries from it's parents
 // wich means that all it's parents should be discrete (or 
 // discretized if they are continuous)
 protected:
  DSL_network  *network;
  int           handle;
  DSL_Dmatrix   table;      // the table with the costs

 public:
  // creation/destruction
  DSL_nodeCost(int myHandle, DSL_network *theNetwork);
  DSL_nodeCost(DSL_nodeCost &likeThisOne);
  virtual ~DSL_nodeCost() {CleanUp();};

  int operator =(DSL_nodeCost &likeThisOne);

  // member access
  DSL_network  *Network(void) {return(network);};
  int           SetNetwork(DSL_network *thisOne);
  int           Handle(void)  {return(handle);};
  void SetNetworkAndHandle(DSL_network *network, int handle);
  DSL_Dmatrix     &GetCosts(void)       {return(table);};

  int GetSize(void) {return(table.GetSize());};
  int SetCosts(DSL_doubleArray &theCosts);
  int SetCost(DSL_intArray &thisCoordinates, double theCost);
  int GetCost(DSL_intArray &thisCoordinates, double &theCost);

  // methods to deal with changes in my relationships
  int AddParent(int theParent);
  int AbsorbEvidenceFromParent(int theParent);
  int MarginalizeParent(int theParent);
  int RemoveParent(int theParent);
  // methods to deal with changes in the definition of my parents
  int DaddyGetsBigger(int daddy, int thisPosition);
  int DaddyGetsSmaller(int daddy, int thisPosition);
  

  // methods to deal with direct changes in my definition
  virtual int BeforeDirectChange(int changeType = 0);
  virtual int AfterDirectChange(int changeType = 0);

  // methods for internal use (no checking at all)
  //void SetCost(int thisOne, double theCost) {table[thisOne] = theCost;};
  //double GetCost(int thisOne) {return(table[thisOne]);};

  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);

  int Clone(DSL_nodeCost &thisGuy);
  int ReCreateFromNetworkStructure(void);

  int CheckParentsStructure(void);

  int NormalizeExpectedCosts(double origMinVal, double origMaxVal, double destMinVal, double destMaxVal);
  double GetMinimumCost(void);
  double GetMaximumCost(void);

  // Bug Fix Adam 10/21/04
  int DaddyChangedOrderOfOutcomes(int daddy, DSL_intArray &newOrder);
 public:
  // get/set definitions in several ways
  int GetDefinition(DSL_Dmatrix **here) {*here = &GetCosts(); return(DSL_OKAY);};
  int GetDefinition(DSL_doubleArray **here) {*here = &GetCosts().GetItems(); return(DSL_OKAY);};
  int SetDefinition(DSL_Dmatrix &withThis) {table = withThis; return(DSL_OKAY);};
  int SetDefinition(DSL_doubleArray &withThis) {table.GetItems().FillFrom(withThis); return(DSL_OKAY);};
};

#endif
