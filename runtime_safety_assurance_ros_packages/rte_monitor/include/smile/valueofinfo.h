#ifndef VALUEOFINFORMATION_H
#define VALUEOFINFORMATION_H

// {{SMILE_PUBLIC_HEADER}}

#include "dslobject.h"
#include "dmatrix.h"


class DSL_network;

class DSL_valueOfInformation : public DSL_object
{
 protected:
  DSL_network  *theNetwork; 

  DSL_intArray theDecisionNodes;      // decision nodes in temporal order
  DSL_intArray theTemporalActions;    // actions in temporal order

  // the data
  DSL_intArray  theNodes;         // nodes we want to know the value about
  int           theDecision;      // value of observing [theNodes] before [theDecision]
  int           pointOfView;      // calculated from this [pointOfView] 

  // the results
  DSL_intArray  theIndexingNodes;
  DSL_Dmatrix   theValues;

 public:
  DSL_valueOfInformation(DSL_network *thisOne);
 ~DSL_valueOfInformation(void) {CleanUp();};

  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);
  void CheckConsistency(int deep = 0);

  DSL_network  *GetNetwork(void)          {return(theNetwork);};
  DSL_intArray &GetNodes(void)            {return(theNodes);};
  int           GetDecision(void)      {return(theDecision);};
  int           GetPointOfView(void)   {return(pointOfView);};
  DSL_intArray &GetIndexingNodes(void) {return(theIndexingNodes);};
  DSL_Dmatrix  &GetValues(void)        {return(theValues);};

  int SetNetwork(DSL_network *thisOne);
  int AddNode(int thisNode);
  int RemoveNode(int thisNode);
  int SetDecision(int thisOne);
  int SetPointOfView(int thisOne);

  DSL_intArray &GetDecisions(void) {return(theDecisionNodes);};
  DSL_intArray &GetActions(void) {return(theTemporalActions);};

 protected:
  int ReCreateFromNetworkStructure(void);
  int FindTemporalActions(void);
  int FindAllDecisionNodes(void);
};

#endif
