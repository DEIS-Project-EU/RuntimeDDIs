#ifndef SUBMODEL_H
#define SUBMODEL_H

// {{SMILE_PUBLIC_HEADER}}

#include "network.h"

class DSL_node;
class DSL_intArray;

#define DSL_NO_PARENT     (-8888)        // only possible with main submodel
#define DSL_EMPTY_SLOT    (-8889)        
#define DSL_SUBMODEL_MASK 0x4000       // all but 15th bit set to 0


class DSL_submodel : public DSL_object
{
 public:
  DSL_header      header;       // id, comment, etc
  DSL_screenInfo  info;         // submodel as icon
  DSL_rectangle   windowpos;    // submodel as window
  int             bkcolor;      // submodel as window (background color)
  std::vector<std::pair<std::string, DSL_rectangle> > comments; // comments on screen;

public:
  DSL_submodel() { bkcolor = DSL_UNDEFINED_INT; CheckReadiness(); }
  DSL_submodel(DSL_submodel &likeThisOne);
  int operator =(DSL_submodel &likeThisOne);
  int CopyAppearance(DSL_submodel &likeThisOne);
};

////////////////////////////////////////////////////
// class DSL_submodelEntry
////////////////////////////////////////////////////

class DSL_submodelEntry
{
 public:
  DSL_submodel *submodel;
  int           parent;
 public:
  DSL_submodelEntry(void)   {submodel = NULL; parent = DSL_EMPTY_SLOT;};
  int IsEmpty(void)         {return(parent == DSL_EMPTY_SLOT);};
  int IsMainSubmodel(void)  {return(parent == DSL_NO_PARENT);};
};
 

////////////////////////////////////////////////////
// class DSL_submodel
////////////////////////////////////////////////////
#define DSL_MAIN_SUBMODEL (DSL_SUBMODEL_MASK) // 0 is the main submodel

// From now on, [Handler] always make reference to a submodel [Handler]
// (i.e., an integer that uniquely identifies each submodel

class DSL_submodelHandler : public DSL_object
{
 protected:
  int                maxSubmodels;
  DSL_submodelEntry *submodels;    
  DSL_network       *network;  // the network I belong to
  int                delta;
  DSL_screenInfo     defaultNodeValues; // default info for node
  DSL_screenInfo     defaultSubmodelValues; // default info for submodel

 public:
  DSL_submodelHandler(void);
 ~DSL_submodelHandler() {CleanUp();};
  int operator =(const DSL_submodelHandler &likeThisOne);
  void SetNetwork(DSL_network *theNetwork) {network = theNetwork; CheckReadiness();};
  int Initialize(void);
  int IsThisIdentifierInUse(const char *theId);
  int Flush(void);
  int CreateSubmodel(int theParent, const char *thisId);
  int DeleteSubmodel(int theSubmodel);
  int Ok(int index) {OI(index); return(iOk(index));}; 
  int GetParent(int ofThisSubmodel);
  DSL_submodel *GetSubmodel(int handler);
  int GetFirstSubmodel(void);
  int GetNextSubmodel(int ofThisSubmodel);
  int FindSubmodel(const char *withThisID);
  int MoveSubmodel(int thisSubmodel, int intoThisSubmodel);
  int GetSubmodelsOfManyNodes(DSL_intArray &theNodes, DSL_intArray &here);
  void IdentifyArcs(int obj1, int obj2, dsl_arcType layer, DSL_intArray &parents, DSL_intArray &children);
  int IsMainSubmodel(int handle) {return(handle == DSL_MAIN_SUBMODEL);};
  int PartialOrdering(DSL_intArray &here);
  int GetNumberOfSubmodels(void);
  DSL_screenInfo &GetNodeDefaultValues() { return defaultNodeValues; }
  DSL_screenInfo &GetSubmodelDefaultValues() { return defaultSubmodelValues; }

  // INTERFACE FUNCTIONS (MAKE TRANSLATION IN/OUT)
  int GetIncludedSubmodels(int inThisSubmodel, DSL_intArray &here);
  int GetIncludedNodes(int inThisSubmodel, DSL_intArray &here);
  int GetDeepIncludedSubmodels(int inThisSubmodel, DSL_intArray &here);
  int GetDeepIncludedNodes(int inThisSubmodel, DSL_intArray &here);
  int GetParentSubmodels(int ofThisGuy, DSL_intArray &here, dsl_arcType layer);
  int GetChildSubmodels(int ofThisGuy, DSL_intArray &here, dsl_arcType layer);
  int GetParentNodes(int ofThisGuy, DSL_intArray &here, dsl_arcType layer);
  int GetChildNodes(int ofThisGuy, DSL_intArray &here, dsl_arcType layer);
  int GetOutsideParentNodes(int ofThisGuy, DSL_intArray &here, dsl_arcType layer);
  int GetOutsideChildNodes(int ofThisGuy, DSL_intArray &here, dsl_arcType layer);

  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);

  int ReCreate(DSL_network *theNetwork);

 protected:
  int iOk(int index) {return(index>=0 && index<maxSubmodels && !submodels[index].IsEmpty());}; 
  int IsEmpty(int thisSlot) {return(submodels[thisSlot].IsEmpty());};
  int IsSubmodel(int index) {return(index & DSL_SUBMODEL_MASK);};
  int FindEmptySlot(void);
  int Grow(void);
  int IsIncluded(int thisSubmodel, int intoThisSubmodel);
  int CreateMainSubmodel(void);
  int CreateUniqueSubmodelIdentifier(char *here, int tryThis);
  int GetNodeSubmodel(int thisNode);
  int IO(int &aSubmodel) {return(aSubmodel |= DSL_SUBMODEL_MASK);};
  int OI(int &aSubmodel) {return(aSubmodel &= ~DSL_SUBMODEL_MASK);};
  // INTERNAL FUNCTIONS
  int iCreateSubmodel(int theParent, const char *thisId);
  int iDeleteSubmodel(int theSubmodel);
  int iGetParent(int ofThisSubmodel);
  DSL_submodel *iGetSubmodel(int handler);
  int iGetFirstSubmodel(void);
  int iGetNextSubmodel(int ofThisSubmodel);
  int iFindSubmodel(const char *withThisID);
  int iMoveSubmodel(int thisSubmodel, int intoThisSubmodel);
  int iGetSubmodelsOfManyNodes(DSL_intArray &theNodes, DSL_intArray &here);
  int FillMainSubmodelDefaultValues(void);
  int iFillDefaultValues(int intoThisSubmodel, int fromThisSubmodel);
  /////////   RELATIONSHIPS
  int iGetIncludedSubmodels(int inThisSubmodel, DSL_intArray &here);
  int iGetIncludedNodes(int inThisSubmodel, DSL_intArray &here);
  int iGetDeepIncludedSubmodels(int inThisSubmodel, DSL_intArray &here);
  int iGetDeepIncludedNodes(int inThisSubmodel, DSL_intArray &here);
  // parent submodels
  int iGetParentSubmodelsOfNode(int theNode, DSL_intArray &here, dsl_arcType layer/* = dsl_normalArc*/);
  int iGetParentSubmodelsOfSubmodel(int theSubmodel, DSL_intArray &here);
  // child submodels
  int iGetChildSubmodelsOfNode(int theNode, DSL_intArray &here, dsl_arcType layer);
  int iGetChildSubmodelsOfSubmodel(int theSubmodel, DSL_intArray &here, dsl_arcType layer);
  // parent nodes
  int iGetParentNodesOfNode(int theNode, DSL_intArray &here, dsl_arcType layer);
  int iGetParentNodesOfSubmodel(int theSubmodel, DSL_intArray &here);
  // child nodes
  int iGetChildNodesOfNode(int theNode, DSL_intArray &here, dsl_arcType layer);
  int iGetChildNodesOfSubmodel(int theSubmodel, DSL_intArray &here);
  // outside parent nodes
  int iGetOutsideParentNodesOfNode(int theNode, DSL_intArray &here, dsl_arcType layer);
  int iGetOutsideParentNodesOfSubmodel(int theSubmodel, DSL_intArray &here, dsl_arcType layer);
  // outside child nodes
  int iGetOutsideChildNodesOfNode(int theNode, DSL_intArray &here, dsl_arcType layer);
  int iGetOutsideChildNodesOfSubmodel(int theSubmodel, DSL_intArray &here, dsl_arcType layer);
};


#endif

