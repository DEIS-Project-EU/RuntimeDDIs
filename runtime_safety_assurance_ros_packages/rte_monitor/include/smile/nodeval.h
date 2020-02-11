#ifndef DSL_NODEVAL_H
#define DSL_NODEVAL_H

// {{SMILE_PUBLIC_HEADER}}

#include "dslobject.h"
#include "intarray.h"
#include <vector>

/*
DEFINITIONS     ParentsDeterministic       VALUES

TRUTHTABLE            -                    BELIEF VECTOR
CPT                   -                    BELIEF VECTOR 

EQUATION            TRUE                   NUMBER
                    FALSE                  HISTOGRAM  

DISTRIBUTION          -                    HISTOGRAM
LIST                                       LIST OF NUMBERS (posible states) // not yet
RANGE                                      LIST OF NUMBERS (discretized range)


TABLE               TRUE                   NUMBER (or matrix of numbers)
EQUATION            FALSE                  NUMBER (value of the equation once we substitute the parameters) 
*/

// types of values
#define DSL_NOVALUE             0     // when the value is not yet created
#define DSL_BELIEFVECTOR        3
#define DSL_EXPECTEDUTILITY     4
#define DSL_LISTOFDECISIONS     6
#define DSL_MAUEXPECTEDUTILITY  7
#define DSL_EQUATIONEVALUATION  8
#define DSL_CONTINUOUSVAL		9

// flags 
#define DSL_VALUE_EVIDENCE             1       
#define DSL_VALUE_VALID                2
#define DSL_VALUE_EVIDENCE_PROPAGATED  4 
#define DSL_VALUE_CONTROLLED           8  // TCL: for manipulation      
#define DSL_VALUE_SOFT_EVIDENCE        16

#define DSL_VALUE_DEFAULT_FLAGS      0

class DSL_network;
class DSL_Dmatrix;

class DSL_nodeValue : public DSL_object
{
public:
  DSL_nodeValue(int myHandle, DSL_network *theNetwork);
  DSL_nodeValue(DSL_nodeValue &likeThisOne);
  DSL_nodeValue(void) {handle = -1; network = NULL; flags = DSL_VALUE_DEFAULT_FLAGS; CheckReadiness();}
  virtual ~DSL_nodeValue() {CleanUp();}

  DSL_network  *Network(void) {return(network);}
  int           SetNetwork(DSL_network *thisOne);
  void SetNetworkAndHandle(DSL_network *network, int handle);
  int           Handle(void)  {return(handle);}
  
  virtual int operator =(DSL_nodeValue &likeThisOne);
  virtual int Clone(DSL_nodeValue &thisGuy);
  void CleanUp(int deep = 0);

  virtual int ReCreateFromNetworkStructure(void) { return DSL_WRONG_NODE_TYPE; }

  // type
  virtual int GetType() const = 0;

  virtual int GetSize(void);

  const DSL_intArray& GetIndexingParents() const { return indexingParents; }
  virtual int AddIndexingParent(int theParent);

  void SetFlag(int thisOne) { flags |= thisOne; }
  void ClearFlag(int thisOne) { flags &= ~thisOne; }
  int GetAllFlags() { return flags; }
  
  int IsFlagSet(int thisOne) const { return flags & thisOne; } 

  int IsValueValid() { return IsFlagSet(DSL_VALUE_VALID); }
  void SetValueValid() { SetFlag(DSL_VALUE_VALID); lazyTag = NULL;  }
  void SetValueInvalid() { ClearFlag(DSL_VALUE_VALID); lazyTag = NULL; }
  
  void* GetLazyTag() const { return lazyTag;  }
  void SetLazyTag(void *lt) { lazyTag = lt; }

  virtual int GetEvidence() const { return DSL_WRONG_NODE_TYPE; } 
  virtual int GetEvidence(double &evidence) const { return DSL_WRONG_NODE_TYPE; } 
  virtual int SetEvidence(double evidence) { return DSL_WRONG_NODE_TYPE; } 
  virtual int SetEvidence(int evidence) { return DSL_WRONG_NODE_TYPE; } 
  virtual int ClearEvidence() { return DSL_WRONG_NODE_TYPE; } 
  virtual int ClearPropagatedEvidence() { return DSL_WRONG_NODE_TYPE; }
  virtual int SetPropagatedEvidence(int evidence) { return DSL_WRONG_NODE_TYPE; } 
  virtual int SetPropagatedEvidence(double evidence) { return DSL_WRONG_NODE_TYPE; } 
  virtual int GetVirtualEvidence(std::vector<double> &evidence) const { return DSL_WRONG_NODE_TYPE; }
  virtual int SetVirtualEvidence(const std::vector<double> &evidence) { return DSL_WRONG_NODE_TYPE; }

  // tomek 2003-08-05 - changed IsXXXEvidence to non-virtual after profiling
  // there are no derived classes which change the implementation of these methods,
  // and non-virtualness gives the opportunity for inlining to compiler
  int IsEvidence() const { return IsRealEvidence() || IsPropagatedEvidence(); }
  int IsRealEvidence() const { return IsFlagSet(DSL_VALUE_EVIDENCE); }
  int IsPropagatedEvidence() const { return IsFlagSet(DSL_VALUE_EVIDENCE_PROPAGATED); }
  int IsVirtualEvidence() const { return IsFlagSet(DSL_VALUE_SOFT_EVIDENCE); }

  // get/set values; right now we only support DSL_Dmatrix
  virtual int GetValue(DSL_Dmatrix **here)        { return DSL_WRONG_NODE_TYPE; }
  virtual int SetValue(DSL_Dmatrix &withThis)     { return DSL_WRONG_NODE_TYPE; }

  DSL_Dmatrix* GetMatrix() { DSL_Dmatrix *m = NULL; GetValue(&m); return m;  }

  bool HasTemporalEvidence() const;
  bool IsTemporalEvidence(int slice) const;
  bool IsTemporalVirtualEvidence(int slice) const;
  int GetTemporalEvidence(int slice) const;
  int GetTemporalEvidence(int slice, std::vector<double> &evidence) const;
  int SetTemporalEvidence(int slice, int evidence);
  int SetTemporalEvidence(int slice, const std::vector<double> &evidence);
  int ClearTemporalEvidence(int slice);

protected:
  DSL_network  *network;
  int           handle;
  int           flags;
  DSL_intArray  indexingParents;  // handles of the parents indexing the values
  void *lazyTag; // initialized to NULL and set back to NULL whenever valid flag changes
};

#endif
