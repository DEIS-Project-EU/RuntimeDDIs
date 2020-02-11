#ifndef VALLISTOFDECISIONS_H
#define VALLISTOFDECISIONS_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodeval.h"
#include "dmatrix.h"

/////////////////////
// listOfDecisions //
/////////////////////

class DSL_listOfDecisions : public DSL_nodeValue
{
 protected:
  DSL_Dmatrix thePolicyValues;  // expected utilities of the policies
  int decision;

 public:
  DSL_listOfDecisions(int myHandle, DSL_network *theNetwork);
  DSL_listOfDecisions(DSL_nodeValue &likeThisOne);
 ~DSL_listOfDecisions(void) {CleanUp();};

  int GetType() const { return DSL_LISTOFDECISIONS; }

  // access methods
  DSL_Dmatrix &GetPolicyValues(void) {return(thePolicyValues);};

  int operator =(DSL_nodeValue &likeThisOne);
  int Clone(DSL_nodeValue &thisGuy);

  int SetEvidence(int theEvidence);
  int ClearEvidence(void);
  int SetPropagatedEvidence(int theEvidence);
  int ClearPropagatedEvidence(void);

  int GetEvidence() const { return IsEvidence() ? decision : DSL_OUT_OF_RANGE; }

  int SetPolicyValue(DSL_intArray &theCoordinates, double theValue) {thePolicyValues.Subscript(theCoordinates) = theValue; return(DSL_OKAY);};
  int GetPolicyValue(DSL_intArray &theCoordinates, double &here) {return((int)(here = thePolicyValues.Subscript(theCoordinates)));};

  void CleanUp(int deep = 0);

  int ReCreateFromNetworkStructure(void);

 // common interface
 public:
  void *GetValue(void)              {return(&GetPolicyValues());};
  int GetValue(DSL_Dmatrix **here)        {*here = &GetPolicyValues(); return(DSL_OKAY);};
  int SetValue(DSL_Dmatrix &withThis)     {return(thePolicyValues = withThis);};

 protected:
  int StoreEvidence(int theEvidence); 
  int CleanEvidence(void); 
};

#endif

