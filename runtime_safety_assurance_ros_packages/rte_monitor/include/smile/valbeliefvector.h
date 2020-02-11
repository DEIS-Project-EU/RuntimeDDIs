#ifndef DSL_VALBELIEFVECTOR_H
#define DSL_VALBELIEFVECTOR_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodeval.h"
#include "dmatrix.h"

//////////////////
// beliefVector //
//////////////////

class DSL_beliefVector : public DSL_nodeValue
{
public:
	DSL_beliefVector(int myHandle, DSL_network *theNetwork);
	DSL_beliefVector(DSL_nodeValue &likeThisOne);
	~DSL_beliefVector();

	int GetType() const { return DSL_BELIEFVECTOR; }

	void CleanUp(int deep = 0);
	int ReCreateFromNetworkStructure();

	int operator =(DSL_nodeValue &likeThisOne);
	int Clone(DSL_nodeValue &thisGuy);

	int GetEvidence() const { return IsEvidence() ? evidence : DSL_OUT_OF_RANGE; } 
	int SetEvidence(int theEvidence);
	int ClearEvidence();

	int SetPropagatedEvidence(int theEvidence);
	int ClearPropagatedEvidence();

	int SetVirtualEvidence(const std::vector<double> &evidence);
	int GetVirtualEvidence(std::vector<double> &evidence) const;

	DSL_Dmatrix &GetBeliefs() { return beliefs; }
	double GetBelief(int thisOne) { return beliefs[thisOne]; } 

	int SetBelief(int thisOne, double thisValue) {beliefs[thisOne] = thisValue; return DSL_OKAY;} 
	int SetBelief(DSL_intArray &theCoordinates, double theBelief) { beliefs.Subscript(theCoordinates) = theBelief; return DSL_OKAY; }

	//------------------------
	// TCL: manipulation 
	//------------------------
	int ControlValue(int theValue);
	int ClearControlledValue();
	int GetControlledValue() {return IsControlled() ? evidence : DSL_OUT_OF_RANGE; }
	int IsControlled() { return IsFlagSet(DSL_VALUE_CONTROLLED) && IsEvidence(); }
	bool IsControllable();

	// common interface
	void *GetValue() { return &GetBeliefs(); }
	int GetValue(DSL_Dmatrix **here) { *here = &GetBeliefs(); return DSL_OKAY; }
	int SetValue(DSL_Dmatrix &withThis) { return beliefs = withThis; }

private:
	int StoreEvidence(int theEvidence); 
	void ClearVirtualEvidence();

	DSL_Dmatrix  beliefs;
	int evidence;
	std::vector<double> *virtualEvidence;
};

#endif
