#ifndef VALMAUEXPECTEDUTILITY_H
#define VALMAUEXPECTEDUTILITY_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodeval.h"
#include "dmatrix.h"

class DSL_mauExpectedUtility : public DSL_nodeValue
{
public:
	DSL_mauExpectedUtility(int myHandle, DSL_network *theNetwork);
	DSL_mauExpectedUtility(DSL_nodeValue &likeThisOne);
	~DSL_mauExpectedUtility() {CleanUp();}

	int GetType() const { return DSL_MAUEXPECTEDUTILITY; }

	void CleanUp(int deep = 0);
	int ReCreateFromNetworkStructure();

	DSL_Dmatrix &GetUtilities() { return theUtilities; }

	int operator =(DSL_nodeValue &likeThisOne);
	int Clone(DSL_nodeValue &thisGuy);

	void *GetValue() {return &GetUtilities(); }
	int GetValue(DSL_Dmatrix **here) {*here = &GetUtilities(); return DSL_OKAY; }
	int SetValue(DSL_Dmatrix &withThis) { return theUtilities = withThis; }

	int SetUtility(DSL_intArray &theCoordinates, double theUtility) { theUtilities.Subscript(theCoordinates) = theUtility; return DSL_OKAY; }
	int GetUtility(DSL_intArray &theCoordinates, double &here) { here = theUtilities.Subscript(theCoordinates); return DSL_OKAY; }
	int CalculateMAU();

private:
	int BuildCustomVector(int forThisNode, DSL_intArray &fromTheseCoords, DSL_intArray &here);
	double Utility(const DSL_doubleArray &theParentsUtilities, int startIndex);

	DSL_Dmatrix  theUtilities; // holds the expected utility for each policy
};

#endif
