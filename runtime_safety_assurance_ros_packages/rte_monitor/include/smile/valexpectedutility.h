#ifndef VALEXPECTEDUTILITY_H
#define VALEXPECTEDUTILITY_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodeval.h"
#include "dmatrix.h"

class DSL_expectedUtility : public DSL_nodeValue
{
public:
	DSL_expectedUtility(int myHandle, DSL_network *theNetwork);
	DSL_expectedUtility(DSL_nodeValue &likeThisOne);
	~DSL_expectedUtility() {CleanUp();}

	int GetType() const { return DSL_EXPECTEDUTILITY; }

	// access methods
	DSL_Dmatrix &GetUtilities() {return(theUtilities);}

	int operator =(DSL_nodeValue &likeThisOne);
	int Clone(DSL_nodeValue &thisGuy);

	int SetUtility(DSL_intArray &theCoordinates, double theUtility) {theUtilities.Subscript(theCoordinates) = theUtility; return(DSL_OKAY);}
	int GetUtility(DSL_intArray &theCoordinates, double &here) {return((int)(here = theUtilities.Subscript(theCoordinates)));}

	void CleanUp(int deep = 0);

	int ReCreateFromNetworkStructure();

	int NormalizeExpectedUtilities(double origMinVal, double origMaxVal, double destMinVal, double destMaxVal);
	double GetMinimumUtility();
	double GetMaximumUtility();

	/////////// REVISE THIS !!!!!!!!!!!!! REMOVE LATER REMOVE LATER
	int SetUtility(int thisOne, double thisValue) {theUtilities[thisOne] = thisValue; return(DSL_OKAY);} 
	int GetUtility(int thisOne, double &here) {here = theUtilities[thisOne]; return(DSL_OKAY);} ;

	void *GetValue()              {return(&GetUtilities());}
	int GetValue(DSL_Dmatrix **here)        {*here = &GetUtilities(); return(DSL_OKAY);}
	int SetValue(DSL_Dmatrix &withThis)     {return(theUtilities = withThis);}

private:
	DSL_Dmatrix  theUtilities; // holds the expected utility for each policy
};

#endif
