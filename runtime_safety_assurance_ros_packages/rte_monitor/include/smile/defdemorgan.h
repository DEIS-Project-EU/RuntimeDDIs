#ifndef DEFDEMORGAN_H
#define DEFDEMORGAN_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodedef.h"
#include "dmatrix.h"
#include "doublearray.h"

// different types of parents

#define DSL_DEMORGAN_INHIBITOR     0
#define DSL_DEMORGAN_REQUIREMENT   1
#define DSL_DEMORGAN_CAUSE         2
#define DSL_DEMORGAN_BARRIER       3

class DSL_demorgan : public DSL_nodeDefinition 
{
public:
	DSL_demorgan();
	DSL_demorgan(int myHandle, DSL_network *theNetwork);
	DSL_demorgan(DSL_nodeDefinition &likeThisOne);
	virtual ~DSL_demorgan() { CleanUp(); }

	int operator =(DSL_nodeDefinition &likeThisOne);
	
	int AddParent(int theParent);
	int RemoveParent(int theParent);
	void CheckReadiness(int deep = 0); 
	int ReCreateFromNetworkStructure();
	int Clone(DSL_nodeDefinition &thisGuy);
	void CleanUp(int deep=0);
	DSL_idArray* GetOutcomesNames();
	
	int SetParentWeights(DSL_doubleArray& weights);
	int SetParentWeight(int numParent, double value);
	double GetParentWeight(int numParent);
	int SetParentTypes(const DSL_intArray& types);	
	int SetParentType(int numParent, int value);
	int	GetParentType(int numParent);
	
	double GetPriorBelief() { return prior_belief; }
	int SetPriorBelief(double value);
		
	const char *GetTypeName() const { return "DEMORGAN"; }
	int GetType() const { return DSL_DEMORGAN; }			
	int GetSize() { return table.GetSize(); }
	int GetNumberOfOutcomes() const { return 2; }
    int OrderOfParentsGetsChanged(DSL_intArray &newOrder);

	int GetDefinition(DSL_Dmatrix **here) { *here=&table; return DSL_OKAY; }
	int GetDefinition(DSL_doubleArray **here) { *here=&table.GetItems(); return DSL_OKAY; }

	DSL_Dmatrix &GetProbabilities() { return table;		}
	DSL_intArray &GetParentTypes() { return parent_types; }
	DSL_doubleArray &GetParentWeights()	{ return parent_weights; }

    void DoCopyParameters(DSL_nodeDefinition &target) const;

private:
	int ResizeTable();
	void UpdateTable();
	void UpdateTableColumn(int index); 

	DSL_Dmatrix table;				// stores CPT
	DSL_intArray parent_types;		// stores the type of each parent
	DSL_doubleArray parent_weights;	// stores the weight of each parent
	double prior_belief;			// stores the prior belief for this node
};

#endif
