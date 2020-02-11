#ifndef DEFMAU_H
#define DEFMAU_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodedef.h"
#include "dmatrix.h"
#include <vector>

class DSL_expression;

template <class T, class Policy> class DSL_Tmatrix;
class DSL_EmatrixPolicy;
typedef DSL_Tmatrix<DSL_expression, DSL_EmatrixPolicy> DSL_Ematrix;

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//   MAU  (multi-attribute utility)
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

// represents a discrete & deterministic utility node with contiunous parents
class DSL_mau : public DSL_nodeDefinition
{
public:
	// creation/destruction
	DSL_mau(int myHandle, DSL_network *theNetwork);
	DSL_mau(DSL_nodeDefinition &likeThisOne);
	~DSL_mau() { CleanUp(); }

	int operator=(DSL_nodeDefinition &likeThisOne);

	int GetType() const { return DSL_MAU; }
	const char *GetTypeName() const { return "MAU"; }

	const DSL_Dmatrix &GetWeights() const { return weights; }

	int GetSize() { return weights.GetSize(); }
	int SetWeights(const DSL_Dmatrix &newWeights);

	const DSL_Ematrix* GetExpressions() const { return expressions; }
	int GetExpressions(std::vector<std::string> &e) const;
	int SetExpressions(const DSL_Ematrix *e, std::string *errMsg = NULL);
	int SetExpressions(const std::vector<std::string> &e, std::string *errMsg = NULL);
	bool ValidateExpression(const std::string &str, std::string &errMsg, int *errPos = NULL);
	void InitExpressionMatrix(DSL_Ematrix *mtx);
	
	// methods to deal with changes in my relationships
	int AddParent(int theParent);
	int RemoveParent(int theParent);

	void CleanUp(int deep = 0);
	void CheckReadiness(int deep = 0);

	int Clone(DSL_nodeDefinition &thisGuy);
	int ReCreateFromNetworkStructure();

	int CheckParentsStructure();

	int GetDefinition(DSL_Dmatrix **here) {*here = &weights; return(DSL_OKAY);}
	int GetDefinition(DSL_doubleArray **here) { *here = &weights.GetItems(); return DSL_OKAY; }
	int SetDefinition(DSL_doubleArray &withThis) { weights.GetItems().FillFrom(withThis); return DSL_OKAY; }
	int SetDefinition(DSL_Dmatrix &withThis)
	{
		assert(weights.CompatibleWith(withThis));
		weights = withThis; 
		return DSL_OKAY;
	}

	void GetParentsSplitByType(DSL_intArray &decisions, DSL_intArray &utilities) 
	{
		GetDecisionParentsHelper(decisions, true);
		GetDecisionParentsHelper(utilities, false);
	}

    int ValidateExpressions(
        const DSL_extFunctionContainer &extFxn,
        int &errFxnIdx,
        std::string &errMsg) const;

    int PatchExpressions(const DSL_extFunctionContainer &extFxn);

private:
	int DaddyGetsBigger(int parent, int position);
	int DaddyGetsSmaller(int parent, int position);
	int DaddyChangedOrderOfOutcomes(int parent, DSL_intArray &newOrder);
	int OrderOfParentsGetsChanged(DSL_intArray &newOrder);
	void ParentIdChanged(int parentHandle, const char *oldId, const char *newId);
	void GetDecisionParentsHelper(DSL_intArray &parents, bool decisions) const;
	void AfterExpressionMatrixShrink();

	// This node is expecting continous entries from it's utility/MAU parents
	// but may be indexed by discrete decision nodes
	DSL_Dmatrix weights;    // the weights of each parent for simple linear combination
	DSL_Ematrix *expressions; // generalized expressions
};

#endif
