#ifndef DSL_DEFEQUATION_H
#define DSL_DEFEQUATION_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodedef.h"
#include "generalequation.h"
#include <string>

// represents an equation node which can only have
// DSL_equation as its parents.

class DSL_equation : public DSL_nodeDefinition
{
public:
	typedef std::vector<std::pair<std::string, double> > IntervalVector;

    DSL_equation(int myHandle, DSL_network *theNetwork);
	DSL_equation(DSL_nodeDefinition &that);
	~DSL_equation();

    int GetType() const { return DSL_EQUATION; }
    const char* GetTypeName() const { return "EQUATION"; }

    int AddParent(int theParent);
    int RemoveParent(int theParent);

    DSL_generalEquation& GetEquation() { return equation; }
    const DSL_generalEquation& GetEquation() const { return equation; }
	DSL_expression* GetSolution() { return solution ? solution : Solve(); }
	const DSL_expression* GetSolution() const { return solution ? solution : Solve(); }
    
    int SetEquation(const std::string &eq, int *errPos = NULL, std::string *errMsg = NULL);
    bool ValidateEquation(const std::string &eq, std::vector<std::string> &vars, std::string &errMsg, int *errPos = NULL) const;

    void SetBounds(double low, double high);
    void GetBounds(double &low, double &high) const { low = lowBound; high = highBound; }

	bool IsDiscretized() const { return !discIntervals.empty(); }
	void EnsureIntervalsExist();
	const DSL_Dmatrix* GetDiscProbs() const { return discProbs; }
	void InvalidateDiscProbs();
	int UpdateDiscProbs(int randSeed = 0, std::vector<std::vector<double> > *samples = NULL);
	int SetDiscProbs(const DSL_doubleArray& probs);
	int SetDiscIntervals(const IntervalVector &intervals) { return SetDiscIntervals(lowBound, highBound, intervals); }
	int SetDiscIntervals(double lo, double hi, const IntervalVector &intervals);
	int ClearDiscIntervals();
	const IntervalVector& GetDiscIntervals() const { return discIntervals; }
	int GetDiscInterval(int intervalIndex, double &lo, double &hi) const;
	int Discretize(double x) const;

	int Clone(DSL_nodeDefinition &that);

    void GetStateNames(DSL_stringArray &states) const;

	int DaddyGetsBigger(int daddy, int thisPosition) { return OnDiscreteParentChange(); }
	int DaddyGetsSmaller(int daddy, int thisPosition) { return OnDiscreteParentChange(); }
	int DaddyChangedOrderOfOutcomes(int daddy, DSL_intArray &newOrder) { return OnDiscreteParentChange(); }
	int OrderOfParentsGetsChanged(DSL_intArray &newOrder) { return OnDiscreteParentChange(); }

    int ValidateExpressions(
        const DSL_extFunctionContainer &extFxn,
        int &errFxnIdx,
        std::string &errMsg) const;

    int PatchExpressions(const DSL_extFunctionContainer &extFxn);

    int PrepareForDiscreteChild();

private:
	void Construct();
	void ParentIdChanged(int parentHandle, const char *oldId, const char *newId);
    void InvalidateDescendants();
	void InvalidateWithDescendants();
	DSL_expression* Solve() const;
	int OnDiscreteParentChange();

    DSL_generalEquation equation;
    double lowBound;
    double highBound;
	mutable DSL_expression* solution;
	
	IntervalVector discIntervals;
	DSL_Dmatrix *discProbs;
};

#endif
