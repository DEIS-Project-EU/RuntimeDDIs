#ifndef DSL_DEFNOISYADDER_H
#define DSL_DEFNOISYADDER_H

// {{SMILE_PUBLIC_HEADER}}

#include "cidefinition.h"
#include "network.h"
#include "node.h"


class DSL_noisyAdder : public DSL_ciDefinition  
{
public:
    enum Function { fun_average, fun_single_fault };

    DSL_noisyAdder(int myHandle, DSL_network *theNetwork);
    DSL_noisyAdder(DSL_nodeDefinition &likeThisOne);
  
    int GetType() const { return DSL_NOISY_ADDER; } 
    const char* GetTypeName() const { return "NOISY_ADDER"; }

    int ReCreateFromNetworkStructure();
    
    int operator=(DSL_nodeDefinition &likeThisOne);
    int Clone(DSL_nodeDefinition &thisGuy);
    void CheckConsistency(int deep = 0);

    int GetNumberOfParents() const { return network->GetParents(handle).NumItems(); }
    int GetNumberOfParentOutcomes(int parentPos) const;
    int GetParentStartingPosition(int parentPos) const;
  
    int AddParent(int theParent);
    int RemoveParent(int theParent);
    int DaddyGetsBigger(int daddy, int thisPosition);
    int DaddyGetsSmaller(int daddy, int thisPosition);
    int DaddyChangedOrderOfOutcomes(int daddy, DSL_intArray &newOrder);
    int OrderOfParentsGetsChanged(DSL_intArray &newOrder);
    int ChangeOrderOfOutcomes(DSL_intArray &newOrder);
    
    int SetDefinition(DSL_Dmatrix &withThis);
    int SetDefinition(DSL_doubleArray &withThis);
    int CheckCiWeightsConsistency(DSL_Dmatrix &ciWeights, char * errorMsg, int errorMsgBufSize);

    //  ==== Converting noisy-adder into CPT distribution ====
    int CiToCpt();
    int CiToCpt(DSL_Dmatrix& ci,DSL_Dmatrix& cpt);


    int AddOutcome(const char *thisName) {return InsertOutcome(GetNumberOfOutcomes(),thisName);} 
    int InsertOutcome(int here, const char *thisName);
    int RemoveOutcome(int outcomeNumber);
    int SetNumberOfOutcomes(int aNumber);
    int SetNumberOfOutcomes(DSL_stringArray &theOutcomeNames);

    int CptToCi() { return DSL_OUT_OF_RANGE; } // It's a subject for the next paper...
    DSL_Dmatrix &GetCpt();
    int CiIndexConstrained(DSL_Dmatrix &ci,int index);

    /*
    // Methods for relavance
    int AbsorbEvidenceFromParent(int theParent);
    int MarginalizeParent(int theParent);
    */   

    int GetDistinguishedState() {return dState; }
    int GetParentDistinguishedState(int parentPos) const { return dParentStates[parentPos]; }
    double GetParentWeight(int parentPos) { return parentWeights[parentPos]; }

    DSL_doubleArray& ParentWeights() { return parentWeights; }
    DSL_intArray& ParentDistinguishedStates() { return dParentStates;}

    int SetDistinguishedState(int thisState);
    int SetParentDistinguishedState(int parentPos, int newDState);
    int SetParentWeight(int parentPos, double value);

    int SetFunction(Function val);
    Function GetFunction() const { return function; }

    int GetTemporalFunction(int order, Function &val) const;
    int SetTemporalFunction(int order, Function val);

    int GetTemporalDistinguishedState(int order) const;
    int SetTemporalDistinguishedState(int order, int state);

    int GetTemporalParentInfo(int order, DSL_doubleArray &weights, DSL_intArray &distStates) const;
    int SetTemporalParentInfo(int order, const DSL_doubleArray &weights, const DSL_intArray &distStates);

private:
    void DoCopyParameters(DSL_nodeDefinition &target) const;
    int CiToCptAverage(DSL_Dmatrix& ci,DSL_Dmatrix& cpt);
    int CiToCptSingleFault(DSL_Dmatrix& ci,DSL_Dmatrix& cpt);

    int TemporalHelper(int order, DSL_noisyAdder*& def) const;

    int dState;
    DSL_intArray dParentStates;
    DSL_doubleArray parentWeights;
    Function function;


};

#endif
