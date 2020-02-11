#ifndef DSL_DEFNOISYMAX_H
#define DSL_DEFNOISYMAX_H

// {{SMILE_PUBLIC_HEADER}}

#include "cidefinition.h"
#include <vector>

// this is an implementation of noisy-MAX.
// as native parametrization we use weight[i] = P(Y|Xi), 
// called 'Diez' but this convention is not strict. 
// We DON'T use P(Y|~X1,...,Xi,...,Xn)

class DSL_noisyMAX : public DSL_ciDefinition  
{
public:
    DSL_noisyMAX(int myHandle, DSL_network *theNetwork);
    DSL_noisyMAX(DSL_nodeDefinition &likeThisOne);

    int GetNumberOfParentOutcomes(int parentPos) const { return parentOutcomeStrengths[parentPos].NumItems(); }
    int GetParentStartingPosition(int parentPos) const { return parentStartingPos[parentPos]; }

    const DSL_intArray &GetParentOutcomeStrengths(int parentPos) const { return parentOutcomeStrengths[parentPos]; }
    int GetStrengthOfOutcome(int parentPos, int parentPosOutcome) const { return parentOutcomeStrengths[parentPos].FindPosition(parentPosOutcome); }
    int GetOutcomeOfStrength(int parentPos, int outcomeStrength) const { return parentOutcomeStrengths[parentPos][outcomeStrength]; }
    int GetNumberOfParents() const { return (int)parentOutcomeStrengths.size(); }
    int SetParentOutcomeStrengths(int parentPos, const DSL_intArray &newStrengths); 

    int GetTemporalParentOutcomeStrengths(int order, std::vector<DSL_intArray>& strengths) const;
    int SetTemporalParentOutcomeStrengths(int order, const std::vector<DSL_intArray>& strengths);

    int GetType() const { return DSL_NOISY_MAX; } 
    const char* GetTypeName() const { return "NOISY_MAX"; }
    int operator=(DSL_nodeDefinition &likeThisOne);
    int Clone(DSL_nodeDefinition &thisGuy);
    void CleanUp(int deep = 0);  
    int ReCreateFromNetworkStructure();
    
    void CheckConsistency(int deep = 0);

    int AddParent(int theParent);
    int RemoveParent(int theParent);
    int DaddyGetsBigger(int daddy, int thisPosition);
    int DaddyGetsSmaller(int daddy, int thisPosition);
    int DaddyChangedOrderOfOutcomes(int daddy, DSL_intArray &newOrder);
    int OrderOfParentsGetsChanged(DSL_intArray &newOrder);
    int ChangeOrderOfOutcomes(DSL_intArray &newOrder);
    int ChangeOrderOfStrengths(int parentPos, const DSL_intArray &newOrder);

    int IsParentNecessary(int parentIndex, double epsilon, bool &necessary);

    int SetDefinition(DSL_Dmatrix &withThis);
    int SetDefinition(DSL_doubleArray &withThis);
    int CheckCiWeightsConsistency(DSL_Dmatrix &ciWeights, char * errorMsg, int errorMsgBufSize);

    //  ==== Converting noisy-MAX into Henrion parametrization ====
    int GetHenrionProbabilities(DSL_Dmatrix &henrion);
    int SetHenrionProbabilities(DSL_Dmatrix &henrion);
    int CheckHenrionConsistency(DSL_Dmatrix &henrion, char * errorMsg, int errorMsgBufSize, int &errRow, int &errCol, bool &leakInconsistent);

    //  ==== Converting noisy-MAX into CPT distribution ====
    int CiToCpt();
    int CiToCpt(DSL_Dmatrix& ci,DSL_Dmatrix& cpt);

    //  ==== Converting CPT to noisy-MAX distribution ====
    static void CumulativeCiToCpt(const DSL_Dmatrix& ci, const std::vector<DSL_intArray> &parentOutcomeStrengths, const DSL_intArray &parentStartingPos, DSL_Dmatrix& cpt);
    static void CiToCumulativeCi(DSL_Dmatrix &ci);
    int CptToCi() { return SquareCptToCi(table,ciWeights,1/(double)GetNumberOfOutcomes(),0.0001); }
    int CiIndexConstrained(DSL_Dmatrix &ci,int index);
  
    // Square distance routines
    double SquareDistance(DSL_Dmatrix& cptA, DSL_Dmatrix& cptB) const;
    int SquareCptToCi(DSL_Dmatrix& cpt,DSL_Dmatrix& ci,double step, double minStep);

    // Kullback-Leibler distance routines
    double KLDistance(DSL_Dmatrix& cptReal, DSL_Dmatrix& cptApprox) const;
    int KLCptToCi(DSL_Dmatrix& cpt,DSL_Dmatrix& ci,double step, double minStep);

    // Part Added during transition noisyOR/AND -> noisyMAX
    int SetLegacyNoisyOrProbabilities(DSL_doubleArray &legacyWeights);
    int GetLegacyNoisyOrProbabilities(DSL_doubleArray &legacyWeights);
    int CalculateCptColumn(const DSL_intArray &coordinates, DSL_doubleArray &here); 

    DSL_Dmatrix &GetCpt();

    // Methods for relavance
    int AbsorbEvidenceFromParent(int theParent);
    int MarginalizeParent(int theParent);

    void SetParentOutcomeStrengthsUnchecked(int parentPos, const DSL_intArray &newStrengths);

private:
    bool IsNonZero(const DSL_Dmatrix& cpt) const;
    double SquareCiToCptSingleStep(DSL_Dmatrix& ci,DSL_Dmatrix& cpt,int index,double step);
    double KLCiToCptSingleStep(DSL_Dmatrix& ci,DSL_Dmatrix& cpt,int index,double step, bool undo = true);
    void DoCopyParameters(DSL_nodeDefinition &target) const;
    void RecalcParentStartingPositions();

    std::vector<DSL_intArray> parentOutcomeStrengths;
    DSL_intArray parentStartingPos;
};

#endif 
