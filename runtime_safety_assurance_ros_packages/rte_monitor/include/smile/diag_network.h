#ifndef DSL_DIAG_NETWORK_H
#define DSL_DIAG_NETWORK_H

// {{SMILE_PUBLIC_HEADER}}

#include "extradefinition.h"
#include <vector>
#include <algorithm>

class DSL_network;
class DSL_fastEntropyAssessor;


#define DSL_DIAG_DEFAULT (DSL_DIAG_INDEPENDENCE | DSL_DIAG_PURSUE_ATLEAST_ONE_COMB)

///////////////////////////////
// class DIAG_network
///////////////////////////////

struct DIAG_faultyState
{
	int node;
	int state;

	//operators added by Denver 11-12-03 to allow sorting in STL:
	DIAG_faultyState& operator=(const DIAG_faultyState& rhs) 
	{
		if (this == &rhs) return (*this);
		node = rhs.node; state=rhs.state;
		return (*this);
	}

	bool operator<(const DIAG_faultyState& rhs) const
	{
		if (node == rhs.node)
			return (state<rhs.state);
		return (node<rhs.node);
	}

	bool operator==(const DIAG_faultyState& rhs) const
	{
		return (node==rhs.node && state==rhs.state);
	}
};

struct DIAG_testInfo
{
	int    test;
	double entropy;
	double cost;
	double strength;
	int observationPriorStartIndex;
	int faultPosteriorsStartIndex;
};

struct DIAG_faultInfo
{
  int nodeHandle;
  int nodeState;
  int faultHandle;
  double posterior;

  bool operator<(const DIAG_faultInfo& rhs) const
  {
	  return (posterior>rhs.posterior);
  }
};

class DIAG_network
{
public:
	DIAG_network();  
	DIAG_network(const DIAG_network &other);  

	~DIAG_network();

	int LinkToNetwork(DSL_network *thisNet);
	DSL_network& GetNetwork() { return *theNet; }

	void UpdateFaultBeliefs();

	// loads [fileName] and collects network information. It 
	// is assumed that the model will not change after this
	// method is called (i.e., we are in the troubleshooting
	// phase, as opposed to the model-building phase).
	int LoadModel(char *fileName);

	// clears everything and leaves the network as
	// if [LoadModel] has just been called...
	int RestartDiagnosis();

	// computes VOI for all the unperformed tests.
	int ComputeTestStrengths(int flags = DSL_DIAG_DEFAULT);

	bool IsDSepEnabled() const { return dsepEnabled; }
	void EnableDSep(bool enable) { dsepEnabled = enable; }

	bool AreQuickTestsEnabled() const { return quickTestsEnabled; }
	void EnableQuickTests(bool enable) { quickTestsEnabled = enable; }

	bool IsDetailedEntropyEnabled() const { return detailedEntropyEnabled; }
	void EnableDetailedEntropy(bool enable) { detailedEntropyEnabled = enable; }
	
	// returns the array of unperformed tests
	DSL_intArray& GetUnperformedTests() { return unperformedTests; }
	// returns the array of tests statistics. This array is paired with the
	// one returned by [GetUnperformedTests]
	std::vector<DIAG_testInfo>& GetTestStatistics() { return testStatistics; }

	// returns the array of faults
	const std::vector<DIAG_faultyState>& GetFaults() const { return faults; }
	
	const std::vector<double>& GetEntropyDetails() const { return entropyDetails; }
	void InitEntropyDetails();
	void AppendEntropyDetails(const double *p, int count);

	//
	void CalculateRankedFaults(std::vector<DIAG_faultInfo> &here, double lower, double upper);
	void CalculateRankedFaults(std::vector<DIAG_faultInfo> &here, bool ignoreEvidence = false);

	// sets the ratio in [theNet] as a user-defined property
	void SetEntropyCostRatio(double ratio, double maxRatio, DSL_network *onThisNet);
	double GetEntropyCostRatio();
	double GetMaxEntropyCostRatio();

	// unloads the network, leaving everything as it was 
	// before [LoadModel] was called
	int ClearModel();

	int InstantiateObservation(int thisNode, int toThisOutcome);
	int ReleaseObservation(int thisNode);

	// sets [faultHandle] as the pursued fault
	int SetPursuedFault(int faultHandle);
	int GetPursuedFault() const;  

	// nescessary functions for working with int_Array
	int SetPursuedFaults(const DSL_intArray & faultHandles);
	int AddPursuedFault(int faultHandle);    
	int DeletePursuedFault(int faultHandle);   
	const DSL_intArray& GetPursuedFaults() const { return pursuedFaults;}

	// traverses the network and set to their default state all those
	// nodes flagged as 'set to default'
	void SetDefaultStates();

	// returns the handle of the most likely fault (assummes 
	// the [theNet] is solved already). To solve the DSL_network 
	// you should call GetNetwork()->UpdateBeliefs();
	int FindMostLikelyFault();

	// helper function that should not be here but that it is so 
	// the bridge can be more easily implemented. I suggest this function 
	// is moved to some other level above here. Dani 29/8/2000
	// This function will return [here] a list of handles according 
	// to the parameters. These handles could be observation handles (node
	// handles in SMILE) or target handles (fault handles in DIAG_network)
	// NOTE: it does NOT clear [here] before adding the new elements
	int GetHandles(
		DSL_extraDefinition::troubleType ofThisType, 
		bool getAll, 
		bool ranked, 
		bool mandatory, 
		bool setToDefault, 
		std::vector<int> &here);


 public:
  // BUT NOT NEEDED FOR THE BRIDGE/API

	// traverses the network to find and store targets and unperformed tests
	int CollectNetworkInfo();

	// returns true if all mandatories are instantiated...
	bool MandatoriesInstantiated();

	// returns the handle of the fault [faultNode,faultState]
	int FindFault(int faultNode, int faultState);

	// set the network and recollect the network info
	// this function avoid compilation but allows us to use diag info
	void SetNetwork(DSL_network *thisNet);


private:
	void operator=(const DIAG_network&); // never implemented
	void CompileModel();
	double GetFromProperty(const char *propertyName, double defaultValue);
	void DoCalcRankedFaults(std::vector<DIAG_faultInfo> &here, double lower, double upper, bool openSet );

	DSL_network  *theNet;
	DSL_fastEntropyAssessor *theAssessor;

	// contains all ranked observations not observed yet
	DSL_intArray unperformedTests;
	// test results (paired with [unperformedTests])
	std::vector<DIAG_testInfo> testStatistics;

	std::vector<DIAG_faultyState> faults;
	DSL_intArray pursuedFaults; // indexes in [faults]

	std::vector<double> entropyDetails;

	bool dsepEnabled;
	bool quickTestsEnabled;
	bool detailedEntropyEnabled;
};


#endif
