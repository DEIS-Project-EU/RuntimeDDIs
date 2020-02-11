#ifndef EXTRADEFINITION_H
#define EXTRADEFINITION_H

// {{SMILE_PUBLIC_HEADER}}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//   EXTRA DEFINITION 
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
#include "dslobject.h"
#include "intarray.h"
#include "generalclases.h"

#include <vector>
#include <string>

class DSL_network;
class DSL_sfoDefinition;

class DSL_extraDefinition : public DSL_object
{
 // holds some extra information for the node. Used for
 // customizations that are not easily done anywhere else,
 // like some of the ones done for DIAG.
 private:
  DSL_network  *network;
  int           handle;

  DSL_intArray          faultStates;     // contains a 1 if the state is a fault state

  int           defaultOutcome; // holds the default outcome for those
                                // node types that have a default...

  std::vector<DSL_documentation> stateDocumentation; // holds the documentation for each state

  DSL_stringArray stateDescriptions; // description for each of the states
  DSL_idArray     faultLabels;       // labels indexed by the STATES, not by the faulty states
  DSL_stringArray faultNames;        // names for the faulty states, again, indexed by all states.

  DSL_stringArray stateRepairInfo; // repair (for faults) and state clarity test (for observables)
  std::string question;

  ///////////////////////////////////////////////
  ///////////////////////////////////////////////
  // new node troubleshooting types
 public:
  enum troubleType {target = 0, observation = 1, auxiliary = 2};
  enum showAsType {userDefined = 0, nodeName = 1, stateName = 2, nodeAndStateName = 3, inherit = 4};

 private:
  troubleType type;
  bool ranked;
  bool mandatory;   
  bool setToDefault;

  showAsType showAs;

 public:
  troubleType GetType(void) const {return(type);};
  bool IsRanked(void) const {return(ranked);};
  bool IsMandatory(void) const {return(mandatory);};
  bool IsSetToDefault(void) const {return(setToDefault);};
  void SetType(troubleType newType) {type = newType;};
  void SetFlags(bool setRanked = false, bool setMandatory = false, bool setSetToDefault = false)
   {
    ranked = setRanked;
    mandatory = setMandatory;
    setToDefault = setSetToDefault;
   };

  void SetShowAs(showAsType asThis) {showAs = asThis;};
  showAsType GetShowAs(void) const {return(showAs);};

  ///////////////////////////////////////////////
  ///////////////////////////////////////////////

 public:
  // creation/destruction
  DSL_extraDefinition(int myHandle, DSL_network *theNetwork);
  DSL_extraDefinition(DSL_extraDefinition &likeThisOne);
  virtual ~DSL_extraDefinition() {CleanUp();};

  int operator =(DSL_extraDefinition &likeThisOne);

  // fast operations
  DSL_extraDefinition(DSL_extraDefinition &likeThisOne, bool fast);
  int FastCopy(DSL_extraDefinition &likeThisOne);


  // member access
  DSL_network  *Network(void) {return(network);};
  int           SetNetwork(DSL_network *thisOne);
  int           Handle(void)  {return(handle);};

  void SetNetworkAndHandle(DSL_network *network, int handle);

  int GetDefaultOutcome(void) {return(defaultOutcome);};
  int SetDefaultOutcome(int thisOne);

  void SetFaultState(int thisState, int isFault = DSL_TRUE);
  int  IsFaultState(int thisState);

  int SetFaultStates(DSL_intArray &fromHere);

  // methods to deal with changes in my definition
  int NodeGetsBigger(int thisPosition);
  int NodeGetsSmaller(int thisPosition);
  int NodeChangedOrderOfStates(DSL_intArray &newOrder);
  
  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);

  int Clone(DSL_extraDefinition &thisGuy);
  int ReCreateFromNetworkStructure(void);

  DSL_intArray &GetFaultStates(void) {return(faultStates);};

  DSL_documentation &GetDocumentation(int forThisState);

  char *GetStateDescription(int forThisState);

  char *GetStateRepairInfo(int forThisState);
  int   SetStateDescription(int forThisState, const char *newDescription);
  int   SetStateRepairInfo(int forThisState, const char *newInfo);

  DSL_stringArray &GetStateDescriptions(void);
  DSL_stringArray &GetFaultNames(void);
  const DSL_idArray     &GetFaultLabels(void);
  DSL_stringArray &GetStateRepairInfos(void);
  std::string &GetQuestion(void) {return(question);};

  // NOTE: labels must be unique for a given network
  // These functions will return an error if an attempt is
  // made to enter a duplicate label
  int SetLabel(int thisOne, const char *theLabel);
  int SetLabels(DSL_idArray &theLabels);

  // moved to public interface - we need to check uniqueness
  // in node description propery page
  int GenerateUniqueLabel(char *prefix, char *buffer);
  int IsLabelInUse(char *theLabel, int ignoredNode = DSL_OUT_OF_RANGE);

  // tomek feb 27, 2001
  // call GetEffectiveFaultName with forceCompose == true
  // to get effective name even if node is not target or state is not fault
  // this functionality is used by node description page
  std::string GetEffectiveFaultName(int state, bool forceCompose = false);

  // this override accepts alternate state name passed in szStateName
  // parameter. set szStateName to NULL if you want to use real state name.
  // this functionality is used by node description page
  std::string GetEffectiveFaultName(int state, showAsType format, const char *szStateName, bool forceCompose);

		// TCL: definition for serach for opportunities
	 private:
			DSL_sfoDefinition* sfoDef;
		public:
			DSL_sfoDefinition& SfoDef(void);  // instantiate [sfoDef] if it has not been instantiated
			bool HasSfoDef(void) const {return (NULL != sfoDef);}; 

 private:
  int CreateAndInsertValidLabel(char *prefix, int here);
};

#endif
