#ifndef DSL_CASELIBRARY_H
#define DSL_CASELIBRARY_H

// {{SMILE_PUBLIC_HEADER}}

#ifdef _WIN32
#pragma warning (disable: 4786)
#endif

class DIAG_network;

#include <vector>
#include <string>

///////////////////////////////
// class DSL_case
///////////////////////////////

struct DSL_case
{
 typedef std::pair<std::string,std::string> _evidenceItem;  // (nodeId,outcomeId)
 typedef std::vector<_evidenceItem>         _evidenceContainer;

 typedef std::pair<std::string,std::string> _pursuedFaultItem;  // (nodeId,outcomeId)
 typedef std::vector<_pursuedFaultItem>     _pursuedFaultsContainer;

 DSL_case();
 //DSL_case(const DSL_case &likeThisOne);

 std::string title;
 std::string comment;
 _pursuedFaultsContainer pursuedFaults;
 std::string modelName;
 _evidenceContainer evidence;
 double entropyCostRatio;
 double maxEntropyCostRatio;
};

//-----------------------------------

class DSL_caseLibrary
{
 public:
  typedef std::vector<DSL_case> _caseContainer;

  DSL_caseLibrary(DIAG_network *linkToNetwork);
  DSL_caseLibrary(void);

  void LinkTo(DIAG_network *thisNetwork);

  int ReadCaseLibrary(char *thisFile);
  int WriteCaseLibrary(char *thisFile);

  const _caseContainer &GetCases(void) const;
  _caseContainer &GetCases(void);
  int AddCase(const DSL_case &newCase);
  const DSL_case &GetCase(int thisOne);
  int DeleteCase(int thisOne);

  int InstantiateCaseEvidence(int thisOne);
  int FillCaseWithEvidence(DSL_case &here);
  int FillCaseWithPursuedFaults(DSL_case &here);

  bool EvidenceValid(const DSL_case::_evidenceItem &thisEvidence, std::pair<int,int> &handles);
  bool FaultValid(const std::string &faultNode, const std::string &faultState, std::pair<int,int> &handles);

  //int SaveCurrentCase(const char *withThisTitle, bool overwrite = false);
  //int InstantiateCase(char *withThisTitle);

  int FindCase(const char *withThisTitle);

// new interfarce...
  void SaveCurrentCase(DSL_case &here);
  void InstantiateCase(const DSL_case &thisOne);
  int ConsistentCase(const DSL_case &thisOne, bool ignoreModelName);



 private:
  _caseContainer theCases;
  DIAG_network *theNetwork;
};

//-----------------------------------


#endif


