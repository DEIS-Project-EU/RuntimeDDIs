#ifndef NODEDEF_H
#define NODEDEF_H

// {{SMILE_PUBLIC_HEADER}}

#include "dslobject.h"

// these four flags represent the four basic types of nodes
#define DSL_DECISION           1    // set if decision node
#define DSL_CHANCE             2    // set if chance node
#define DSL_DETERMINISTIC      4    // set if deterministic node
#define DSL_UTILITY            8    // set if utility node
// these other flags represent some properties of the nodes
#define DSL_DISCRETE          16    // set if discrete, cleared if continous
#define DSL_CASTLOGIC         32    // set if node is CAST logic 
#define DSL_DEMORGANLOGIC     64    // set if node is DeMorgan
#define DSL_NOISYMAXLOGIC    128    // set if Noisy MAX representation
#define DSL_NOISYADDERLOGIC  256    // set if NoisyAdder representation
// temporary flag
#define DSL_PARENTSCONTIN    512    // set if any parent is continuous
// and more stuff
#define DSL_SCC             1024    // set if strongly-coupled component of equation


// types of definitions
//                          node type           features
#define DSL_TRUTHTABLE     (DSL_DETERMINISTIC | DSL_DISCRETE)
#define DSL_CPT            (DSL_CHANCE        | DSL_DISCRETE)
#define DSL_NOISY_MAX      (DSL_CHANCE        | DSL_DISCRETE |  DSL_NOISYMAXLOGIC)
#define DSL_NOISY_ADDER    (DSL_CHANCE        | DSL_DISCRETE |  DSL_NOISYADDERLOGIC)
#define DSL_CAST           (DSL_CHANCE        | DSL_DISCRETE |  DSL_CASTLOGIC)
#define DSL_DEMORGAN       (DSL_CHANCE        | DSL_DISCRETE |  DSL_DEMORGANLOGIC)
#define DSL_LIST           (DSL_DECISION      | DSL_DISCRETE)
#define DSL_TABLE          (DSL_UTILITY)
#define DSL_MAU            (DSL_UTILITY       | DSL_PARENTSCONTIN)
#define DSL_EQUATION       (DSL_DETERMINISTIC)
#define DSL_EQUATION_SCC   (DSL_DETERMINISTIC | DSL_SCC)
#define DSL_NO_DEFINITION  0  // used when definition not created

class DSL_network;
class DSL_intArray;
class DSL_stringArray;
class DSL_idArray;
class DSL_doubleArray;
class DSL_Dmatrix;
class DSL_extFunctionContainer;

class DSL_nodeDefinition : public DSL_object
{
 public:
  DSL_nodeDefinition(int myHandle, DSL_network *theNetwork);
  DSL_nodeDefinition(DSL_nodeDefinition &likeThisOne);
  DSL_nodeDefinition() {network = NULL; handle = -1; CheckReadiness();}
  virtual ~DSL_nodeDefinition() {CleanUp();}

  DSL_network  *Network() {return(network);}
  int           SetNetwork(DSL_network *thisOne);
  void SetNetworkAndHandle(DSL_network *network, int handle);
  int Handle() const { return handle; }

  void CopyParameters(DSL_nodeDefinition &target);

  // type
  virtual int GetType() const = 0;
  virtual const char *GetTypeName() const = 0;
  int NodeTypeIs(int flags) const { return flags & GetType(); }
  
  // methods to deal with changes in my relationships
  virtual int AddParent(int theParent) { return DSL_WRONG_NODE_TYPE; }
  virtual int RemoveParent(int theParent) { return DSL_WRONG_NODE_TYPE; }
  virtual int AbsorbEvidenceFromParent(int theParent) { return DSL_WRONG_NODE_TYPE; }
  virtual int MarginalizeParent(int theParent) { return DSL_WRONG_NODE_TYPE; }

  // methods to deal with changes in the definition of my parents
  virtual int DaddyGetsBigger(int daddy, int thisPosition) { return DSL_WRONG_NODE_TYPE; }
  virtual int DaddyGetsSmaller(int daddy, int thisPosition) { return DSL_WRONG_NODE_TYPE; }
  virtual int DaddyChangedOrderOfOutcomes(int daddy, DSL_intArray &newOrder) { return DSL_WRONG_NODE_TYPE; }
  virtual int OrderOfParentsGetsChanged(DSL_intArray &newOrder) { return DSL_WRONG_NODE_TYPE; }
  virtual void ParentIdChanged(int parentHandle, const char *oldId, const char *newId) {}

  // methods to deal with direct changes in my definition
  virtual int BeforeDirectChange(int changeType = 0);
  virtual int AfterDirectChange(int changeType = 0);

  virtual int GetSize() { return DSL_WRONG_NODE_TYPE; }
  virtual int operator =(DSL_nodeDefinition &likeThisOne) { return DSL_WRONG_NODE_TYPE; }
  virtual void CheckReadiness(int deep = 0);

  virtual int ReCreateFromNetworkStructure() { return DSL_WRONG_NODE_TYPE; }
  virtual int Clone(DSL_nodeDefinition &thisGuy);

  virtual int CheckParentsStructure() { return DSL_WRONG_NODE_TYPE; }
  
  virtual int IsParentNecessary(int parentIndex, double epsilon, bool &necessary);

  virtual int ValidateExpressions(const DSL_extFunctionContainer &extFxn, int &errFxnIdx,std::string &errMsg) const  { return DSL_OKAY; }
  virtual int PatchExpressions(const DSL_extFunctionContainer &extFxn) { return DSL_OKAY; }

 // common interface
 public:
  virtual int AddOutcome(const char *thisName) { return DSL_WRONG_NODE_TYPE; }
  virtual int InsertOutcome(int here, const char *thisName) { return DSL_WRONG_NODE_TYPE; }
  virtual int RemoveOutcome(int outcomeNumber) { return DSL_WRONG_NODE_TYPE; }
  virtual int GetNumberOfOutcomes() const { return DSL_WRONG_NODE_TYPE; }
  virtual int RenameOutcome(int outcomeNumber, const char *newName) { return DSL_WRONG_NODE_TYPE; }
  virtual int RenameOutcomes(DSL_stringArray &theOutcomeNames) { return DSL_WRONG_NODE_TYPE; }
  virtual DSL_idArray *GetOutcomesNames() { return NULL; }
  virtual int SetNumberOfOutcomes(int aNumber) { return DSL_WRONG_NODE_TYPE; }
  virtual int SetNumberOfOutcomes(DSL_stringArray &theOutcomeNames) { return DSL_WRONG_NODE_TYPE; }
  virtual int ChangeOrderOfOutcomes(DSL_intArray &newOrder) { return DSL_WRONG_NODE_TYPE; }
  virtual int ChangeOrderOfOutcomesWithAddAndRemove(DSL_idArray &ids, DSL_intArray &newOrder);
  virtual int MakeUniform() { return DSL_WRONG_NODE_TYPE; }

  // get/set definitions in several ways
  virtual int GetDefinition(DSL_Dmatrix **here) { return DSL_WRONG_NODE_TYPE; }
  virtual int GetDefinition(DSL_doubleArray **here) { return DSL_WRONG_NODE_TYPE; }
  virtual int GetDefinition(DSL_intArray **here) { return DSL_WRONG_NODE_TYPE; }
  virtual int GetDefinition(DSL_stringArray **here) { return DSL_WRONG_NODE_TYPE; }

  // be careful with these guys... (USUALLY, NO CHECKING)
  virtual int SetDefinition(DSL_doubleArray &withThis) { return DSL_WRONG_NODE_TYPE; }
  virtual int SetDefinition(DSL_intArray &withThis) { return DSL_WRONG_NODE_TYPE; }
  virtual int SetDefinition(DSL_stringArray &withThis) { return DSL_WRONG_NODE_TYPE; }
  virtual int SetDefinition(DSL_Dmatrix &withThis) { return DSL_WRONG_NODE_TYPE; }

  const DSL_doubleArray* GetTemporalDefinition(int order);
  int SetTemporalDefinition(int order, DSL_doubleArray& temporal);

  DSL_Dmatrix* GetMatrix()
  {
	  DSL_Dmatrix *m = NULL;
	  GetDefinition(&m);
	  return m;
  }

 protected:   
  DSL_network  *network;
  int           handle;
  int CommonInsertOutcome(int outcomeIndex);
  int CommonRemoveOutcome(int outcomeIndex, DSL_idArray &outcomeNames);
  int CommonChangeOrderOfOutcomes(DSL_intArray &newOrder);
  int RemoveArcsFromParents();

  virtual void DoCopyParameters(DSL_nodeDefinition &target) const {}
};

#endif
