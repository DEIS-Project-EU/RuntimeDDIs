#ifndef NODE_H
#define NODE_H

// {{SMILE_PUBLIC_HEADER}}

#include "generalclases.h"

class DSL_nodeDefinition;
class DSL_nodeValue;
class DSL_nodeCost;


//////////////////////////////////////////////////////////////////////////////
// class DSL_nodeInfo
//////////////////////////////////////////////////////////////////////////////

class DSL_nodeInfo : public DSL_object // this "class" just holds all the "not so important" info
{
 protected: 
  DSL_header         header;         // id, name, comment, etc
  DSL_screenInfo     screen;         // size, position, color, etc
  DSL_userProperties userProperties; // user-defined properties...
  DSL_documentation  documentation;  // documentation for the node 

 public:
  DSL_nodeInfo(void) {CheckReadiness();};
  DSL_nodeInfo(DSL_nodeInfo &likeThisOne);

  int operator =(DSL_nodeInfo &likeThisOne);

  DSL_header         &Header(void)         {return(header);};
  DSL_screenInfo     &Screen(void)         {return(screen);};
  DSL_userProperties &UserProperties(void) {return(userProperties);};
  DSL_documentation  &Documentation(void)  {return(documentation);};
};



//////////////////////////////////////////////////////////////////////////////
// class DSL_node
//////////////////////////////////////////////////////////////////////////////

class DSL_network;
class DSL_extraDefinition;

class DSL_node : public DSL_object
{
 protected:
  DSL_nodeInfo        *info;      // information "irrelevant" for reasoning
  DSL_nodeDefinition  *def;       // Definition of the node
  DSL_nodeValue       *val;       // Value of the node
  DSL_nodeCost        *observCost; // Cost of observing the state of the node
  DSL_extraDefinition *extraDef;  // holds some custom information
  DSL_network         *network;   // (points to) the network I am included in
  int                 handle;
  int                 submodel;   // the (graphical) submodel I belong to (MOVE SOMEWHERE ELSE!!!)

 public:
  DSL_node(int type, int theID, DSL_network *theNetwork);
  DSL_node(DSL_node &likeThisOne);
 ~DSL_node() {CleanUp();};

  int operator =(DSL_node &likeThisOne);
  
  // fast operations
  DSL_node(DSL_node &likeThisOne, bool fast);
  int FastCopy(DSL_node &likeThisOne);

  static bool IsNodeTypeValid(int nodeType);

  DSL_nodeInfo        &Info(void)            {return(*info);}; 
  DSL_network         *Network(void)         {return(network);}; 
  DSL_nodeDefinition  *Definition(void)      {return(def);};
  DSL_nodeValue       *Value(void)           {return(val);};
  DSL_nodeCost        *ObservCost(void)      {return(observCost);};
  DSL_extraDefinition *ExtraDefinition(void) {return(extraDef);};

  int                  Handle(void)     {return(handle);};

  const char* GetId() const;
  int SetId(const char *newId);

  int GetSubmodel(void) {return(submodel);};
  int SetSubmodel(int thisSubmodel);
  int SetSubmodel(char *thisSubmodel);

  int SetNetwork(DSL_network *thisOne);

  void SetDefinition(DSL_nodeDefinition &thisOne) {def = &thisOne;};
  void SetValue(DSL_nodeValue &thisOne) {val = &thisOne;};

  void InheritAspect();

  void CleanUp(int deep = 0);
  void CheckReadiness(int deep = 0);
  void CheckConsistency(int deep = 0);

  int CreateDefinition(int thisType);
  int CreateValue(int thisType);
  int ChangeType(int newType);

  int CreateDefinition(DSL_nodeDefinition &likeThisOne);
  int CreateValue(DSL_nodeValue &likeThisOne);

  int NodeTypeFromName(const char *typeName);
  int ChangeType(const char *typeName);

  int CheckNetworkStructure(int warnChildren);

 private:
  int CreateExtraDefinition(void);
};

#endif






