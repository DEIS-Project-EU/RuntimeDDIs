#ifndef IDARRAY_H
#define IDARRAY_H

// {{SMILE_PUBLIC_HEADER}}

#include "stringarray.h"

class DSL_idArray : public DSL_stringArray
{
 public:
  DSL_idArray(bool emptyIdsEnabled = false);
  DSL_idArray(int initialSize, int initialDelta=10, bool emptyIdsEnabled = false);
  DSL_idArray(DSL_idArray &likeThisOne);
  int operator=(DSL_stringArray &likeThisOne);

  bool EmptyIdsEnabled() const { return emptyIdsEnabled; }


  int  SetString(int thisPosition, const char *thisString);
  int  Add(const char *thisString);
  int  Insert(int here, const char *thisString);
  int  CreateAndAddValidId(const char *prefix = NULL, int serialNumber = 0);
  int  CreateAndInsertValidId(const char *prefix = NULL, int serialNumber = 0, int here = 0);
  int  CreateValidId(const char *prefix, int serialNumber, char *buffer);
  int  MakeConsistent(void);
  void CheckConsistency(int deep = 1);
  void AddBlank();

 protected:
  int  ValidId(const char *thisString);
  int  ValidId(const char *thisString, int forThisPosition);
  int  MakeValidId(char *fromThisString);
  int  MakeValidIdExtra(char *fromThisString);

  bool emptyIdsEnabled;
};


#endif
