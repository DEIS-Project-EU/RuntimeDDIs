#ifndef SYSCOORD_H
#define SYSCOORD_H

// {{SMILE_PUBLIC_HEADER}}

#include "dslobject.h"
#include "intarray.h"

class DSL_Dmatrix;
class DSL_nodeDefinition;
class DSL_nodeValue;

class DSL_sysCoordinates : public DSL_object
{
 protected:
  DSL_Dmatrix *link;
  DSL_intArray currentPosition;
  int          currentIntPosition;

  DSL_intArray lockedCoords;

 public:
  DSL_sysCoordinates(void);
  DSL_sysCoordinates(DSL_Dmatrix         &theLink);
  DSL_sysCoordinates(DSL_nodeDefinition  &theLink);
  DSL_sysCoordinates(DSL_nodeValue       &theLink);
  void LinkTo(DSL_Dmatrix         &theLink);
  void LinkTo(DSL_nodeDefinition  &theLink);
  void LinkTo(DSL_nodeValue       &theLink);

  int &operator[](int index) {return(currentPosition[index]);};
  // navigation
  void GoFirst(void);
  void GoLast(void);
  int GoTo(int theIndex);
  int Next(void);
  int Previous(void);
  int GoToCurrentPosition(void);
  // conversion
  int  Index(void) {return(currentIntPosition);};
  DSL_intArray &Coordinates(void) {return(currentPosition);};
  // accesing data
  double &UncheckedValue(void);
  double &CheckedValue(void);

  int LockCoordinate(int thisOne);
  int UnlockCoordinate(int thisOne);
  int UnlockAll(void);

 protected:
  void Initialize(void);
};


#endif 
