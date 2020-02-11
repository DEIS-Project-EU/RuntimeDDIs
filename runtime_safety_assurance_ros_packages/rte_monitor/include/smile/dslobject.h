#ifndef DSLOBJECT_H
#define DSLOBJECT_H

// {{SMILE_PUBLIC_HEADER}}

#include "errors.h"
#include "general.h"

// These are some flags that most dsl classes should
// have. Some of them will not be useful or even meaningful
// for some of the classes but they are general enough to
// have them grouped here.
// Make every class you define inherit from [DSL_object]

// some general flags
#define DSL_OBJECT_READY       1
#define DSL_OBJECT_CONSISTENT  2

#define DSL_NOCRASH

#ifdef DSL_NOCRASH
#define DSL_DEFAULT_OBJECT_FLAGS (DSL_OBJECT_CONSISTENT)
#else
#define DSL_DEFAULT_OBJECT_FLAGS (DSL_OBJECT_READY | DSL_OBJECT_CONSISTENT)
#endif

// Explanation of the flags

// DSL_OBJECT_READY : this flag is set when the object is ready
// for operation. This means that the object is allocated and 
// initialized correctly. Every object should have this flag set
// if and only if it is ready for working with it. The fact that
// this flag is set for an object doesn't imply that every object
// contained in it has the flag also set. DEFAULT: FALSE

// DSL_OBJECT_CONSISTENT : this flag is very dependable on the
// nature of the object. The object can be perfectly allocated
// but still be inconsistent. Ex: a TruthTable that contains
// other than 0's or 1's on his conditional probability table.
// This flag imply that every object contained is also consistent. <----- check later !!!
// DEFAULT: TRUE


// Explanation of the functions

// CheckReadiness : this function should check that all the members
// of the object are there (allocated) and, if so, set the flag 
// DSL_OBJECT_READY. This function SHOULD NOT check the readiness of
// the members of the class (no deepening) unless specified otherwise.

// CheckConsistency : this function should check that all the members
// of the object are consistent and, if so, set the flag 
// DSL_OBJECT_CONSISTENT. This function SHOULD check the consistency  <----- check later !!!
// of the members of the class (deepening) unless specified otherwise.

// CleanUp : this function should "clean" the object an leave it
// empty. After this call, the object is probably consistent and
// ready. Think of this function as the destructor of the class (in
// fact, almost every destructor will just invoke CleanUp)            <----- check later !!!

class DSL_object
{
public:
  // Housekeeping
  virtual void CleanUp(int deep = 0);
  // Is everithing Ok?
  virtual void CheckReadiness(int deep = 0)       {SetObjectReady();}
  virtual void CheckConsistency(int deep = 1)     {SetObjectConsistent();}

  int ObjectReady() const{ return status & DSL_OBJECT_READY; }
  int ObjectConsistent() { return status & DSL_OBJECT_CONSISTENT; }

 protected:
  DSL_object() {status = DSL_DEFAULT_OBJECT_FLAGS;}
  virtual ~DSL_object() { CleanUp(); }

  // Ready
  void SetObjectReady()         {status |=  DSL_OBJECT_READY;}
  void SetObjectNotReady()      {status &= ~DSL_OBJECT_READY;}
  // Consistent
  void SetObjectConsistent()    {status |=  DSL_OBJECT_CONSISTENT;}
  void SetObjectNotConsistent() {status &= ~DSL_OBJECT_CONSISTENT;}

  int status;
};

#endif
