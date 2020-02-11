#ifndef DSL_DMATRIX_H
#define DSL_DMATRIX_H

// {{SMILE_PUBLIC_HEADER}}

#include "intarray.h"
#include "doublearray.h"
#include "errors.h"

class DSL_Dmatrix
{
 public:
  DSL_Dmatrix() {}
  DSL_Dmatrix(const DSL_intArray &theseDimensions);

  DSL_Dmatrix(const DSL_Dmatrix  &likeThisOne) :
  	theDimensions(likeThisOne.theDimensions),
	thePreProduct(likeThisOne.thePreProduct),
	theData(likeThisOne.theData)
  {
  }

  int operator=(const DSL_Dmatrix &likeThisOne)
  {
	theDimensions = likeThisOne.theDimensions;
	thePreProduct = likeThisOne.thePreProduct;
	theData       = likeThisOne.theData;
	return DSL_OKAY;
  }

  double &operator[](int index) { return theData[index]; }
  double operator[](int index) const { return theData[index]; }
  double &operator[](const DSL_intArray &theCoordinates);
  double operator[](const DSL_intArray &theCoordinates) const;
  double &operator[] (const int * const * indC) { return theData[CoordinatesToIndex(indC)]; }
  double operator[] (const int * const * indC) const { return theData[CoordinatesToIndex(indC)]; }

  double &Subscript(int index);
  double Subscript(int index) const;
  double &Subscript(DSL_intArray &theCoordinates);
  double Subscript(DSL_intArray &theCoordinates) const;
  int GetNumberOfDimensions() const { return theDimensions.GetSize(); } 
  int GetLastDimension() const { return GetNumberOfDimensions() - 1 ; }
  int GetSizeOfDimension(int aDimension) const;
  int GetSize() const { return theData.GetSize(); }
  int Sum(const DSL_Dmatrix &aDmatrix, const DSL_Dmatrix &withDmatrix);
  int Subtract(const DSL_Dmatrix &aDmatrix, const DSL_Dmatrix &minusThisDmatrix);
  int Add(const DSL_Dmatrix &byTheseValues);
  int Add(double byThisMuch);
  int Multiply(double byThisNumber);
  int Multiply(DSL_doubleArray &byThisVector);
  int FillWith(double aNumber);
  int Setup(const DSL_intArray &theseDimensions);
  int Setup(const int * dimensions, int dimCount);

  int AddDimension(int thisSize);
  int AddDimensions(const DSL_intArray &newDimensions);
  int InsertDimension(int here, int thisSize);
  int RemoveDimension(int thisDimension);                                    // with nothing
  int RemoveDimension(int thisDimension, int preservingThis);                // with evidence
  int RemoveDimension(int thisDimension, const DSL_doubleArray &usingTheseValues); // with marginals
  int RemoveDimension(int thisDimension, int preservingThis, DSL_Dmatrix &intoThisMatrix);
  int RemoveDimension(int thisDimension, const DSL_doubleArray &usingTheseValues, DSL_Dmatrix &intoThisMatrix);
  int RemoveDimensions(DSL_intArray &theseDimensions, DSL_Dmatrix &intoThisMatrix);
  int RemoveDimension(int thisDimension, DSL_Dmatrix &usingTheseValues, DSL_intArray &dimensionMapping);

  int IncreaseDimensionSize(int thisDimension);
  int DecreaseDimensionSize(int thisDimension);
  int ResizeDimension(int thisDimension, int thisSize);
  int ShiftDataUpwards(int thisDimension, int fromThisValue, double defVal);
  int ShiftDataUpwards(int thisDim, int fromThisVal) { return ShiftDataUpwards(thisDim, fromThisVal, 0); }
  int ShiftDataDownwards(int thisDimension, int fromThisValue);

  int Normalize(); 
  int Normalize(DSL_intArray &theseCoordinates);
  int Normalize(int thisElement);
  int Normalized(DSL_intArray &theseCoordinates);
  int Normalized(int thisElement);
  int Complement(DSL_intArray &theseCoordinates);
  int Complement(int thisElement);

  // this CheckElements will substitute the old one (the one below it)
  int CheckElements(double aValue, DSL_intArray &theseCoordinates);
  int CheckElements(double aValue, int firstDim, int firstVal, int secondDim, int secondVal);

  void CleanUp();

  void Swap(DSL_Dmatrix &that) { theDimensions.Swap(that.theDimensions); thePreProduct.Swap(that.thePreProduct); theData.Swap(that.theData); }

  int IndexToCoordinates(int theIndex, DSL_intArray &theCoordinates) const;
  int CoordinatesToIndex(DSL_intArray &theCoordinates) const;
  int CoordinatesToIndex(const int * const * indirectCoordinates) const;
  int NextCoordinates(DSL_intArray &theCoordinates) const;
  int PrevCoordinates(DSL_intArray &theCoordinates) const;

  // access
  DSL_doubleArray& GetItems() { return theData; }
  const DSL_doubleArray& GetItems() const { return theData; }
  const DSL_intArray& GetDimensions() const { return theDimensions; }
  const DSL_intArray& GetPreProduct() const { return thePreProduct; }

  //Added by Adam 2/23/02
  int ChangeOrderOfDimensions(const DSL_intArray &newOrder);
  int ChangeOrderWithinDimension(int thisDim, const DSL_intArray &newOrder);

  int CompatibleWith(const DSL_Dmatrix &thisDmatrix) const;

 protected:
  int Ok(int index) const { return index >= 0 && index < theData.GetSize(); }
  int Ok(const DSL_intArray &theCoordinates) const;
  int ReCreate(const DSL_intArray &theseDimensions) { return ReCreate(theseDimensions.Items(), theseDimensions.NumItems()); }
  int ReCreate(const int * dimensions, int dimCount);
  void CalculatePreProd();
  int RemoveDimension(int thisDimension, int preservingThis, DSL_intArray &targetDims, DSL_doubleArray &targetData);
  int RemoveDimension(int thisDimension, const DSL_doubleArray &usiingTheseValues, DSL_intArray &targetDims, DSL_doubleArray &targetData);

  // NOTE: these three arrays are used internally as arrays  <<<<-- IMPORTANT
  DSL_intArray    theDimensions;
  DSL_intArray    thePreProduct; // precalculated products of dimensions
  DSL_doubleArray theData;
};

#endif 
