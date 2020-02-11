#ifndef DOUBLEARRAY_H
#define DOUBLEARRAY_H

// {{SMILE_PUBLIC_HEADER}}

#include <cassert>

class DSL_intArray;

class DSL_doubleArray
{
public:
	DSL_doubleArray()
	{
		items = 0;
		size = numitems = 0;
	}

	DSL_doubleArray(int initialSize)
	{
		items = 0;
		size = numitems = 0;
		SetSize(initialSize);
	}

	DSL_doubleArray(const DSL_doubleArray &likeThisOne);

	~DSL_doubleArray() { CleanUp(); }
  
  
	int  operator=(const DSL_doubleArray &likeThisOne);
	void Swap(DSL_doubleArray &other);

	double &operator[](int index) { assert(Ok(index)); return items[index]; }
	const double &operator[](int index) const { assert(Ok(index)); return items[index]; }
	double &Subscript(int index);
	double Subscript(int index) const;

	int GetSize() const { return size; }
	int NumItems() const { return numitems; }
	void UseAsList(int nItems = -1) {if (Ok(nItems)) numitems = nItems; else numitems = size;};
	int Add(double thisNumber);
	int Insert(int here, double thisNumber);
	int Delete(int index);
	int DeleteByContent(double thisContent);
	int FindPosition(double ofThisNumber);
	int IsInList(double aNumber);
	void Flush(void) {numitems = 0;};
	int RoomGuaranteed(int forThisPeople);
	int SetSize(int thisSize);
	double* Items() { return items; }
	const double* Items() const { return items; }
	int  FillFrom(const DSL_doubleArray &thisOne);
	void FillWith(double thisValue);
	int  ChangeOrder(const DSL_intArray &newPositions);
	void CleanUp();

private:
	int Grow(void);
	int Full(void) { return size == numitems; }
	bool Ok(int index) const { return index >= 0 && index < size; }
	bool DynamicMode() const { return items && (items != localItems); }
	
	void SwapScalars(DSL_doubleArray &other);
	void SwapWithDynamic(DSL_doubleArray &other);

	enum { localSize = 4 };
	double localItems[localSize];
	double *items;   
	int size;
	int numitems;
};

#endif
