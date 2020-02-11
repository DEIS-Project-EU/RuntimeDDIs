#ifndef INTARRAY_H
#define INTARRAY_H

// {{SMILE_PUBLIC_HEADER}}

#include <cassert>
#include <cstring>

// tomek, may 27, 2000
// we're using local storage for arrays of small sizes.
// this has real and positive impact on performance, since
// we're using lots of small arrays

// tomek, july 31, 2003
// reducend local storage
// the exact size of local storage is defined by 
// localsize enum. It used to be 32, but it caused
// problems with excessive memory usage for large 
// (1000+ nodes) models.


class DSL_intArray
{
public:
	DSL_intArray() 
	{
		items = localItems;
		numitems = size = 0;
	}

	DSL_intArray(int initialSize)
	{
		items = localItems;
		numitems = size = 0;
		SetSize(initialSize);
	}

	DSL_intArray(const DSL_intArray &likeThisOne)
	{
		items = 0;
		numitems = size = 0;

		SetSize(likeThisOne.size);
		numitems = likeThisOne.numitems;

		CopyItems(likeThisOne.items, items, size);
	}

  	~DSL_intArray() { if (DynamicMode()) delete [] items; }

	int operator=(const DSL_intArray &likeThisOne);
	void Swap(DSL_intArray &other);

	bool operator==(const DSL_intArray &likeThisOne) const;
	bool operator<(const DSL_intArray &likeThisOne) const;

	int &operator[](int index) { assert(Ok(index)); return items[index]; }
	const int &operator[](int index) const { assert(Ok(index)); return items[index]; }

	int GetSize() const { return size; }
	int NumItems() const { return numitems; }
	void UseAsList(int nItems = -1) { if (Ok(nItems)) numitems = nItems; else numitems = size;};
	int &Last();
	int &List_Last();
	int &Subscript(int index) const;
	int  Add(int thisNumber);
	int  AddExclusive(int thisNumber);
	int  Insert(int here, int thisNumber); 
	int  Delete(int index);
	int  DeleteByContent(int thisContent);
	int  DeleteDuplicates(void);
	int  FindPosition(int ofThisNumber) const;
	int  IsInList(int aNumber) const;
	int  RoomGuaranteed(int forThisPeople);
	int  SetSize(int thisSize);
	int *Items() { return items; }
	const int *Items() const { return items; }
	void Flush(void);
	int  FillFrom(DSL_intArray &thisOne);
	void FillWith(int thisNumber);
	int  ChangeOrder(const DSL_intArray &newPositions);
	void CleanUp();
  
private:
	enum { localSize = 8 };
	int localItems[localSize];
	int *items;   
	int size;
	int numitems;
	
	bool DynamicMode() const { return items && (items != localItems); }

	void CopyItems(int *source, int *target, int itemCount)
	{
		memcpy(target, source, itemCount * sizeof(int));
	}

	int Grow();
	int Full() const { return size == numitems; }
	bool Ok(int index) const { return index >= 0 && index < size; }

	void SwapScalars(DSL_intArray &other);
	void SwapWithDynamic(DSL_intArray &other);
};

#endif
