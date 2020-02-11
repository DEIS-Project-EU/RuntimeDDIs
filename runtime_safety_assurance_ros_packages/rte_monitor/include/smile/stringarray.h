#ifndef DSL_STRINGARRAY_H
#define DSL_STRINGARRAY_H

// {{SMILE_PUBLIC_HEADER}}

#include "dslobject.h"

class DSL_intArray;

class DSL_stringArray : public DSL_object
{
public:
    DSL_stringArray();
    DSL_stringArray(int initialSize, int initialDelta=10);
    DSL_stringArray(const DSL_stringArray &likeThisOne);
    ~DSL_stringArray() { CleanUp(); }
    int operator=(const DSL_stringArray &likeThisOne);
    char *operator[](int index) { return items[index]; }
    char *Subscript(int index);
    const char * operator[](int index) const { return items[index]; }
    const char * Subscript(int index) const;
    int GetSize() const { return size; }
    int NumItems() const { return numitems; }
    virtual int SetString(int thisPosition, const char *thisString);
    virtual int Add(const char *thisSring);
    virtual int Insert(int here, const char *thisString);
    int Delete(int index);
    int DeleteByContent(char *thisContent);
    int FindPosition(const char *ofThisString) const;
    int IsInList(const char *thisString) const;
    void Flush();
    int RoomGuaranteed(int forThisPeople);
    int SetSize(int thisSize);
    char **Items() {return(items);};
    int FillFrom(DSL_stringArray &thisOne);
    int ChangeOrder(DSL_intArray &newPos);
    void CheckReadiness(int deep = 0);
    void CleanUp(int deep = 0);
    void UseAsList(int nItems = -1) { if (Ok(nItems)) numitems = nItems; else numitems = size; };

protected:
    int Ok(int index) const { return index >= 0 && index<size; }
    int Full() const { return size == numitems; }
    int Grow();
    void DeleteString(int atThisPosition);
    void ChangeString(int atThisPosition, const char *newString);

    char **items;
    int size;
    int numitems;
    int delta;
    
};

#endif
