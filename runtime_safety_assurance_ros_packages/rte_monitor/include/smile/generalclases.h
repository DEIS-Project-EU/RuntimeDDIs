#ifndef SMILE_GENERALCLASES_H
#define SMILE_GENERALCLASES_H

// {{SMILE_PUBLIC_HEADER}}

#include "stringarray.h"
#include "idarray.h"
#include <vector>

class DSL_node;

//////////////////////////////////////////////////////////////////////////////
// class DSL_header
//////////////////////////////////////////////////////////////////////////////

class DSL_header : public DSL_object
{
public:
	DSL_header();
	DSL_header(DSL_header &likeThisOne);
	~DSL_header();
	int operator =(const DSL_header &likeThisOne);
	void CleanUp(int deep = 0);
	int SetId(const char *theID);
	int SetName(const char *theName);
	int SetComment(const char *theComment);
	const char *GetId() const { return id; }
	const char *GetName() const { return name; }
	const char *GetComment() const { return comment; }
	static int IsThisIdValid(const char *theID);
	static int MakeValidId(char *theID);
	void CheckConsistency(int deep = 1);
	void AssociateWithNode(DSL_node *n);
private:
	char *id;       // For quick reference (including formulae), no spaces.
	char *name;     // Longer name for display and printing, spaces allowed.
	char *comment;  // A possibly longer text
	DSL_node *node; // node associated with this header (may be NULL for network/submodel headers)
};

class DSL_rectangle : public DSL_object
{
public: // no control, so make them public
    int center_X;
    int center_Y;
    int width;
    int height;

public:
    DSL_rectangle(void);
    DSL_rectangle(const DSL_rectangle &likeThisOne);
    int operator =(const DSL_rectangle &likeThisOne);
    int IsInside(int X, int Y);
    int FillDefaultValues(DSL_rectangle &fromHere);
    void Set(int cX, int cY, int W, int H) { center_X = cX; center_Y = cY, width = W; height = H; };
};


//////////////////////////////////////////////////////////////////////////////
// class DSL_screenInfo
//////////////////////////////////////////////////////////////////////////////

// DSL_screenInfo flags 
#define DSL_SHOW_IN_ARCS   1  // Set if incoming arcs should be displayed.
#define DSL_SHOW_OUT_ARCS  2  // Set if outgoing arcs should be displayed.

#define DSL_DEFAULT_SCREENINFO_FLAGS (DSL_SHOW_IN_ARCS | DSL_SHOW_OUT_ARCS)

class DSL_screenInfo : public DSL_object
{
 public:
  DSL_rectangle position; // center, width and height
  int color;              // default color
  int selColor;           // color when selected.
  int font;               // Font used to display info
  int fontColor;
  int borderThickness;
  int borderColor;

 protected:
  int flags;

 public:
  DSL_screenInfo(void);
  DSL_screenInfo(const DSL_screenInfo &likeThisOne);
  int operator =(const DSL_screenInfo &likeThisOne);
  int ShowInArcs(void)      {return(flags & DSL_SHOW_IN_ARCS);};
  int ShowOutArcs(void)     {return(flags & DSL_SHOW_OUT_ARCS);};  
  void SetShowInArcs(void)  {flags |= DSL_SHOW_IN_ARCS;};
  void SetHideInArcs(void)  {flags &= ~DSL_SHOW_IN_ARCS;};
  void SetShowOutArcs(void) {flags |= DSL_SHOW_OUT_ARCS;};  
  void SetHideOutArcs(void) {flags &= ~DSL_SHOW_OUT_ARCS;};  
  int FillDefaultValues(DSL_screenInfo &fromHere);
};

//////////////////////////////////////////////////////////////////////////////
// class DSL_creation
//////////////////////////////////////////////////////////////////////////////

class DSL_creation : public DSL_object
{
 protected:
  char *creator;   // Name of the creator
  char *created;   // Creation date.
  char *modified;  // Last modification date.

 public:  
  DSL_creation(void);
  DSL_creation(const DSL_creation &likeThisOne);
 ~DSL_creation();
  int operator =(const DSL_creation &likeThisOne);
  void CleanUp(int deep = 0);
  int SetCreator(const char *theCreator);
  int SetCreated(const char *thisDate);
  int SetModified(const char *thisDate);
  char *GetCreator(void) const {return(creator);};
  char *GetCreated(void) const {return(created);};
  char *GetModified(void) const {return(modified);};
};

//////////////////////////////////////////////////////////////////////////////
// class DSL_textBox
//////////////////////////////////////////////////////////////////////////////

typedef std::vector<std::pair<std::string, DSL_rectangle> > DSL_textBoxList;

/*
class DSL_textBoxList : public DSL_object
{
 // this class implements a list of comments on the screen.
 // For each comment we have a rectangle with its position
 // and dimensions and a string with the comment itself
public:  
    DSL_textBoxList();
 
    std::vector<DSL_rectangle> &GetPositions() { return positions; }
    DSL_stringArray &GetStrings() { return strings; }
    const char * GetString(int idx) const { return strings[idx]; }
    const DSL_rectangle &GetPosition(int idx) const { return positions[idx]; }
    int NumItems() const { return strings.NumItems(); }
    int operator=(const DSL_textBoxList &likeThisOne);

private:
    std::vector<DSL_rectangle>  positions;
    DSL_stringArray strings;
};
*/

//////////////////////////////////////////////////////////////////////////////
// class DSL_userProperties
//////////////////////////////////////////////////////////////////////////////

class DSL_userProperties : public DSL_object
{
 private:
  // NOTE: a user property is a pair ([id],[string])
  DSL_idArray names;
  DSL_stringArray values;

 public:
  DSL_userProperties();
  DSL_userProperties(const DSL_userProperties &likeThisOne);
 ~DSL_userProperties();
  int operator =(const DSL_userProperties &likeThisOne);
  void CleanUp(int deep = 0);

  int AddProperty(const char *propertyName, const char *propertyValue);
  int InsertProperty(int here, const char *propertyName, const char *propertyValue);
  const char *GetPropertyName(int index) const {return(names[index]);}; // no checking !!!
  const char *GetPropertyValue(int index) const {return(values[index]);}; // no checking !!!
  int ChangePropertyName(int thisOne, const char *thisName);
  int ChangePropertyValue(int thisOne, const char *thisValue);

  int FindProperty(const char *withThisName) const;
  int DeleteProperty(int thisOne);
  int DeleteAllProperties();
  int GetNumberOfProperties() const ;

  void CheckConsistency(int deep = 1);
};

//////////////////////////////////////////////////////////////////////////////
// class DSL_nodeDocumentation
//////////////////////////////////////////////////////////////////////////////

class DSL_documentation : public DSL_object
{
 // this class represents the documentation part on another
 // component like, for example, a state of a node.
 private:
  // NOTE: a document is a pair ([title],[path])
  DSL_stringArray titles;
  DSL_stringArray paths;

 public:
  DSL_documentation(void);
  DSL_documentation(const DSL_documentation &likeThisOne);
 ~DSL_documentation();
  int operator =(const DSL_documentation &likeThisOne);
  void CleanUp(int deep = 0);

  int AddDocument(const char *title, const char *path);
  int InsertDocument(int here, const char *title, const char *path);
  char *GetDocumentTitle(int index) {return(titles[index]);}; // no checking !!!
  char *GetDocumentPath(int index) {return(paths[index]);}; // no checking !!!
  int ChangeDocumentTitle(int thisOne, char *newTitle);
  int ChangeDocumentPath(int thisOne, char *newPath);

  int FindDocument(char *withThisTitle);
  int DeleteDocument(int thisOne);
  int DeleteAllDocuments(void);
  int GetNumberOfDocuments(void);

  void CheckConsistency(int deep = 1);
};


#endif
