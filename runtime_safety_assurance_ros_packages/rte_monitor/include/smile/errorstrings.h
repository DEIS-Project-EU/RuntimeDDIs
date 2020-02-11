#ifndef ERRORSTRINGS_H
#define ERRORSTRINGS_H

// {{SMILE_PUBLIC_HEADER}}

#include "intarray.h"
#include "stringarray.h"

class DSL_errorStringRedirect
{
public:
	virtual ~DSL_errorStringRedirect() {}
	virtual void LogError(int code, const char *message) = 0;
};

class DSL_errorStringHandler
{
public:
	DSL_errorStringHandler() { redirection = NULL; ownedRedirection = false; }
	~DSL_errorStringHandler() { if (ownedRedirection) delete redirection; }

	int LogError(int theCode, const char *theMessage = NULL, const char *prefix = NULL);
	int GetError(int thisOne);
	int GetLastError();
	const char* GetErrorMessage(int thisOne);
	const char* GetLastErrorMessage();
	int GetNumberOfErrors() { return errorCodes.NumItems(); }
	void Flush();

	// stdout/stderr can be passed as file arg
	void RedirectToFile(FILE *file, const char *format = NULL);
	void Redirect(DSL_errorStringRedirect *newRedirect);
   
private:
	static const char * GetDefaultErrorString(int forThisCode);
	DSL_stringArray errorStrings;
	DSL_intArray    errorCodes;
	DSL_errorStringRedirect *redirection;
	bool ownedRedirection;
};

// access to global error handler
DSL_errorStringHandler& DSL_errorH();

#endif 
