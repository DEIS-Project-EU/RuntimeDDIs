#ifndef DSL_ERRORS_H
#define DSL_ERRORS_H

// {{SMILE_PUBLIC_HEADER}}

// definition of all internal SMILE error codes
#define DSL_OKAY                    0
#define DSL_GENERAL_ERROR          -1
#define DSL_OUT_OF_RANGE           -2
#define DSL_NO_ITEM                -3
#define DSL_INVALID_VALUE          -4
#define DSL_NO_USEFUL_SAMPLES      -5
#define DSL_CANT_SOLVE_EQUATION    -6
#define DSL_CYCLE_DETECTED         -11
#define DSL_WRONG_NUM_STATES       -19
#define DSL_CONFLICTING_EVIDENCE  -26
#define DSL_ILLEGAL_ID            -30
#define DSL_DUPLICATED_ID         -32
#define DSL_OUT_OF_MEMORY         -42
#define DSL_ZERO_POTENTIAL		  -43
#define DSL_WRONG_NODE_TYPE       -51
#define DSL_WRONG_ELEMENT_TYPE		-52
#define DSL_INTERRUPTED        -99

//textfile
#define DSL_FILE_READ         -100
#define DSL_FILE_WRITE        -101
#define DSL_END_OF_FILE       -102   
#define DSL_WRONG_FILE        -103

// lexical/syntactical
#define DSL_NO_MORE_TOKENS        -111
#define DSL_LEXICAL_ERROR         -126
#define DSL_SYNTAX_ERROR          -127
#define DSL_UNEXPECTED_EOF        -128
#define DSL_FIELD_NOT_FOUND       -129

#define DSL_OBJECT_NOT_READY      -567
#define DSL_OBJECT_NOT_CONSISTENT -568

#endif 
