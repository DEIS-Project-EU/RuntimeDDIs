#ifndef DSL_CONSTANTS_H
#define DSL_CONSTANTS_H

// {{SMILE_PUBLIC_HEADER}}

#include <climits>
#include <cfloat>

// definition of some DSL_GNL constants
#define DSL_DEFAULT_SUBMODEL_WIDTH             76
#define DSL_DEFAULT_SUBMODEL_HEIGHT            36
#define DSL_DEFAULT_SUBMODEL_COLOR             0x00f7f6e5     //lightblue
#define DSL_DEFAULT_SUBMODEL_SELCOLOR          0x00bbbbbb     // medium grey
#define DSL_DEFAULT_SUBMODEL_FONT              1
#define DSL_DEFAULT_SUBMODEL_FONTCOLOR         0x00000000     //black
#define DSL_DEFAULT_SUBMODEL_BORDERTHICKNESS   3
#define DSL_DEFAULT_SUBMODEL_BORDERCOLOR       0x00bb0000	    //blue
#define DSL_DEFAULT_SUBMODEL_BKCOLOR           0x00ffffff	    //white
#define DSL_DEFAULT_NODE_WIDTH                 76
#define DSL_DEFAULT_NODE_HEIGHT                36
#define DSL_DEFAULT_NODE_COLOR                 0x00f7f6e5     //lightblue
#define DSL_DEFAULT_NODE_SELCOLOR              0x00bbbbbb     // medium grey
#define DSL_DEFAULT_NODE_FONT                  1 
#define DSL_DEFAULT_NODE_FONTCOLOR             0x00000000     //black
#define DSL_DEFAULT_NODE_BORDERTHICKNESS       1 
#define DSL_DEFAULT_NODE_BORDERCOLOR           0x00bb0000	    //blue


// DEFINED IN <LIMITS.H>
//#define INT_MIN     (-2147483647 - 1) /* minimum (signed) int value */
#define DSL_UNDEFINED_INT    (INT_MIN+1) 

// DEFINED IN <FLOAT.H>
// #define DBL_MIN 	2.2250738585072014e-308 /* min positive value */
#define DSL_UNDEFINED_DOUBLE (DBL_MIN * 2) 
#define DSL_ILLEGAL_DOUBLE   (DBL_MIN * 3) 
#define DSL_NOT_AVAILABLE    (DBL_MIN * 4)
#define DSL_NOT_RELEVANT    (DBL_MIN * 5)

// the text representation (for the user) of the 
// values above is as follows:
#define DSL_UNDEFINED_DOUBLE_TEXT "UNDEF."
#define DSL_ILLEGAL_DOUBLE_TEXT   "ILLEGAL" 
#define DSL_NOT_AVAILABLE_TEXT    "NA"
// the text representation (for the files) of the 
// values above is as follows:
#define DSL_UNDEFINED_DOUBLE_TEXT_FILE "0.0E-2"
#define DSL_ILLEGAL_DOUBLE_TEXT_FILE   "0.0E-3" 
#define DSL_NOT_AVAILABLE_TEXT_FILE    "0.0E-4"

// DEFINED IN <FLOAT.H>
//#define DBL_MAX 	1.7976931348623158e+308 /* max value */
#define DSL_MINIMUM_DOUBLE   (-DBL_MAX)

// file formats
#define DSL_ERGO_FORMAT         1
#define DSL_NETICA_FORMAT       2
#define DSL_INTERCHANGE_FORMAT  3 
#define DSL_DSL_FORMAT          4
#define DSL_HUGIN_FORMAT        5
#define DSL_XMLBIF_FORMAT       6
#define DSL_KI_FORMAT           7
#define DSL_XDSL_FORMAT         8


// BN algorithms
#define DSL_ALG_BN_LAURITZEN              0
#define DSL_ALG_BN_HENRION                1
#define DSL_ALG_BN_PEARL                  2
#define DSL_ALG_BN_LSAMPLING              3
#define DSL_ALG_BN_SELFIMPORTANCE         4
#define DSL_ALG_BN_HEURISTICIMPORTANCE    5
#define DSL_ALG_BN_BACKSAMPLING           6
#define DSL_ALG_BN_AISSAMPLING            7
#define DSL_ALG_BN_EPISSAMPLING           8
#define DSL_ALG_BN_LBP					  9
#define DSL_ALG_BN_LAURITZEN_OLD          10
#define DSL_ALG_BN_RELEVANCEDECOMP          11
#define DSL_ALG_BN_RELEVANCEDECOMP2       12

// ID algorithms
#define DSL_ALG_ID_COOPERSOLVING          0
#define DSL_ALG_ID_SHACHTER               1

// Approaches for multiple fault diagnosis
#define DSL_DIAG_MARGINAL		1
#define DSL_DIAG_INDEPENDENCE	2
#define DSL_DIAG_DEPENDENCE		4

// Selections of combinations to pursue
#define DSL_DIAG_PURSUE_ATLEAST_ONE_COMB	8
#define DSL_DIAG_PURSUE_ONLY_ONE_COMB		16
#define DSL_DIAG_PURSUE_ONLY_ALL_COMB		32

// Marginal Function for the marginal approach
#define DSL_DIAG_MARGINAL_STRENGTH1	64
#define DSL_DIAG_MARGINAL_STRENGTH2	128


#endif 
