/*

SMILE ACADEMIC version 1.4.0 / 2019-07-31
SMILE: Structural Modeling, Inference and Learning Engine

This software can be used only with a valid license, obtained from BayesFusion, LLC.
The following document contains full terms and conditions of SMILE Academic license:
https://download.bayesfusion.com/license_academic.txt

SMILE Academic can be used without cost for academic teaching and research use.
We would like to stress that "academic teaching and research use" means using the 
software (1) for the purpose of academic teaching or research as part of an 
academic program or an academic research project, and (2) by a user who is 
at the time of use affiliated with an academic institution. In other words, 
affiliation with an academic institution alone, research conducted at government 
or industrial research centers, research conducted by members of university 
faculty in consulting projects, or use in a commercial educational institution 
DO NOT qualify as academic teaching and research use.

The documentation is available at:
http://support.bayesfusion.com/

*/


#ifndef SMILE_H
#define SMILE_H

// {{SMILE_PUBLIC_HEADER}}

// client-side SMILE header
// DO NOT USE for internal SMILE development

#ifdef _MSC_VER
#ifndef SMILE_VC_NO_AUTOLINK
#include "autolink.h"
#endif
#endif

// basic data structures
#include "intarray.h"
#include "doublearray.h"
#include "dmatrix.h"
#include "syscoord.h"
#include "generalclases.h"

// network structure
#include "network.h"
#include "node.h"
#include "submodel.h"
#include "simplecase.h"
#include "errorstrings.h"
#include "progress.h"
#include "randgen.h"

// supported node definition classes
#include "defcpt.h"
#include "deftruthtable.h"
#include "defnoisymax.h"
#include "defnoisyadder.h"
#include "deflist.h"
#include "deftable.h"
#include "defmau.h"
#include "defdemorgan.h"
#include "defequation.h"

// supported node value classes
#include "valbeliefvector.h"
#include "vallistofdecisions.h"
#include "valexpectedutility.h"
#include "valmauexpectedutility.h"
#include "valequationevaluation.h"

// voi
#include "valueofinfo.h"

// sensitivity
#include "sensitivity.h"

// define SMILE_NO_DIAGNOSIS before including smile.h to exclude diagnosis
#ifndef SMILE_NO_DIAGNOSIS
#include "diag_network.h"
#include "nodecost.h"
#include "extradefinition.h"
#include "caselibrary.h"
#endif // !SMILE_NO_DIAGNOSIS

// define SMILE_NO_LEARNING before including smile.h to exclude learning
#ifndef SMILE_NO_LEARNING
#include "dataset.h"
#include "datagenerator.h"
#include "validator.h"
#include "discretizer.h"
#include "pattern.h"
#include "bkgndknowledge.h"
#include "em.h"
#include "bs.h"
#include "pc.h"
#include "nb.h"
#include "tan.h"
#include "abn.h"
#include "dbcml.h"
#include "essentialsearch.h"
#endif // !SMILE_NO_LEARNING

#endif
