#ifndef DSL_ESSENTIAL_SEARCH_H
#define DSL_ESSENTIAL_SEARCH_H

// {{SMILE_PUBLIC_HEADER}}

#include "bkgndknowledge.h"

class DSL_network;
class DSL_dataset;

class DSL_essentialSearch
{
public:
	DSL_essentialSearch()
	{
        maxAdjacency = 8;
        minSignificance = 0.01;
        maxSignificance = 0.1;
        restarts = 1000;
        maxSearchTime = 0;
	}

	int Learn(const DSL_dataset &data, DSL_network &net);
	
    int maxAdjacency;
    int restarts;
    int maxSearchTime;
    double minSignificance;
    double maxSignificance;

	DSL_bkgndKnowledge bkk;
};

#endif
