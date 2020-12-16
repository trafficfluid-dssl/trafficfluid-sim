#include <stdio.h>


#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree_win.h>
#include "libLaneFreePlugin_Export.h"
#endif

void print_test() {
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	
	for (int i = 0; i < n_myedges; i++) {
		printf("%lld", myedges[i]);
	}

}