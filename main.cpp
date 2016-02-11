#include <iostream>
#include <vector>
#include <cmath>
#include <set>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <tuple>
#include <list>

#include "astar.hpp"
#include "knightpath.hpp"
#include "tsp.hpp"

int main( int argc, char* argv[] )
{
	clock_t start, end;
	std::cout << "Nodes Expanded\tTime(ms)\tDepth" << std::endl;

//	for( int i=0; i<10; ++i )
//	{
//		int x = rand() % 15;
//		int y = rand() % 15;
//
//		KnightPath kp( std::make_pair(0,0), std::make_pair(10,10) );
//		AStar<KnightPath> astar( kp );
//		start = clock();
//		astar.solve();
//		end = clock();
//		int msecs = ((double)(end-start))*1000 / CLOCKS_PER_SEC;
//		std::cout << astar.expandedNodes << "\t" << msecs << "\t" << astar.solutionDepth << std::endl;
//	}

	for( int i=3; i<11; ++i )
	{
		TSP tsp(i);
		AStar<TSP> astar( tsp );
		start = clock();
		astar.solve();
		end = clock();
		int msecs = ((double)(end-start))*1000 / CLOCKS_PER_SEC;
		std::cout << astar.expandedNodes << "\t" << msecs << "\t" << astar.solutionDepth << std::endl;
	}


	return 0;
}
