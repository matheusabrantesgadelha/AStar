#ifndef KNIGHTPATH_HPP
#define KNIGHTPATH_HPP

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

struct KnightPath
{
	typedef std::pair<int,int> NodeData;

	struct Node
	{
		Node( NodeData nd, const Node* previous, float f ) : mData( nd ), mPrev( previous ), mF( f ){}

		friend bool operator<( const Node& a, const Node& b )
		{
			return a.mF > b.mF;
		}

		friend bool operator!=( const Node& a, const Node& b )
		{
			return a.mData != b.mData;
		}

		friend bool operator==( const Node& a, const Node& b )
		{
			return a.mData == b.mData;
		}

		~Node()
		{
		}

		NodeData mData;
		const Node* mPrev;
		float mF;
	};

	KnightPath(NodeData start, NodeData goal) :
		mStart( start ), mGoal( goal )
	{
	};

	std::vector<Node> getSuccessors( const Node& n )
	{
		std::vector<Node> successors;

		successors.push_back( Node( std::make_pair( n.mData.first+1, n.mData.second+2 ), &n, 1.0f + n.mF ));
		successors.push_back( Node( std::make_pair( n.mData.first-1, n.mData.second+2 ), &n, 1.0f + n.mF ));
		successors.push_back( Node( std::make_pair( n.mData.first-1, n.mData.second-2 ), &n, 1.0f + n.mF ));
		successors.push_back( Node( std::make_pair( n.mData.first+1, n.mData.second-2 ), &n, 1.0f + n.mF ));

		successors.push_back( Node( std::make_pair( n.mData.first+2, n.mData.second+1 ), &n, 1.0f + n.mF ));
		successors.push_back( Node( std::make_pair( n.mData.first-2, n.mData.second+1 ), &n, 1.0f + n.mF ));
		successors.push_back( Node( std::make_pair( n.mData.first-2, n.mData.second-1 ), &n, 1.0f + n.mF ));
		successors.push_back( Node( std::make_pair( n.mData.first+2, n.mData.second-1 ), &n, 1.0f + n.mF ));

		return successors;
	}
	
	float heuristic( NodeData& n )
	{
		float h = abs(n.first - mGoal.first);
		h += abs(n.second - n.second);
		return float((int)h/3);
	}

	bool checkSolution( const Node& n )
	{
		if( n.mData == mGoal ) return true;
		return false;
	}

	NodeData mStart;
	NodeData mGoal;
};

#endif
