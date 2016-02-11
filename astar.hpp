#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <set>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <tuple>
#include <list>

#define DEBUG

template<typename TProblem>
struct AStar
{
	struct compNodePtr
	{
		bool operator()( const typename TProblem::Node lhs,
				const typename TProblem::Node rhs )
		{
			return (lhs) < (rhs);
		}
	};

	AStar( TProblem p ) : mProblem( p ), mStart( p.mStart, NULL, 0.0f )
		{}

	std::vector<typename TProblem::Node> solve()
	{
		std::vector<typename TProblem::Node> frontier;
		std::list<typename TProblem::Node> visited;
		frontier.push_back( mStart );
		visited.push_back( mStart );

#ifdef DEBUG
		expandedNodes = 0;
#endif

		while( !frontier.empty() )
		{
			for( unsigned int i=0; i < frontier.size(); ++i )
			{

				visited.push_back(frontier.front());

				if( mProblem.checkSolution(visited.back()) ) return buildSolution(visited.back());

				std::pop_heap( frontier.begin(), frontier.end() );
				frontier.pop_back();

				std::vector<typename TProblem::Node> successors = mProblem.getSuccessors(visited.back());
				computeHeuristics( successors );
#ifdef DEBUG
				expandedNodes++;
#endif
				for( unsigned int j=0; j < successors.size(); ++j )
				{
					if( std::find(visited.begin(), visited.end(), successors[j]) == visited.end() )
					{
						frontier.push_back( successors[j] );
						std::push_heap( frontier.begin(), frontier.end() );
					}
				}
			}
		}

		return std::vector<typename TProblem::Node>();
	}

	std::vector<typename TProblem::Node> buildSolution( typename TProblem::Node n )
	{
		std::vector<typename TProblem::Node> result;
		const typename TProblem::Node* it = &n;
		while( it != NULL )
		{
			result.push_back( *it );
			it = it->mPrev;
		}
		std::reverse( result.begin(), result.end());
		solutionDepth = result.size();

		return result;
	}

	void computeHeuristics( std::vector<typename TProblem::Node>& succ )
	{
		for( int i=0; i < succ.size(); ++i )
		{
			succ[i].mF += mProblem.heuristic( succ[i].mData );
		}
	}

	TProblem mProblem;
	typename TProblem::Node mStart;
	
	int solutionDepth;

#ifdef DEBUG
	int expandedNodes;
#endif
};

#endif

