#include <iostream>
#include <vector>
#include <cmath>
#include <set>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <tuple>

#define DEBUG

template<typename T>
bool exists( std::vector<T*> v, T val )
{
	for( int i=0; i<v.size(); ++i )
	{
		if( *(v[i]) != val ) return false;
	}
	return true;
}

template<typename TProblem>
struct AStar
{
	struct compNodePtr
	{
		bool operator()( const typename TProblem::Node* lhs,
				const typename TProblem::Node* rhs )
		{
			return (*lhs) < (*rhs);
		}
	};

	AStar( TProblem p ) : mProblem( p ), mStart( p.mStart, NULL, 0.0f ),
		mGoal( p.mGoal, NULL, 0.0f ) {}

	std::vector<typename TProblem::Node> solve()
	{
		std::vector<typename TProblem::Node> frontier;
		std::set<typename TProblem::Node*, compNodePtr> visited;
		frontier.push_back( mStart );
		visited.insert( &mStart );

#ifdef DEBUG
		expandedNodes = 0;
#endif

		while( !frontier.empty() )
		{
			for( unsigned int i=0; i < frontier.size(); ++i )
			{
				typename TProblem::Node* currentNode = new typename TProblem::Node( 
						frontier.front().mData, frontier.front().mPrev, frontier.front().mF );

				if( (*currentNode).mData == mProblem.mGoal ) return buildSolution( currentNode );

				visited.insert(currentNode);

				std::pop_heap( frontier.begin(), frontier.end() );
				frontier.pop_back();

				std::vector<typename TProblem::Node> successors = mProblem.getSuccessors( currentNode );
				computeHeuristics( successors );
#ifdef DEBUG
				expandedNodes++;
#endif
				for( unsigned int j=0; j < successors.size(); ++j )
				{
					if( visited.find(&successors[j]) == visited.end() )
					{
						frontier.push_back( successors[j] );
						std::push_heap( frontier.begin(), frontier.end() );
					}
				}
			}
		}
	}

	std::vector<typename TProblem::Node> buildSolution( typename TProblem::Node* n )
	{
		std::vector<typename TProblem::Node> result;
		typename TProblem::Node* it = n;
		while( it != NULL )
		{
			result.push_back( *it );
			it = it->mPrev;
		}
		std::reverse( result.begin(), result.end());
		solutionDepth = result.size();
//		for( int i=0; i< result.size(); ++i )
//		{
//			std::cout << result[i].mData.first << " " << result[i].mData.second << std::endl;
//			std::cout << std::endl;
//		}

#ifdef DEBUG
//		std::cout << "Expanded nodes: " << expandedNodes << std::endl;
#endif

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
	typename TProblem::Node mGoal;
	
	int solutionDepth;

#ifdef DEBUG
	int expandedNodes;
#endif
};

struct KnightPath
{
	typedef std::pair<int,int> NodeData;

	struct Node
	{
		Node( NodeData nd, Node* previous, float f ) : mData( nd ), mPrev( previous ), mF( f ){}

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

		NodeData mData;
		Node* mPrev;
		float mF;
	};

	KnightPath(NodeData start, NodeData goal) :
		mStart( start ), mGoal( goal )
	{
	};

	std::vector<Node> getSuccessors( Node* n )
	{
		std::vector<Node> successors;

		successors.push_back( Node( std::make_pair( n->mData.first+1, n->mData.second+2 ), n, 1.0f + n->mF ));
		successors.push_back( Node( std::make_pair( n->mData.first-1, n->mData.second+2 ), n, 1.0f + n->mF ));
		successors.push_back( Node( std::make_pair( n->mData.first-1, n->mData.second-2 ), n, 1.0f + n->mF ));
		successors.push_back( Node( std::make_pair( n->mData.first+1, n->mData.second-2 ), n, 1.0f + n->mF ));

		successors.push_back( Node( std::make_pair( n->mData.first+2, n->mData.second+1 ), n, 1.0f + n->mF ));
		successors.push_back( Node( std::make_pair( n->mData.first-2, n->mData.second+1 ), n, 1.0f + n->mF ));
		successors.push_back( Node( std::make_pair( n->mData.first-2, n->mData.second-1 ), n, 1.0f + n->mF ));
		successors.push_back( Node( std::make_pair( n->mData.first+2, n->mData.second-1 ), n, 1.0f + n->mF ));

		return successors;
	}
	
	float heuristic( NodeData& n )
	{
		float h = abs(n.first - mGoal.first);
		h += abs(n.second - n.second);
		return float((int)h/3);
	}

	NodeData mStart;
	NodeData mGoal;
};

struct TSP
{
	struct Edge
	{
		Edge( std::pair<int,int> verts, float cost ):
			mVerts( verts ), mCost( cost ){}
		friend bool operator==( const Edge& a, const Edge& b )
		{
			return (a.mVerts.first == b.mVerts.first && a.mVerts.second == b.mVerts.second )
				|| (a.mVerts.second == b.mVerts.first && a.mVerts.first == b.mVerts.second );
		}
		std::pair<int,int> mVerts;
		float mCost;
	};

	typedef std::vector<Edge> NodeData;

	struct Node
	{
		Node( NodeData nd, Node* previous, float f ) : mData( nd ), mPrev( previous ), mF( f ){}

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
			if( a.mData.size() != b.mData.size() ) return false;

			for( size_t i=0; i<a.size(); ++i )
			{
			}
			return a.mData == b.mData;
		}

		NodeData mData;
		Node* mPrev;
		float mF;
	};

	TSP( const unsigned int nVertices ) : adjacencyVec( nVertices, std::vector<Edge>() );
	{
		std::vector< std::pair<float,float> > vertices;
		for( unsigned int i=0; i < nVertices; ++i )
		{
			vertices.push_back( std::make_pair( 
						(rand() % 1001) / 1000.0f, (rand() % 1001) / 1000.0f ));
		}

		for( unsigned int i=0; i<nVertices; ++i )
		{
			for( int j=i+1; j<nVertices-1; ++j )
			{
				float dist = sqrt( pow(vertices[i].first - vertices[j].first, 2) + 
					pow(vertices[i].second - vertices[j].second, 2) );
				adjacencyVec[i].push_back( Edge( std::make_pair(i,j), dist ));
				adjacencyVec[j].push_back( Edge( std::make_pair(j,i), dist ));
			}
		}
	}

	std::vector<std::vector<Edge> > adjacencyVec;
};

int main( int argc, char* argv[] )
{
	clock_t start, end;
	std::cout << "Nodes Expanded\tTime(ms)\tDepth" << std::endl;
	for( int i=0; i<1000; ++i )
	{
		int x = rand() % 16;
		int y = rand() % 16;
		KnightPath kp( std::make_pair(0,0), std::make_pair(x,y) );
		AStar<KnightPath> astar( kp );

		start = clock();
		astar.solve();
		end = clock();

		int msecs = ((double)(end-start))*1000 / CLOCKS_PER_SEC;
		std::cout << astar.expandedNodes << "\t\t" << msecs << "\t" << astar.solutionDepth << std::endl;
	}

	return 0;
}
