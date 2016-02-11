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
//		std::cout << n.mData.first << " " << n.mData.second << std::endl;
		if( n.mData == mGoal ) return true;
		return false;
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
			return a.mVerts == b.mVerts;
		}

		friend bool operator!=( const Edge& a, const Edge& b )
		{
			return !( a == b );
		}

		friend bool operator<( const Edge& a, const Edge& b )
		{
			return a.mCost < b.mCost;
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
			for( size_t i=0; i<a.mData.size(); ++i )
			{
				if( a.mData[i] != b.mData[i] ) return false;
			}
			return true;
		}

		NodeData mData;
		Node* mPrev;
		float mF;
	};

	TSP( const unsigned int nVertices ) : adjacencyVec( nVertices, std::vector<Edge>() )
	{
		for( unsigned int i=0; i < nVertices; ++i )
		{
			vertices.push_back( std::make_pair( 
						(rand() % 1001) / 1000.0f, (rand() % 1001) / 1000.0f ));
		}

		for( unsigned int i=0; i<nVertices-1; ++i )
		{
			for( int j=i+1; j<nVertices; ++j )
			{
				float dist = sqrt( pow(vertices[i].first - vertices[j].first, 2) + 
					pow(vertices[i].second - vertices[j].second, 2) );
				adjacencyVec[i].push_back( Edge( std::make_pair(i,j), dist ));
				adjacencyVec[j].push_back( Edge( std::make_pair(j,i), dist ));
			}
		}

		for( size_t i=0; i<adjacencyVec.size(); ++i )
		{
			for( size_t j=0; j<adjacencyVec[i].size(); ++j )
			{
				sortedEdges.push_back(adjacencyVec[i][j]);
			}
		}
		std::sort( sortedEdges.begin(), sortedEdges.end() );

	}

	std::vector<Node> getSuccessors( Node& n )
	{
		std::vector<Node> succ;

		if( n.mData.empty() )
		{
			for( int i=0; i<adjacencyVec[0].size(); ++i )
			{
				NodeData newSol;
				newSol.push_back(adjacencyVec[0][i]);
				succ.push_back(Node(newSol, NULL, newSol[0].mCost));
			}
			return succ;
		}

		if( n.mData.back().mVerts.second == n.mData.front().mVerts.first )
		{
			return succ;
		}

		int lastV = n.mData.back().mVerts.second;
		for( int i=0; i<adjacencyVec[lastV].size(); ++i )
		{
			if( !checkVertexInSolution( n.mData, adjacencyVec[lastV][i].mVerts.second ) )
			{
				std::vector<Edge> newSol = n.mData;
				newSol.push_back( adjacencyVec[lastV][i] );
				succ.push_back( Node( newSol, &n, n.mF + adjacencyVec[lastV][i].mCost ));
			}
			else if( n.mData.size() == vertices.size()-1 &&
					adjacencyVec[lastV][i].mVerts.second == n.mData.front().mVerts.first )
			{
				std::vector<Edge> newSol = n.mData;
				newSol.push_back( adjacencyVec[lastV][i] );
				succ.push_back( Node( newSol, &n, n.mF + adjacencyVec[lastV][i].mCost ));
			}
		}
		return succ;
	}

	bool checkVertexInSolution( const std::vector<Edge>& s, int v )
	{
		for( size_t i=0; i<s.size(); ++i )
		{
			if( s[i].mVerts.first == v || s[i].mVerts.second == v )
				return true;
		}
		return false;
	}

	bool checkSolution( const Node& n )
	{
		if( n.mData.size() == vertices.size() && 
				n.mData.back().mVerts.second == n.mData.front().mVerts.first )
			return true;
		return false;
	}

	float heuristic( const NodeData& n )
	{
		NodeData mst = n;
		size_t orgSize = mst.size();
		for( size_t i=0; i<sortedEdges.size(); ++i )
		{
			if( !checkVertexInSolution(mst, sortedEdges[i].mVerts.first) ||
					!checkVertexInSolution( mst, sortedEdges[i].mVerts.second))
			{
				mst.push_back( sortedEdges[i] );
			}
			if( mst.size() == vertices.size()-1 ) break;
		}
		float total_cost = 0.0f;
		for( size_t i=orgSize; i<mst.size(); ++i )
		{
			total_cost += sortedEdges[i].mCost;
		}
		return total_cost;
	}

	std::vector<Edge> sortedEdges;
	std::vector<std::vector<Edge> > adjacencyVec;
	std::vector< std::pair<float,float> > vertices;
	NodeData mStart;
	NodeData mGoal;
};

int main( int argc, char* argv[] )
{
	clock_t start, end;
	std::cout << "Nodes Expanded\tTime(ms)\tDepth" << std::endl;
	for( int i=3; i<15; ++i )
	{
//		int x = rand() % 10;
//		int y = rand() % 10;
//
		KnightPath kp( std::make_pair(0,0), std::make_pair(10,10) );
//		TSP tsp( i );
		AStar<KnightPath> astar( kp );
//
		start = clock();
		astar.solve();
		end = clock();
//
		int msecs = ((double)(end-start))*1000 / CLOCKS_PER_SEC;
		std::cout << astar.expandedNodes << "\t\t" << msecs << "\t" << astar.solutionDepth << std::endl;
	}

	return 0;
}
