#ifndef TSP_HPP
#define TSP_HPP

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


#endif
