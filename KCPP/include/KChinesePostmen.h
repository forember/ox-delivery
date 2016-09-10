#ifndef KCHINESEPOSTMENPROBLEM_H
#define KCHINESEPOSTMENPROBLEM_H

#include <map>
#include <vector>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
//#include "ChinesePostman.h"

#include <ChinesePostman/ChinesePostman.h>
#include <ChinesePostman/DrawImage.h>
#include <Boustrophedon/BCD.h>
#include <Boustrophedon/DrawImage.h>
#include <Leo/Point2D.h>
#include <Leo/ReebGraph.h>
#include <Leo/RegionData.h>
#include <WayPoints/WayPoints.h>
#include <WayPoints/DrawImage.h>

/**==============================================================
 * \file KChinesePostmen.h
 *
 * An Abstract class for all K Chinese Postman algorithms
 * The pure virtual function solve() must be implemented 
 * for each algorithm
 *
 * \author    Nare Karapetyan
 *
 **==============================================================*/

namespace kcpp {
	typedef boost::adjacency_list< boost::vecS, boost::vecS,
					boost::undirectedS,
					ReebVertex,
					ReebEdge> Graph;

	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<Graph>::edge_descriptor Edge;

	typedef boost::graph_traits<Graph>::vertex_iterator Vertex_Iter;
	typedef boost::graph_traits<Graph>::edge_iterator Edge_Iter;

	typedef std::list<ReebEdge> EulerTour;
}

/*Edge is edge_descriptor defined in ReebGraph.h (libCPP library)
 * The output tour type of the CPP algorithm
 * */
typedef std::list<Edge> EulerTour;

class KChinesePostmen
{
	public:
		//FIXME: const was deleted, see more in scr/KChinesePostman.cpp 
		KChinesePostmen();
		KChinesePostmen(ReebGraph graph, int k = 1);
    virtual void solve(bool mod = false) = 0;

		/** Function that returns the created routes */
		std::vector<EulerTour> getKEulerianTours(){
			return m_eulerTours;
		};

		virtual void printEulerianTours();

		/** Function for building a shortest path for target vertex */
		std::list<ReebEdge> getShortPath(Vertex v, kcpp::Graph g);

		/** Builds graph that contains no parallel edges from the original ReebGraph */
		kcpp::Graph getSimpleGraph(/*ReebGraph g*/);

		// UndirectedGraph* m_coverageGraph; // vi = i, edge_weight = c(i, j)
		ReebGraph m_graph;
		kcpp::Graph m_g;

		//number of robots
		int m_k;

	protected:
		/** Computes shortest Distances based on the non multigraph */
		void computeShortestDistances(kcpp::Graph graph);

		/** Return the cost of a given tour*/
		double pathCost(EulerTour tour);


		/** helper functions for debugging */
		void testGraph(kcpp::Graph graph);
		void printShortestPaths(kcpp::Graph graph); // DEBUG: only for testing

		//FIXME: const is removed 
		kcpp::Vertex getVertex(Vertex vdesc) {
			//FIXME: add assert at this point to be sure 
			// that the element exists or use c++ 11's .at()
			unsigned int  id = m_graph.getVProp(vdesc).Vid;
			//      std::cout << "V --- >>> " << id << std::endl;
			return m_vertices_.at(id);
		}

	protected:
		/*counted by the coverage weight, i.g. by cell area*/
		std::vector<kcpp::Vertex> m_coverPredecessors;
		std::vector<double> m_shortCoverDistances; // m_shortDistance[i] = s(0, i);
		std::vector<kcpp::Edge> m_coverEdgePredecessor; // represents v and his predecessor's edge that is included in the shortest path

		/*predecessors computed accounting the travel weight, i.g. by euclidean distance*/
		std::vector<kcpp::Vertex> m_travelPredecessors;
		std::vector<double> m_shortTravelDistances; // s(v1, i) = m_..[i]
		std::vector<kcpp::Edge> m_travelEdgePredecessor;


		kcpp::Vertex m_sourceVertex; // v1

		//Output: storage for all founded Euler tours
		std::vector<EulerTour> m_eulerTours;
		std::vector<double> m_tourCosts;

	private:
		/** internal container to get the kcpp vertex descriptor by the Vertex desc of CPP
		 * this is the way to get from Vertex(of the type listS) the kcpp::Vertex(of the type vecS) */
		//    std::map<Vertex, kcpp::Vertex>  m_vertices_; //BUG: the descriptors are damaged after getting out from the scope
		std::vector<kcpp::Vertex>  m_vertices_; //BUG: the descriptors are damaged after getting out from the scope
};

#endif // KCHINESEPOSTMENPROBLEM_H
