#include "FredericksonKCPP.h"

#include <boost/graph/graph_utility.hpp>

#include <iostream>

#define DEBUG_FREDERICKSON

/**==============================================================
 * K chinese postman algoritm by Frederisckon et al.
 * \ref Approximation algorithms for some routing problems
 * 
 * Input: Undirected weighted graph, number of routs k, the optimal one route 
 * optained by Chinese postman algorithm
 *
 * Output: k routs in the graph
 *
 * Notations:
 * 1. R- is the optimal acquired by CPP algorithm
 * 2. V_i is the i-th vertex, v_1 is the starting vertex
 * 3. S(v, u) is the shortest path between v and u vertices 
 * 4. c(v, u) is the cost of (v,u) edge 
 * 5. Smax = max{s(V_1, V_ij) + c(V_ij, V_ij_1) + s(V_ji_1, V1)}* 1/2, for all j, 2<=j<=m-1
 * 6. L_j
 *
 * Algorithm:
 * 1. Find Optimal one R route by solving Chinese Postman Problem (*)
 * 2. For each j, 1<=j<=k find the last vertex in optimal tour, 
 * such that cost of that tour is less than L_j
 * 3. Identify which vertex to take on the last edge
 * 4. Compute the shortest path from the source vetex to the j-th route
 * and combine them to get the complete tour
 *
 *
 * \author Nare Karapetyan
 **==============================================================*/

FredericksonKCPP::FredericksonKCPP(const EulerTour& tour, const ReebGraph& graph, int k)
	: KChinesePostmen(), m_optimalPath(tour)
{
#ifdef DEBUG_FREDERICKSON
	std::cout << "-----------------FredericksonKCPP Ctor started------------------\n";
#endif
	m_k = k;
	m_graph = graph;
	m_g = getSimpleGraph(/*graph*/);

#ifdef DEBUG_FREDERICKSON
	Vertex_Iter vi, vi_end;
	for (tie(vi, vi_end) = m_graph.getVertices(); vi != vi_end; vi++) {
		// BUG: mapping fails, after getting out from scope the descriptor is set to 0
		std::cout << m_g[getVertex(*vi)].Vid << " %^  "; 
		std::cout << (m_graph.getVProp(*vi)).Vid << "\n";
	}
#endif    

	//cerr << "computer shortest distance" << "\n";
	computeShortestDistances(m_g);
	m_optimalCost = pathCost(m_optimalPath);
	m_smax = computeSMax(m_optimalPath);

#ifdef DEBUG_FREDERICKSON
	//    boost::print_graph(m_g, boost::get(&ReebEdge::Eid,m_g));
	//  printShortestPaths();
	std::cerr << "Optimal path cost: " << pathCost(m_optimalPath) << std::endl;
	std::cerr << "s_max cost : " << computeSMax(m_optimalPath) << std::endl ;
#endif
#ifdef DEBUG_FREDERICKSON
	std::cout << "-----------------FredericksonKCPP Ctor completed------------------\n";
#endif
}

/**==============================================================
 * The solver for k Chinese postman problem by Frederickson algorithm
 * no return value, the results are stored in the class member variables
 * m_eulerTours, without the shortest paths
 *
 * \param none
 * \return none
 **==============================================================*/
void FredericksonKCPP::solve(bool mod)
{
	if(m_k ==1) {
		m_eulerTours.push_back(m_optimalPath);
		return;
	}
	double c_subR_last = 0.0, c_subR = 0.0;
	c_subR_last++;
	double c_R_last = 0.0, c_R = 0.0;
	double sub_S = 0.0;
	kcpp::Vertex v_last = m_sourceVertex;
	Edge e_last;

	EulerTour::iterator ei= m_optimalPath.begin();

	int k = m_k; // FIXME: added
	for(int j =1; j<=m_k; ++j) {
		std::list<Edge> tour_j;
		std::list<ReebEdge> tour_jv;
		double L_j;
		if(mod) {
			L_j = (m_optimalCost)/double(k) + m_smax; //FIXME: was original
			k--;//FIXME:
		} else {
			L_j = (m_optimalCost-2.0*m_smax) * double(j)/double(m_k) + m_smax; //FIXME: was original
		}

		//double C_R_Vl_j = m_shortTravelDistances.at(v_last);

		/***********************************************
		 * Finding the last vertex on the optimal tour 
		 *  such that the cost of the tour is <= L_j   */
		while(ei!=m_optimalPath.end()) {
			Vertex v_first, v_second;
			tie(v_first, v_second) = m_graph.getEndNodes(*ei);

			kcpp::Vertex v1 = getVertex(v_first);
			kcpp::Vertex v2 = getVertex(v_second);

			ReebEdge edge = m_graph.getEProp(*ei);

			c_subR += edge.area; 
			c_R = c_subR + m_shortTravelDistances.at(v_last) + m_shortTravelDistances.at(v2); 
			tour_jv.push_back(edge);
			tour_j.push_back(*ei);

			if(c_R>L_j) {
				//--ei;
				c_subR = 0;
				c_R = 0;

				// Determine which vertex of the edge to take: first or second one
				double r_j = L_j - c_R_last;
				double sum_1 = r_j + m_shortTravelDistances.at(v1);
				double sum_2 = edge.area  - r_j + m_shortTravelDistances.at(v2);
				if(sum_1 <= sum_2) {
					--ei;
					tour_j.pop_back();
					tour_jv.pop_back();
					sub_S = c_subR; // FIXME: added
					sub_S -= edge.area; //FIXME
				}
				++ei;
				c_subR = 0; // FIXME: was at the beginning of statement
				break;
			}

			//c_subR_last = c_subR;
			c_R_last = c_R;

			++ei;
		}

		if(ei!=m_optimalPath.end()) {

			Vertex v_first, v_second;

			tie(v_first, v_second) = m_graph.getEndNodes(*ei);

			//std::cout << "edge -> " << m_graph.getEProp(*ei).Eid;
			v_last = getVertex(v_second);
		}
		/***********************************************/

		if(!tour_j.empty()) {

			m_eulerTours.push_back(tour_j);
		}
		if(mod){
			m_optimalCost-=sub_S;
			sub_S=0.0;
		}
	}
}

/**==============================================================
 * Computing S_max value:
 * the cost of the fathest node in the Eulerian tour
 *
 * \param Eulerian tour
 * \return double cost
 **==============================================================*/
double FredericksonKCPP::computeSMax(EulerTour tour)
{
	double smax = 0.0;

	for (EulerTour::iterator ei=tour.begin(); ei != tour.end(); ++ei) {
		Vertex v_first, v_second;
		tie(v_first, v_second) = m_graph.getEndNodes(*ei);

		kcpp::Vertex v1 = getVertex(v_first);
		kcpp::Vertex v2 = getVertex(v_second);

		ReebEdge edge = m_graph.getEProp(*ei);

		//FIXME: put this back double S_V1_Vij = m_shortTravelDistances.at(v1);
		//FIXME: put this back double S_Vij_1_V1 = m_shortTravelDistances.at(v2);
		double S_V1_Vij = m_shortCoverDistances.at(v1);
		double S_Vij_1_V1 = m_shortCoverDistances.at(v2);
		smax = std::max(S_V1_Vij+S_Vij_1_V1+edge.area, smax);
	}
	smax = smax/2.0;
	return smax;
}

/*
   std::vector<double> FredericksonKCPP::solve(int k)
   {
   m_results.clear();
   m_k = k;
   countSMax();
   m_lastVertices.clear();

   int lastPathIndex = 0;
   int currIndex=0;
   kcpp::Vertex lastVertex = m_sourceVertex;
   for(int j =1; j<=m_k; ++j) {
   double L_j = (m_optimalCost-2*m_smax) * double(j)/double(m_k) + m_smax;


   double C_R_Vl_j = m_shortTravelDistance->at(lastVertex);

//FIXME remember that we have to go back to previous step
// when condition holds true
double cRV = 0;
double crvlj = cRV;
while( (L_j >= (C_R_Vl_j +cRV)) && size_t(currIndex)<m_optimalPath.size()-1) {
lastPathIndex = currIndex;
crvlj = cRV;
cRV += m_optimalPath.at(currIndex).area;
std::cout << lastPathIndex << std::endl;
currIndex++;
//int o;
//std::cin >> o;
}
currIndex =lastPathIndex;
lastVertex = m_optimalPath.at(lastPathIndex).startNode;
double r_j = L_j - (C_R_Vl_j + crvlj - m_optimalPath.at(lastPathIndex).weight);
// FIXME should be changed to m_shorTravelDistance
//if(2*r_j + m_shortCoverDistance->at(lastVertex)
if(2*r_j + m_shortTravelDistance->at(lastVertex)
<= m_optimalPath.at(lastPathIndex).weight
+ m_shortTravelDistance->at(m_optimalPath.at(lastPathIndex).endNode)) {
//+ m_shortCoverDistance->at(m_optimalPath.at(lastPathIndex).endNode)) {
lastVertex = lastVertex;
} else {
lastVertex = m_optimalPath.at(lastPathIndex).endNode;
if(lastPathIndex<m_optimalPath.size()-1) {
currIndex++;
} else if(currIndex>=m_optimalPath.size()-1) {
std::cout << currIndex << " blbla ";
break;
}
}
m_lastVertices.push_back(lastVertex);
m_results.push_back(C_R_Vl_j+cRV);
}
return m_results;
}*/

/*
   void FredericksonKCPP::printKTours()
   {
   std::cout << "distances " << std::endl;
   boost::graph_traits < UndirectedGraph >::vertex_iterator vi, vend;
   for (boost::tie(vi, vend) = boost::vertices(*m_coverageGraph); vi != vend; ++vi) {
   std::cout << "distance(" << *vi << ") = " << m_shortCoverDistance->at(*vi) << ", " << std::endl;
   }

   std::cout << std::endl;

   for(int j =0; j < m_lastVertices.size(); ++j) {
   std::cout << "--> " << m_lastVertices.at(j);
   }
   std::cout << std::endl;
   }*/
