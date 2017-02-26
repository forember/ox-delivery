#include "NRC.h" // Naive rout clustering

#include <boost/graph/graph_utility.hpp>

#include <iostream>

#define DEBUG_NRC
#define DBG1

/**==============================================================
 * Partitioning the input optimal eulerian tour into equal k parts
 * \author Nare Karapetyan
 **==============================================================*/

	NRC::NRC(const EulerTour& tour, const ReebGraph& graph, int k)
: KChinesePostmen(), m_optimalPath(tour)
{
	m_k = k;
	m_graph = graph;
	m_g = getSimpleGraph(/*graph*/);

#ifdef DEBUG_NRC
	std::cout << "-> Testing the graph mapping -> ...\n";
	Vertex_Iter vi, vi_end;
	for (tie(vi, vi_end) = m_graph.getVertices(); vi != vi_end; vi++) {
		// BUG: mapping fails, after getting out from scope the descriptor is set to 0
		std::cout << m_g[getVertex(*vi)].Vid << " %^  "; 
		std::cout << (m_graph.getVProp(*vi)).Vid << "\n";
	}
	std::cout << "... -> end <-\n";
#endif    

	//cerr << "computer shortest distance" << "\n";
	computeShortestDistances(m_g);
	m_optimalCost = pathCost(m_optimalPath);
	m_smax = computeSMax(m_optimalPath);
	m_maxCoverageCost = m_optimalCost;

#ifdef DEBUG_NRC
	//    boost::print_graph(m_g, boost::get(&ReebEdge::Eid,m_g));
	//  printShortestPaths();
	std::cout << "-> Printing out the single optimal path from CPP...\n";
	EulerTour::iterator ei= m_optimalPath.begin();
    kcpp::Vertex v_first, v_second;
	v_first = getVertex(m_graph.getFirstVertex());
	for( ; ei!=m_optimalPath.end(); ++ei) {
		ReebEdge eprop = m_graph.getEProp(*ei);
		Vertex v_1, v_2;
		tie(v_1, v_2) = m_graph.getEndNodes(*ei); // <---- IMPORTANT
		kcpp::Vertex v1 = getVertex(v_1);
		kcpp::Vertex v2 = getVertex(v_2);
		v_second = (v1 == v_first ? v2 : v1);


		std::cout << eprop <<  " , attached to vertices: " << m_graph.getVProp(v_first).Vid
			<< " & " << m_graph.getVProp(v_second).Vid  << " & ...\nShort dist: " << m_shortTravelDistances.at(v_first)  << ", " << m_shortTravelDistances.at(v_second) << ", is null? " << ((*ei) == ReebGraph::nullEdge()) << endl;
		v_first = v_second;
	}
	std::cerr << "Optimal path cost: " << pathCost(m_optimalPath) << std::endl;
	std::cerr << "s_max cost : " << computeSMax(m_optimalPath) << std::endl ;
	std::cout << "... -> end <-\n";
#endif
#ifdef DEBUG_NRC
	std::cout << "-----------------NRC Ctor completed------------------\n";
#endif
}

/**==============================================================
 * \param none
 * \return none
 **==============================================================*/
void NRC::solve(bool mod)
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
	kcpp::Vertex v_first = m_sourceVertex;
	Edge e_last;

	EulerTour::iterator ei= m_optimalPath.begin();

	int k = m_k; // FIXME: added
	double maxCostTmp = 0;
	for(int j =1; j<=m_k; ++j) {
		std::list<Edge> tour_j;
		std::list<ReebEdge> tour_jv;
		double L_j;
		if(mod) {
			L_j = (m_optimalCost)/double(k) + m_smax; 
			k--;//FIXME:
		} else {
			L_j = m_optimalCost*double(j)/double(m_k) + m_smax;//(m_optimalCost-2.0*m_smax) * double(j)/double(m_k) + m_smax; 
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
				//  ++ei;
				c_subR = 0; // FIXME: was at the beginning of statement
				break;
			}

			//c_subR_last = c_subR;
			c_R_last = c_R;

			++ei;
		}
		v_first = v_last;

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
		double tmp = pathCost(tour_j) + m_shortTravelDistances.at(v_last) + m_shortTravelDistances.at(v_first);
		if(tmp> maxCostTmp) {
			maxCostTmp = tmp;
		}

	}
	if(m_k!=1){
		m_maxCoverageCost = maxCostTmp;
	}
	if(mod){
		m_optimalCost-=sub_S;
		sub_S=0.0;
	}
}

/**==============================================================
 * Computing S_max value:
 * the cost of the fathest node in the Eulerian tour
 *
 * \param Eulerian tour
 * \return double cost
 **==============================================================*/
double NRC::computeSMax(EulerTour tour)
{
	double smax = 0.0;

	for (EulerTour::iterator ei=tour.begin(); ei != tour.end(); ++ei) {
		Vertex v_first, v_second;
		tie(v_first, v_second) = m_graph.getEndNodes(*ei);

		kcpp::Vertex v1 = getVertex(v_first);
		kcpp::Vertex v2 = getVertex(v_second);

		ReebEdge edge = m_graph.getEProp(*ei);

		double S_V1_Vij = m_shortTravelDistances.at(v1);
		double S_Vij_1_V1 = m_shortTravelDistances.at(v2);
		//double S_V1_Vij = m_shortCoverDistances.at(v1);
		//double S_Vij_1_V1 = m_shortCoverDistances.at(v2);
		smax = std::max(S_V1_Vij+S_Vij_1_V1+edge.area, smax);
	}
	smax = smax/2.0;
	return smax;
}

