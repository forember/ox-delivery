#include "FredericksonKCPP.h"

#include <boost/graph/graph_utility.hpp>

#include <iostream>

#define DEBUG_FREDERICKSONI
#define DBG1

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

#ifdef DEBUG_FREDERICKSON
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
#ifdef DEBUG_FREDERICKSON
    std::cout << "SOLVE STARTED============================!\n";
#endif //!DEBUG_FREDERICKSON
	if(m_k ==1) {
		m_eulerTours.push_back(m_optimalPath);
		return;
	}
	kcpp::Vertex v_last = m_sourceVertex; //is v_{l'(j)}
	kcpp::Vertex v_first = m_sourceVertex;
#ifdef DEBUG_FREDERICKSON
    std::cout << "The first and last vertices are: " << v_first << " " << v_last <<"\n";
	EulerTour::iterator end= m_optimalPath.end();
    end--;
    std::cout << "The last edge in the path is: " << m_graph.getEProp(*end) << "\n";
#endif //!DEBUG_FREDERICKSON

	EulerTour::iterator ei= m_optimalPath.begin();
    EulerTour::iterator e_last= m_optimalPath.begin();

    double curr_rout_cost = 0.0; // same as c(R_{v_{l'(j)}}) (in latex)

    int rout_num = 0;
    std::list<EulerTour::iterator> tour_last_edges; // to keep the last edges
    std::list<kcpp::Vertex> tour_last_verteces; // to keep last edges
    ReebEdge edgei = m_graph.getEProp(*ei);
    double next_edge_cost = edgei.area;
    for(int j = 1; j<=m_k; j++) {
        //double L_j = m_optimalCost*double(j)/double(m_k);
        double L_j = (m_optimalCost-2.0*m_smax) * double(j)/double(m_k) + m_smax; 
#ifdef DEBUG_FREDERICKSON
        std::cout << "for robot j: " <<j << " L_j is: " << L_j <<"\n";
#endif //!DEBUG_FREDERICKSON
        if(ei == m_optimalPath.end()) {
            break;
        }

#ifdef DBG
        std::cout << "for robot j: " <<j << " L_j is: " << L_j <<"\n";
#endif
        /*->--------2. finding the last verteces-----------*/
        while(curr_rout_cost + next_edge_cost <= L_j && ei != m_optimalPath.end()) {
            Vertex v_1, v_2;
            tie(v_1, v_2) = m_graph.getEndNodes(*ei);
            kcpp::Vertex v1 = getVertex(v_1);
            kcpp::Vertex v2 = getVertex(v_2);
            v_last = (v1 == v_first ? v2 : v1);

            ReebEdge edge = m_graph.getEProp(*ei);
            curr_rout_cost += edge.area;
            e_last = ei;

            ei++;
            edge = m_graph.getEProp(*ei); //FIXME: this is potential cause for seg fault, must be fixed
            next_edge_cost = edge.area;
            v_first = v_last;
#ifdef DBG
            std::cout  << "checking the last edges: " << m_graph.getEProp(*e_last) << "\n with cost untill here ";
            std::cout  << curr_rout_cost <<  " and the L_J is " << L_j << "\n";
            std::cout  << "checking the next edge: " << m_graph.getEProp(*ei) << "\n\n"; 
#endif
        }
        rout_num++;
#ifdef DEBUG_FREDERICKSON_1
        std::cout  << "checking the last edge: " << m_graph.getEProp(*e_last) << "\n";
        std::cout << "current last vertex is: " << v_last <<"\n";
        if(ei==m_optimalPath.end()){
            std::cout << "*****&&&&&&&&&&&&& j is: " <<  j << "\n";
        }
#endif //!DEBUG_FREDERICKSON
        /*---------2. finding the last verteces-----------<-*/
        EulerTour::iterator last_edge = e_last;
        kcpp::Vertex last_vertex = v_last;
        if(ei!=m_optimalPath.end()){
            //push the last one and do appropriate operations to end this thing

            //to take v_l'(j)+1 vertex
            Vertex v_1, v_2;
            tie(v_1, v_2) = m_graph.getEndNodes(*ei);// ei is one ahead edge that is not in the rout
            kcpp::Vertex v_last_j = (getVertex(v_1) == v_last ? getVertex(v_2) : getVertex(v_1)); //v_{l'(j)+1}
        ReebEdge edge = m_graph.getEProp(*ei);
        double r_j = L_j - curr_rout_cost;
        double sum_1 = r_j + m_shortTravelDistances.at(v_last);
        double sum_2 = edge.area  - r_j + m_shortTravelDistances.at(v_last_j);
        if( sum_1 > sum_2) {
            last_edge = ei;
            last_vertex = v_last_j;
        }
        }
        tour_last_verteces.push_back(last_vertex);
        tour_last_edges.push_back(last_edge);
#ifdef DBG
        std::cout  << "checking the last edge for j : " << j <<" -> "<< m_graph.getEProp(*last_edge) << "\n";
#endif //!DEBUG_FREDERICKSON

    }
    //ensuring that all cells are assigned to robot
    EulerTour::iterator itl = m_optimalPath.end();
    itl--;
    if(itl!=m_optimalPath.end()) {
        if(tour_last_edges.size()>=m_k) {
            tour_last_verteces.pop_back();
            tour_last_edges.pop_back();
        }
            tour_last_edges.push_back(itl);
            tour_last_verteces.push_back(m_sourceVertex);
    }

    // 4. building up the k-ours from tour_last_edges
    if(tour_last_edges.size() == 1 ) {
        m_eulerTours.push_back(m_optimalPath);
        return;
    }
    EulerTour::iterator it, ittmp;
    EulerTour::iterator it_prev = m_optimalPath.begin();
    kcpp::Vertex v_last2 = m_sourceVertex;
    kcpp::Vertex v_first2 = m_sourceVertex;
    double maxCostTmp = 0.0; // for storing the max length
    //for(int j = 0; j < tour_last_edges.size(); j++) {
    int ik =1;
    while(!tour_last_edges.empty()) {
		std::list<Edge> tour_j;
        it = tour_last_edges.front();
        tour_last_edges.pop_front();
        ittmp = it;
            std::cout << "For robot j " << ik << std::endl;
            ik++;
            ittmp++;
        for(EulerTour::iterator curr_it = it_prev; curr_it!=(ittmp); ++curr_it) {
			tour_j.push_back(*curr_it);
#ifdef DBG
            std::cout  << "checking the tour construction: " << m_graph.getEProp(*curr_it)  << '\n';
#endif
        }
        it_prev = ittmp;
        std::cout << "\n\n";

		if(!tour_j.empty()) {

			m_eulerTours.push_back(tour_j);
		}
        // calculating the max cost
        v_last2 = tour_last_verteces.front();
        tour_last_verteces.pop_front();
		double tmp = pathCost(tour_j) + m_shortTravelDistances.at(v_last2) + m_shortTravelDistances.at(v_first2);
#ifdef DBG1
        std::cout << "starting vertex " << v_first2 << "\nending vertex " << v_last2 << "\n";
        std::cout << "the cost of the tour is: " << pathCost(tour_j);
        std::cout << "\n the cost of the shortdist "   << m_shortTravelDistances.at(v_last2) << "\n";
        std::cout << "\n the cost of the shortdist "   << m_shortTravelDistances.at(v_first2) << "\n";
#endif
        v_first2 = v_last2;
		if(tmp> maxCostTmp) {
			maxCostTmp = tmp;
		}

    }
	if(m_k!=1) {
	    m_maxCoverageCost = maxCostTmp;
        std::cout << "TETTTTTTTTTTTTTTTTTTTTT " << m_maxCoverageCost << "\n\n"; 
	}

#ifdef DEBUG_FREDERICKSON
    std::cout << "the size of the tours are: " << tour_last_edges.size();
    std::cout << "SOLVE FINISHED============================!\n";

#endif //!DEBUG_FREDERICKSON
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

		double S_V1_Vij = m_shortTravelDistances.at(v1);
		double S_Vij_1_V1 = m_shortTravelDistances.at(v2);
		//double S_V1_Vij = m_shortCoverDistances.at(v1);
		//double S_Vij_1_V1 = m_shortCoverDistances.at(v2);
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
