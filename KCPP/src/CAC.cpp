#include "CAC.h"

#include <boost/graph/graph_utility.hpp>

#include <iostream>
#include <algorithm>
#include <queue>

#define DEBUG_CAC

/* helper predicate for map with ReebEdge */
struct edgeReebCompare
{
    bool operator() (const ReebEdge& e1, const ReebEdge& e2) const
    {
        return e1.Eid < e2.Eid;
    }
};
struct vertexReebCompare
{
    bool operator() (const ReebVertex& v1, const ReebVertex& v2) const
    {
        return v1.Vid < v2.Vid;
    }
};

/* helper predicates for list sort algorithm  and min_element*/
/* Comparison by X axis*/
bool compareByXCoords (const ReebEdge& e1, const ReebEdge& e2) { 
    unsigned int numPoints1, numPoints2;
    double midX1 = 0.0;
    double midX2 = 0.0;
    numPoints1 = e1.topBoundary.size();
    numPoints2 = e2.topBoundary.size();
    if (numPoints1 == 0 || numPoints2 == 0)
    {
        return false;
    }
    else if (numPoints1 == 1 && numPoints2 == 1) 
    {
        midX1 = e1.topBoundary[0].xcoord();
        midX2 = e2.topBoundary[0].xcoord();
    }
    else
    {
        midX1 = e1.topBoundary[numPoints1/2-1].xcoord();
        midX2 = e2.topBoundary[numPoints2/2-1].xcoord();
    }
    return midX1 < midX2;
}

/* Comparison by Y axis*/
bool compareByYCoords (const ReebEdge& e1, const ReebEdge& e2) { 
    unsigned int numPoints1, numPoints2;
    double midY1 = 0.0;
    double midY2 = 0.0;
    numPoints1 = e1.topBoundary.size();
    numPoints2 = e2.topBoundary.size();
    if (numPoints1 == 0 || numPoints2 == 0)
    {
        return false;
    }
    else if (numPoints1 == 1 && numPoints2 == 1) 
    {
        midY1 = (e1.topBoundary[0].ycoord()
                + e1.bottomBoundary[0].ycoord()) / 2;
        midY2 = (e2.topBoundary[0].ycoord()
                + e2.bottomBoundary[0].ycoord()) / 2;
    }
    else
    {
        midY1 = (e1.topBoundary[numPoints1/2-1].ycoord()
                + e1.bottomBoundary[numPoints1/2-1].ycoord()) / 2;
        midY2 = (e2.topBoundary[numPoints2/2-1].ycoord()
                + e2.bottomBoundary[numPoints2/2-1].ycoord()) / 2;
    }
    return midY1 < midY2;
}

/**==============================================================
 * \ref Coverage with Area Clustering algorithm
 * 
 * Input: Undirected weighted graph, number of routs k, the optimal one route 
 * optained by Chinese postman algorithm
 *
 * Output: k routs in the graph
 *
 * Notations:
 *
 * Algorithm:
 * 1. Sort cells by x and y axis
 *
 * \author Nare Karapetyan
 **==============================================================*/

CAC::CAC(const EulerTour& tour, RegionData data, ReebGraph graph, int k) 
	: KChinesePostmen(), m_optimalPath(tour)
{

    m_k = k;
    m_graph = graph;
    m_data = data;
    m_g = getSimpleGraph(/*graph*/);
    computeShortestDistances(m_g);
    Edge_Iter ei, ei_end;
    Vertex v_first, v_second;
    ReebEdge eprop;
    Edge currEdge;
    Out_Edge_Iter oi, oi_end;
    m_optimalCost = pathCost(m_optimalPath);


#ifdef DEBUG_CAC
/*    tie(v_first, v_second) = graph.getEndNodes(*ei);
    std::cout << eprop << ", attached to vertices: " << graph.getVProp(v_first).Vid
        << " & " << graph.getVProp(v_second).Vid << ", is null? "
        << (currEdge == ReebGraph::nullEdge()) << endl;*/
#endif
}

/**==============================================================
 * The solver for k Chinese postman problem with CAC algorithm
 * no return value, the results are stored in the class member variables
 * m_eulerTours, without the shortest paths
 *
 * \param none
 * \return none
 **==============================================================*/
void CAC::solve(bool t)
{
    //perform BFS over graph but taking into account neighborhood distances
    std::queue<Edge> q_h; //helper queue (must be replaced)
    std::queue<ReebEdge> q;

    Vertex  currVertex = m_graph.getFirstVertex();

    double coverArea = m_optimalCost;

    /**************************************************************************/
    /** Initializing to false flags that indicate if vertex or edge was processed */
    std::map<ReebEdge, bool, edgeReebCompare> visitedEdges; // if edge is already in a cluster it is true
    std::map<ReebVertex, bool, vertexReebCompare> visitedVertices; // for bfs by neighborhood
    Edge_Iter ei, ei_end;
    for (tie(ei, ei_end) = m_graph.getEdges(); ei != ei_end; ei++) {
        ReebEdge e = m_graph.getEProp(*ei);
        visitedEdges.insert( std::pair<ReebEdge,bool> (e, false));
    }
    Vertex_Iter vi, vi_end;
    for (tie(vi, vi_end) = m_graph.getVertices(); vi != vi_end; vi++) {
        ReebVertex v = m_graph.getVProp(*vi);
        visitedVertices.insert( std::pair<ReebVertex,bool> (v, false));
    }
    /**************************************************************************/

    int k = 0;
        int ij = 1;

    double smax = computeSMax(m_optimalPath);
    ReebEdge firstEdge = m_graph.getEProp(m_optimalPath.front()); //FIXME: some assertions must be added
    q.push(firstEdge);
    for(int j = 1; j <=m_k; ++j) {

        std::list<ReebEdge> cluster_j;
        double currClusterLimit = (coverArea) * 1.0/double(m_k - k) + smax; // m_smax must be defined
        double currClusterSize = 0;

        if(j==m_k) {
            for (tie(ei, ei_end) = m_graph.getEdges(); ei != ei_end; ei++) 
            {
                ReebEdge e = m_graph.getEProp(*ei);
                if(!visitedEdges[e]) {
                    cluster_j.push_back(e);
                }
            }
        } else {
        //BFS
        while(!q.empty()) {

            std::list<ReebEdge> adjEdges = getAdjacentList(currVertex); //FIXME: must be checked if there are adjacent edges at all
            std::list<ReebEdge>::iterator it;

            ReebEdge currEdge = q.front();
            Vertex v1, v2;
            tie(v1,v2) = m_graph.getEndNodes(m_graph.getEdge(currEdge.Eid));
            Vertex nextVertex = (v1 == currVertex ? v2 : v1);

            if(!visitedVertices[m_graph.getVProp(currVertex)]) {
                for(it=adjEdges.begin(); it!=adjEdges.end(); ++it) {
                    if(!visitedEdges[*it]) {
                        q.push(*it);
                    }
                }
                visitedVertices[m_graph.getVProp(currVertex)] = true; // don't really need this but anyway
            }

            if(visitedEdges[currEdge]) {
                q.pop();
            } 
            if(!visitedEdges[currEdge] && ((currClusterSize + (currEdge).area) <=currClusterLimit) ) {
                currClusterSize += (currEdge).area;
                cluster_j.push_back(currEdge);
                visitedEdges[currEdge] = true;
                q_h.pop();
                q.pop();
            } else if ((currClusterSize + (currEdge).area) >currClusterLimit) {
                break;
            } 

            currVertex = nextVertex;
        }
    }
        if(!cluster_j.empty()) {
#ifdef DEBUG_CAC
            std::cout << "--- BEFORE findEulerTour is called --- " << cluster_j.size() << std::endl;
#endif
            //here cluster_j must be converted to the graph and CPP must be applied to get optimal euler tour
            EulerTour tour = findEulerTour(cluster_j, ij);
#ifdef DEBUG_CAC
            std::cout << "--- AFTER findEulerTour is called\n";
            std::cout << "in solve checking the size of the cluster -> "<<cluster_j.size() << std::endl;
#endif

            ij++;
            m_eulerTours.push_back(tour);
#ifdef DEBUG_CAC
            std::cout << "-- Pushing the tour into m_eulerTours --\n";
#endif
        }
        coverArea -=currClusterSize;

        k++;
    }
#ifdef DEBUG_CAC
            std::cout << "in solve checking the NUMBER of the clusters -> "<< m_eulerTours.size() << std::endl;
#endif
}

/**==============================================================
 * Returns sorted by X and Y axis adjacent list of edges attached to
 * a specific v vertex
 *
 * \param Vertex
 * \return double cost
 **==============================================================*/
std::list<ReebEdge> CAC::getAdjacentList(Vertex v)
{
    ReebEdge eprop;
    Out_Edge_Iter oi, oi_end;
    unsigned int currDegree;
    std::list<ReebEdge> adjacentList;

    assert(v != ReebGraph::nullVertex() && "the vertex v is null in getAsjacentList");

    currDegree = m_graph.degree(v);

    if (currDegree > 0) 
    {
        for (tie(oi, oi_end) = m_graph.getEdges(v); oi != oi_end; oi++)
        {
            eprop = m_graph.getEProp(*oi);
            adjacentList.push_back(eprop);
        }
    }
    adjacentList.sort(compareByYCoords);
    adjacentList.sort(compareByXCoords);
    return adjacentList;
}

/**==============================================================
 * Transformd cluster into reebgraph, runs cpp to find optimal tour
 *
 * \param cluster as a std::list<ReebGraph>
 * \return euler tour of type std::list<Edge>
 **==============================================================*/
EulerTour CAC::findEulerTour(kcpp::EulerTour cluster, int i)
{
    EulerTour eulerCycle; // std::list<edge> typdefed to EulerTour
    vector<Point2D> wayPoints;

    WayPoints way(m_data, m_graph, m_optimalPath, wayPoints);
    std::vector<std::vector<Point2D> > tourPoints;

    //temporary graph for converting reebedges to a new reeb graph for each tour
    ReebGraph temporaryGraph;
    std::vector<Point2D> tempWayPoints;

    way.convertTourToReebGraph(cluster, m_graph, temporaryGraph); 

    std::list<Edge> tmpBcCpp = temporaryGraph.getEdgeList();

    WayPoints tourWayPoints(m_data, temporaryGraph, tmpBcCpp, tempWayPoints);


    ChinesePostman cpp(m_data, temporaryGraph, eulerCycle, wayPoints);

    QString imageQS = QString("test");
    QString fileName;
    fileName = QString("%1.WayGraph.CRC.In.%2.png").arg(imageQS, QString::number(i));
#ifdef DEBUG_CAC
    std::cout << "findEulerTour is called: " << i << std::endl;
#endif
    //m_cpp.viewEulerGraph(fileName, data, graph, eulerCycle, wayPoints);
    tourWayPoints.viewWaypoints(fileName, m_data, temporaryGraph, tmpBcCpp, tempWayPoints);


    return eulerCycle;
}

/**==============================================================
 * Computing S_max value:
 * the cost of the fathest node in the Eulerian tour
 *
 * \param Eulerian tour
 * \return double cost
 **==============================================================*/
double CAC::computeSMax(EulerTour tour)
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
        smax = std::max(S_V1_Vij+S_Vij_1_V1+edge.area, smax);
    }
    smax = smax/2.0;
    return smax;
}


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

/* bunch of things that might be usefull: Playground 
 *
 * ReebGraph g;
 Edge e = Epointers[i];
 std::cout << "distance for edge -> " << g[e].Eid << " " << g[e].travelCost << std::endl;
 std::list<Edge>::iterator it;
 for (EulerTour::iterator it = tour_i.begin();
 it != tour_i.end(); ++it) 

    for (tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ei++) {
        currEdge = *ei;
        eprop = graph.getEProp(currEdge);
        m_sortedGraphEdges.push_back(eprop);
    }
    m_sortedGraphEdges.sort(compareByYCoords);
    m_sortedGraphEdges.sort(compareByXCoords);
 */
