//#define DEBUG_KCHINESEPOSTMAN
/**==============================================================
 * \file KChinesePostmen.cpp
 *
 * An Abstract class for all K Chinese Postman algorithms
 * The pure virtual function solve() must be implemented 
 * for each algorithm
 *
 * \author    Nare Karapetyan
 * \warning   don't use record_shortest_path_edge and m_vertices_
 *
 **==============================================================*/

#include "KChinesePostmen.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>

/**==============================================================
 *! Shortest edge recorder
 *
 * FIXME: doens't operate correctly.
 * boost visitor object for recording shortest path edges 
 * along with the predecessors as we deal with multigraphs
 **==============================================================*/
template <class T>
class record_shortest_path_edge : public boost::dijkstra_visitor<>
{
public:
    record_shortest_path_edge(T e)
        : m_edgePredecessor(e)
    {}

    template <class Edge, class Graph>
    void edge_relaxed(Edge e, Graph& g) {
        // set the parent of the target(e) to e
        boost::put(m_edgePredecessor, target(e, g), e);
#ifdef DEBUG_KCHINESEPOSTMAN
        std::cout << "from the caller \n";
        std::cout << "target is: " << target(e, g) << "  source is: " << source(e, g) ;
        std::cout << " and edge is "  << g[e].Eid;
        std::cout << " area is "  << g[e].area;
        std::cout << std::endl;
#endif
    }
protected:
    T m_edgePredecessor;
};

template <class T>
record_shortest_path_edge<T>
make_shortest_path_recorder(T e) {
    return record_shortest_path_edge<T>(e);
}

/**==============================================================
 *! Constructor
 **==============================================================*/
KChinesePostmen::KChinesePostmen()
{}

/**==============================================================
 *! Constructor
 *
 * FIXME: get functions from the ChinesePostman are not constants
 * must be redeclared as a const functions, 
 * so here ReebGraph as well can be passed as const
 *
 * \param  Reebgraph and the number of robots
 **==============================================================*/
    KChinesePostmen::KChinesePostmen(ReebGraph graph, int k)
: m_graph(graph),
    m_k(k)
{
        //cerr << "K1 \n";
    //create boost graph based on reebGraph's information
    // and sets also the staring point
    m_g = getSimpleGraph(/*m_graph*/);
#ifdef DEBUG_KCHINESEPOSTMAN
    Vertex_Iter vi, vi_end;
    for (tie(vi, vi_end) = m_graph.getVertices(); vi != vi_end; vi++) {
        // BUG: mapping fails, after getting out from scope the descriptor is set to 0
        std::cout << m_g[getVertex(*vi)].Vid << " /  "; 
    }
    testGraph(m_g);
#endif

    //FIXME: not sure if for all kcpp algorithms 
    // computing shortest paths is necessary
    computeShortestDistances(m_g);
#ifdef DEBUG_KCHINESEPOSTMAN
    printShortestPaths(m_g);
#endif

}

/**==============================================================
 * Shortest path and distance computing method 
 * based on area and travel costs 
 *
 * \param non multigraph 
 * \return void
 **==============================================================*/
void KChinesePostmen::computeShortestDistances(kcpp::Graph graph)
{

    //cerr << "computer shortest distance - inside 1" << "\n";
    
    m_shortCoverDistances.resize(boost::num_vertices(graph));
    m_shortTravelDistances.resize(boost::num_vertices(graph));

    m_coverEdgePredecessor.resize(boost::num_vertices(graph));
    m_travelEdgePredecessor.resize(boost::num_vertices(graph));

    m_coverPredecessors.resize(boost::num_vertices(graph));
    m_travelPredecessors.resize(boost::num_vertices(graph));


    //cerr << "computer shortest distance - inside 2" << "\n";

    /**Computing shortest paths based on area and travel costs*/
    //"-------------------------Based on Area Cost----------------------------------\n";
    boost::dijkstra_shortest_paths(graph, m_sourceVertex,
            boost::weight_map(boost::get(&ReebEdge::area,graph))
            .distance_map(boost::make_iterator_property_map(m_shortCoverDistances.begin(), boost::get(boost::vertex_index,graph)))
            .visitor(make_shortest_path_recorder(&m_coverEdgePredecessor[0]))
            .predecessor_map(boost::make_iterator_property_map(m_coverPredecessors.begin(), boost::get(boost::vertex_index,graph)))
            );

    //cerr << "computer shortest distance - inside 3" << "\n";

    //"-------------------------Based on Travel Cost----------------------------------\n";
    boost::dijkstra_shortest_paths(graph, m_sourceVertex,
            boost::weight_map(boost::get(&ReebEdge::travelCost,graph))
            .distance_map(boost::make_iterator_property_map(m_shortTravelDistances.begin(), boost::get(boost::vertex_index,graph)))
            .visitor(make_shortest_path_recorder(&m_travelEdgePredecessor[0]))
            .predecessor_map(boost::make_iterator_property_map(m_travelPredecessors.begin(), boost::get(boost::vertex_index,graph)))
            );

    //cerr << "computer shortest distance - inside 4" << "\n";
}

/**==============================================================
 * Builds a graph from multigraph with no parallel edges:
 * Removes all parallel edges that will 
 * not potentially appear in shortest path
 *
 * \param ReebGraph created by ChinesePostman algorithm
 * \return graph without parallel edges
 **==============================================================*/
kcpp::Graph KChinesePostmen::getSimpleGraph(/*ReebGraph g*/)
{
    ReebGraph g = m_graph;
#ifdef DEBUG_KCHINESEPOSTMAN
    std::cout << "Simple graph creationg is started -->\n";
#endif

    kcpp::Graph graph;

    Vertex_Iter vi, vi_end;
    Edge_Iter ei, ei_end;

    ReebVertex rVertex;
    ReebEdge rEdge;

#ifdef DEBUG_KCHINESEPOSTMAN
    std::cout << "---- Vertices are -->\n";
#endif
    std::vector<Vertex> cppVert = g.getGraphVertex();
    for (size_t i = 0; i < cppVert.size(); ++i) {
        Vertex v = cppVert.at(i);
        rVertex = g.getVProp(v);
        kcpp::Vertex vk = boost::add_vertex(rVertex, graph);
        m_vertices_.push_back(vk);
//      std::cout << i << " - " << (m_graph.getVProp((cppVert.at(i)))).Vid << " ,";
    }
#ifdef DEBUG_KCHINESEPOSTMAN
    std::cout << std::endl;
    for (tie(vi, vi_end) = g.getVertices(); vi != vi_end; vi++) {
        std::cout << graph[getVertex(*vi)].Vid << " /  ";
    }
    std::cout << std::endl;
#endif
    m_sourceVertex = getVertex(g.getFirstVertex());
#ifdef DEBUG_KCHINESEPOSTMAN
    std::cout << "source vertex ... \n";
    std::cout << graph[m_sourceVertex].Vid << std::endl;
    std::cout << "---- Edges are -->\n";
#endif

    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
    {
        //no need for check
        // all reebGraph's vertices must be included in the graph
        //if( *vi != ReebGraph::nullVertex()) 
        rEdge = g.getEProp(*ei); // the return type is ReebEdge

        //if there is not edge between source and target vertices
        //of current edge then add it to graph
        Vertex v_first, v_second;
        boost::tie(v_first, v_second) = g.getEndNodes(*ei); // <---- IMPORTANT
        kcpp::Vertex v1 = getVertex(v_first);
        kcpp::Vertex v2 = getVertex(v_second);
        std::pair<kcpp::Edge, bool> retrEdge = boost::edge(v1, v2, graph); // for checking if there is edge between this vertices
        if( retrEdge.second == false)  {
            add_edge(v1, v2, rEdge, graph);
            //std::cout << rEdge.Eid << " ";
        } else  {
            //otherwise replace it with an edge that has min travel cost
            //FIXME: if we have to compute also based on area cost  we have to include that edges as well

            ReebEdge retrREdge = graph[retrEdge.first];
            if (rEdge.area < retrREdge.area) { //FIXME: just for checking if the computation is correct
            //if (rEdge.travelCost < retrREdge.travelCost) { 
                //FIXME: remove might cause some problems
                //remove(source(*ei, graph), target(*ei, graph), graph);
                //add_edge(rEdge, graph);
                graph[retrEdge.first] = rEdge;
                //std::cout << rEdge.Eid << " ";
            }
        }

    }
#ifdef DEBUG_KCHINESEPOSTMAN
    std::cout <<std::endl;
    std::cout << "Simple graph is created! \n";
#endif
    return graph;
}

/**==============================================================
 * Computes the cost of a given tour
 *
 * \param Eulerian tour
 * \return the cost of the tour
 **==============================================================*/
double KChinesePostmen::pathCost(EulerTour tour)
{
    double pathCost = 0.0;
    double pathTravelCost = 0.0;
    EulerTour::iterator ei = tour.begin();
    EulerTour::iterator ei_end = tour.end();
    std::cerr << " The area costs: \n";
    for (; ei != ei_end; ei++){
        ReebEdge edge = m_graph.getEProp(*ei);
        pathCost+= edge.area;

#ifdef DEBUG_KCHINESEPOSTMAN
//      std::cout << edge.Eid << " - ";
//      pathTravelCost += edge.travelCost;
        std::cerr << edge.area << " " ;
#endif
    }
    std::cerr << "\n";

#ifdef DEBUG_KCHINESEPOSTMAN
    std::cout << std::endl;
    std::cout << pathTravelCost << std::endl;
#endif
    return pathCost;
}

/**==============================================================
 * Builds a short path for spcified target vertex from the source
 *
 * \param v is the traget vertex
 * \param g is the non multigraph
 * \return vector of ReebEdges
 **==============================================================*/
std::list<ReebEdge> KChinesePostmen::getShortPath(Vertex v, kcpp::Graph g)
{
    std::list<ReebEdge> ePath;
    std::vector<kcpp::Vertex> vPath;
    kcpp::Vertex curr = getVertex(v);
    //std::cout << m_graph.getVProp(v).Vid << "  "  << g[curr].Vid << " ";
    //kcpp::Vertex curr = v;
    while(curr!=m_sourceVertex) {
        vPath.push_back(curr);
        curr = m_travelPredecessors.at(curr);
    }
    vPath.push_back(m_sourceVertex);

    std::vector< kcpp::Vertex >::reverse_iterator rit, next;
    if(vPath.size() <= 1){
        return ePath;
    }
    for ( rit=vPath.rbegin(), next = rit + 1; next!=vPath.rend(); ++next) 
    {
        
        std::pair<kcpp::Edge, bool> edgeDesc = boost::edge(*rit, *next, g);
#ifdef DEBUG_KCHINESEPOSTMAN
            std::cout << g[*rit].Vid << " - "  << g[*next].Vid << std::endl;
#endif
        if(edgeDesc.second == true) {
            ReebEdge edge = g[edgeDesc.first];
            ePath.push_back(edge);
        }
        rit = next;
    }
    return ePath;
}

/**==============================================================
 * Helper function for debugging the created routes
 * Prints out the edge indexes in that are in the routes
 *
 * \param none
 * \return void
 **==============================================================*/
void KChinesePostmen::printEulerianTours()
{
            /*Vertex v_first, v_second;

            Edge e = tour_j.front();
            tie(v_first, v_second) = m_graph.getEndNodes(e);
            std::list<ReebEdge> t1 = getShortPath(v_first, m_g);
            tour_jv.splice(tour_jv.begin(), t1);

            e = tour_j.back();
            tie(v_first, v_second) = m_graph.getEndNodes(e);
            t1 = getShortPath(v_second, m_g);
            tour_jv.splice(tour_jv.end(), t1);
            */

    //std::cerr << "start \n";
    
    std::cout << "----------------------Eulerian Tours-----------------------------\n";
    std::cout << "The number of robots is: " << m_k << "\n";
    std::cout << "The number of routes is: " << m_eulerTours.size() << "\n";
    for(size_t i = 0; i < m_eulerTours.size(); ++i) {
        std::cout << "Route no " <<  i+1 << "  -->  ";
        EulerTour tour_i = m_eulerTours.at(i);
            Vertex v_first, v_second;

    std::cerr << "1/4 \n";

        Edge e = tour_i.front();
        tie(v_first, v_second) = m_graph.getEndNodes(e);
        std::list<ReebEdge> t1 = getShortPath(v_first, m_g);
        std::list<ReebEdge>::iterator i1;
        std::cout << " travel edges ( ";
        for(i1 = t1.begin(); i1!=t1.end(); ++i1) {
            std::cout << (*i1).Eid << " ";
        }
        std::cout << " )";

    std::cerr << "1/2 \n";

        std::cout << " coverage edges ( ";
        for (EulerTour::iterator it = tour_i.begin(); it!=tour_i.end(); ++it) {
            std::cout << m_graph.getEProp(*it).Eid << " ";
        }
        std::cout << " )";

    std::cerr << "3/4 \n";

        e = tour_i.back();
        tie(v_first, v_second) = m_graph.getEndNodes(e);
        t1 = getShortPath(v_second, m_g);
        std::cout << " travel edges ( ";
        for(i1 = t1.begin(); i1!=t1.end(); ++i1) {
            std::cout << (*i1).Eid << " ";
        }
        std::cout << " )";

        std::cout << std::endl;
    }

    std::cerr << "end \n";

    std::cout << "----------------------------------------------------------------\n";
}

/**==============================================================
 * Helper function for debugging the created non multigraph
 * Prints out the basic graph information
 *
 * \param testing graph 
 * \return void
 **==============================================================*/
void KChinesePostmen::testGraph(kcpp::Graph graph)
{
    kcpp::Edge_Iter ei, eend;
    std::cout << "------ Edges of simple graph created from multigraph----\n";
    for (boost::tie(ei, eend) = boost::edges(graph); ei != eend; ++ei) {
        ReebEdge edge = graph[*ei];
        std::cout << edge.Eid << " area " << edge.area << " travel cost " << edge.travelCost << std::endl;
    }
    std::cout << "------------------ !!!!!!!!! ---------------------------\n";
}

/**==============================================================
 * Helper function for debugging the computeShortestDistances(...)
 * and getShortPath(...) functions
 * Prints out shortest path from source 
 * for each target vertex in the graph
 *
 * \param the graph for which the shortest paths are computed
 * \return void
 * \sa computeShortestDistances(), getShortPath()
 **==============================================================*/
void KChinesePostmen::printShortestPaths(kcpp::Graph graph)
{
    std::cout << "----------------------SHORTEST PATH-----------------------------\n";
    Vertex_Iter vi, vend;
    for (boost::tie(vi, vend) = m_graph.getVertices(); vi != vend; vi++) {
        kcpp::Vertex v = getVertex(*vi);
        std::cout << graph[v].Vid << " the path is ";
        std::list<ReebEdge> path = getShortPath(*vi, graph);
        std::list<ReebEdge>::iterator it;
        for ( it=path.begin(); it!=path.end(); ++it) {
            std::cout << (*it).Eid << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "----------------------------------------------------------------\n";

}


