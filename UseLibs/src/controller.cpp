#include "controller.h"

Controller::Controller() :
    m_cppSolved(false), m_k(1) {}


/*************************************************************************
 * Function 'run()'
 *
 * Runs BCD, CPP and if k is > 1 also KCPP algorithms 
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void Controller::run(const std::string& directory, const std::string& image,
        int k)
{
    RegionData data;
    ReebGraph graph;
    std::list<Edge> eulerCycle;
    vector<Point2D> wayPoints;

    std::cout << "Controller's run executed\n";
    checkInputParams(directory, image, k);
    std::cerr << "here...\n";

    m_directory = directory;
    m_image = image;
    m_k = k;

    //-------------------- BCD call -------------------------
    std::cerr << "BCD is starting...\n";
#ifdef DEBUG
    std::cerr << "BCD is starting...\n";
#endif

    BCD bcd(directory, image, data, graph);

#ifdef DEBUG
    std::cerr << "BCD is completed\n"; 
#endif
    std::cerr << "BCD is completed\n"; 

    //----------------- BCD call ended ----------------------

    ChinesePostman m_cpp(data, graph, eulerCycle, wayPoints);

    std::cerr << "After running cpp, checking the m_cpp var ...\n";
#ifdef DEBUG

    std::cout << "After running cpp, checking the m_cpp var ...\n";
    std::cout << eulerCycle;
    std::cout << graph;
    std::cout << std::endl;
    std::cout << "Checking is passed!\n";
    m_cppSolved = true;

#endif
    std::cerr << "Checking is passed!\n";

    m_cppSolved = true;
    if(k > 1) 
    {
        std::cerr << "k greater \n";
        runkCPP(m_k, graph, eulerCycle);
        m_kcpp->printEulerianTours();
        std::cout << std::endl;
    }
    else
    {
        std::cerr << "k not greater \n";
        m_tours.push_back(eulerCycle);
    }

    WayPoints way(data, graph, eulerCycle, wayPoints);

    std::vector<std::vector<Point2D> > tourPoints;

    std::vector<EulerTour> m_eulerTours = m_kcpp->getKEulerianTours();


    std::cerr << "start \n";

    std::cout << "----------------------Eulerian Tours-----------------------------\n";
    std::cout << "The number of robots is: " << m_kcpp->m_k << "\n";
    std::cout << "The number of routes is: " << m_eulerTours.size() << "\n";
    for(size_t i = 0; i < m_eulerTours.size(); ++i) {
        std::cout << "Route no " <<  i+1 << "  -->  ";
        EulerTour tour_i = m_eulerTours.at(i);
        Vertex v_first, v_second;

        std::cerr << "1/4 \n";

        Edge e = tour_i.front();
        tie(v_first, v_second) = (m_kcpp->m_graph).getEndNodes(e);
        std::list<ReebEdge> t1 = m_kcpp->getShortPath(v_first, m_kcpp->m_g);
        std::list<ReebEdge>::iterator i1;
        std::cout << " travel edges ( ";
        for(i1 = t1.begin(); i1!=t1.end(); ++i1) {
            std::cout << (*i1).Eid << " ";
        }
        std::cout << " )";

        std::cerr << "1/2 \n";

        std::cout << " coverage edges ( ";
        for (EulerTour::iterator it = tour_i.begin(); it!=tour_i.end(); ++it) {
            std::cout << (m_kcpp->m_graph).getEProp(*it).Eid << " ";
        }
        std::cout << " )";

        std::cerr << "3/4 \n";

        e = tour_i.back();
        tie(v_first, v_second) = (m_kcpp->m_graph).getEndNodes(e);
        t1 = m_kcpp->getShortPath(v_second, m_kcpp->m_g);
        std::cout << " travel edges ( ";
        for(i1 = t1.begin(); i1!=t1.end(); ++i1) {
            std::cout << (*i1).Eid << " ";
        }
        std::cout << " )";

        std::cout << std::endl;
    }

    std::cerr << "end \n";

    std::cout << "----------------------------------------------------------------\n";



    std::cerr <<"Size: "<< m_eulerTours.size() << "\n";
    for (size_t i = 0; i < m_eulerTours.size(); ++i)
    {
        std::cerr << "Route no " <<  i+1 << "  -->  ";
        EulerTour tour_i = m_eulerTours.at(i);

        Vertex v_first, v_second;
        Edge e = tour_i.front();
        tie(v_first, v_second) = (m_kcpp->m_graph).getEndNodes(e);
        //kcpp::Graph m_g = m_kcpp->getSimpleGraph();
        std::list<ReebEdge> t1 = m_kcpp->getShortPath(v_first, m_kcpp->m_g);

        cerr << "\nTour 1:\n";
        std::list<ReebEdge>::iterator it;
        for (it = t1.begin(); it != t1.end(); ++it)
        {
            std::cerr << "ReebEdge: (" /*<< it->topBoundary << ","
                << it->bottomBoundary << ","*/ << it->color << "," << it->cost
                << "," << it->Eid << "," << it->area << "," << it->travelCost
                << ")\n";
            std::cerr << "  Top Boundary:" << std::endl;
            vector<Point2D>::iterator itTop;
            for (itTop = it->topBoundary.begin();
                    itTop != it->topBoundary.end(); ++itTop)
            {
                std::cerr << "\t" << &(*itTop);
            }
            std::cerr << std::endl;
            std::cerr << "  Bottom Boundary:" << std::endl;
            vector<Point2D>::iterator itBot;
            for (itBot = it->bottomBoundary.begin();
                    itBot != it->bottomBoundary.end(); ++itBot)
            {
                std::cerr << "\t" << &(*itBot);
            }
            std::cerr << std::endl;
        }

        //temporary graph that will be used for converting
        //  reebedges to a new reeb graph for each tour
        ReebGraph temporaryGraph;
        std::vector<Point2D> tempWayPoints;

        //only used in modifying temporaryGraph. Nothing else. 
        way.convertTourToReebGraph(t1, m_kcpp->m_graph, temporaryGraph); 

        std::cerr << "\n\nVerticies:\n";
        temporaryGraph.printVertex();
        std::cerr << "\nEdges:\n";
        temporaryGraph.printEdges();

        std::list<Edge> tmpBcCpp = temporaryGraph.getEdgeList();
        std::cerr << "\nEdge count: " << tmpBcCpp.size() << "\n";

        std::cerr << std::endl << "Graph Edges:" << std::endl;
        std::list<Edge>::iterator tbcit;
        for (tbcit = tmpBcCpp.begin(); tbcit != tmpBcCpp.end(); ++tbcit)
        {
            std::cerr << " Edge:" << std::endl;
            ReebEdge e = temporaryGraph.getEProp(*tbcit);
            std::cerr << "  Top Boundary:" << std::endl;
            vector<Point2D>::iterator itTop;
            for (itTop = e.topBoundary.begin();
                    itTop != e.topBoundary.end(); ++itTop)
            {
                std::cerr << "\t" << &(*itTop);
            }
            std::cerr << std::endl;
            std::cerr << "  Bottom Boundary:" << std::endl;
            vector<Point2D>::iterator itBot;
            for (itBot = e.bottomBoundary.begin();
                    itBot != e.bottomBoundary.end(); ++itBot)
            {
                std::cerr << "\t" << &(*itBot);
            }
            std::cerr << std::endl;
        }
        std::cerr << std::endl;

        //should generate a new waypoints path for this route
        //  Done like this because the constructor automatically runs
        //  the functions necessary to create waypoints. 
        WayPoints tourWayPoints(data, temporaryGraph, tmpBcCpp, tempWayPoints);

        cout << "\nWayPoints:\n";
        vector<Point2D>::iterator iter;
        for (iter = tempWayPoints.begin(); iter != tempWayPoints.end(); ++iter)
        {
            cout << (*iter) << " ";
        }
        cout << "\n\n";

        //pushes each tours waypoints to store for future use
        tourPoints.push_back(tempWayPoints);

        int rmExt = m_image.find_last_of("."); 
        string img = m_image.substr(0, rmExt); 
        QString imageQS = QString(img.c_str());
        QString fileName = QString("%1.WayGraph.%2.png").arg(imageQS, QString::number(i));
        //m_cpp.viewEulerGraph(fileName, data, graph, eulerCycle, wayPoints);
        tourWayPoints.viewWaypoints(fileName, data, temporaryGraph, tmpBcCpp, tempWayPoints);
    }

    for (int i = 0; i < tourPoints.size(); ++i)
    {
        cerr << "\n";
        cerr << "START" << "\n"; 
        std::vector<Point2D> tempPoints = tourPoints.at(i);

        vector<Point2D>::iterator iter;
        for(iter = tempPoints.begin(); iter != tempPoints.end(); ++iter)
        {
            cout << (*iter) << " ";
        }

        cerr << "\n";
        cerr << "END" << "\n"<< "\n";
    }

}


/*************************************************************************
 * Function 'runkCPP'
 *
 * Runs the kcpp algorithm.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
 //void Controller::runkCPP(int k, ReebGraph graph, std::list<Edge> eulerCycle)
void Controller::runkCPP(int k, ReebGraph& graph, std::list<Edge>& eulerCycle)
{
    assert(m_cppSolved && "runCPP must be called before runkCPP");

    try
    {
        m_kcpp = new FredericksonKCPP(eulerCycle, graph, k);
        m_kcpp->solve();
    }

    catch (const std::string& err) 
    {
        std::cout << "runkCPP error";
    }

}


/*************************************************************************
 * Function 'checkInputParams()'
 *
 * Checks the input parameters.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void Controller::checkInputParams(const std::string& directory,
        const std::string& image, int k)
{
    std::string message = "";

    if(k<1) 
    {
        message = "ERR:Number of robots must be positive integer!";
        throw std::invalid_argument(message);
    }

    /**TODO: Add also check for valid image format*/
    if( !std::ifstream((directory + image).c_str())) 
    {
        message = "ERR:There is no image at this path:  " + directory + image;
        throw std::invalid_argument(message);
    }
}


/*
Interesting idea for waypoints issue. Why not find the beggining vertex
of each edge, then use that to search the existing waypoints for the
waypoints to hit. 



        std::cerr << "start \n";

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


*/
