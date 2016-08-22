#define DEBUG
#include "controller.h"

Controller::Controller() : m_cppSolved(false), m_k(1) {}

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

#ifdef DEBUG
    std::cout << "Controller's run executed\n";
#endif

    checkInputParams(directory, image, k);

    m_directory = directory;
    m_image = image;
    m_k = k;

    //-------------------- BCD call -------------------------

#ifdef DEBUG
    std::cerr << "BCD is starting...\n";
#endif

    BCD bcd(directory, image, data, graph);

#ifdef DEBUG
    std::cerr << "BCD is completed\n"; 
#endif

    //----------------- BCD call ended ----------------------

    ChinesePostman m_cpp(data, graph, eulerCycle, wayPoints);

#ifdef DEBUG
    //std::cout << "After running cpp, checking the m_cpp var ...\n";
    //std::cout << eulerCycle;
    //std::cout << graph;
    //std::cout << std::endl;
    //std::cout << "Checking is passed!\n";
    //m_cppSolved = true;
#endif

    m_cppSolved = true;
    if(k > 1) 
    {

#ifdef DEBUG
        std::cerr << "k greater \n";
#endif

        runkCPP(m_k, graph, eulerCycle);
        m_kcpp->printEulerianTours();
        std::cout << std::endl;
    }
    else
    {

#ifdef DEBUG
        std::cerr << "k not greater \n";
#endif

        m_tours.push_back(eulerCycle);
    }

    WayPoints way(data, graph, eulerCycle, wayPoints);

    //Coverage edges are the ones you need to use to move
    std::vector<std::vector<Point2D> > tourPoints;
    std::vector<EulerTour> m_eulerTours = m_kcpp->getKEulerianTours();
    for (size_t i = 0; i < m_eulerTours.size(); ++i)
    {
        EulerTour tour_i = m_eulerTours.at(i);

        std::list<ReebEdge> t1;
        for (EulerTour::iterator it = tour_i.begin();
                it != tour_i.end(); ++it) {
            t1.push_back((m_kcpp->m_graph).getEProp(*it));
        }

#ifdef DEBUG
        cerr << "\nCOVERAGE"<< ":\n";
        cerr << "\nTour " << i << ":\n";
        std::list<ReebEdge>::iterator it;
        for (it = t1.begin(); it != t1.end(); ++it)
        {
            std::cerr << "ReebEdge: (" /*<< it->topBoundary << ","
                << it->bottomBoundary << ","*/ << it->color << "," << it->cost
                << "," << it->Eid << "," << it->area << "," << it->travelCost
                << ")\n";
        }
#endif

        //temporary graph that will be used for converting
        //  reebedges to a new reeb graph for each tour
        ReebGraph temporaryGraph;
        std::vector<Point2D> tempWayPoints;

        //only used in modifying temporaryGraph. Nothing else. 
        way.convertTourToReebGraph(t1, m_kcpp->m_graph, temporaryGraph); 

#ifdef DEBUG
        std::cerr << "\n\nVerticies:\n";
        temporaryGraph.printVertex();
        std::cerr << "\nEdges:\n";
        temporaryGraph.printEdges();
#endif

        std::list<Edge> tmpBcCpp = temporaryGraph.getEdgeList();

#ifdef DEBUG
        std::cerr << "\nEdge count: " << tmpBcCpp.size() << "\n";
        std::cerr << std::endl << "Graph Edges:" << std::endl;
        std::list<Edge>::iterator tbcit;
        for (tbcit = tmpBcCpp.begin(); tbcit != tmpBcCpp.end(); ++tbcit)
        {
            std::cerr << " Edge:" << std::endl;
            ReebEdge e = temporaryGraph.getEProp(*tbcit);

        }
        std::cerr << std::endl;
#endif

        //should generate a new waypoints path for this route
        //  Done like this because the constructor automatically runs
        //  the functions necessary to create waypoints. 
        WayPoints tourWayPoints(data, temporaryGraph, tmpBcCpp, tempWayPoints);

#ifdef DEBUG
        cout << "\nWayPoints:\n";
        vector<Point2D>::iterator iter;
        for (iter = tempWayPoints.begin(); iter != tempWayPoints.end(); ++iter)
        {
            cout << (*iter) << " ";
        }
        cout << "\n\n";
#endif

        //pushes each tours waypoints to store for future use
        tourPoints.push_back(tempWayPoints);

        int rmExt = m_image.find_last_of("."); 
        string img = m_image.substr(0, rmExt); 
        QString imageQS = QString(img.c_str());
        QString fileName = QString("%1.WayGraph.%2.png").arg(imageQS, QString::number(i));
        //m_cpp.viewEulerGraph(fileName, data, graph, eulerCycle, wayPoints);
        tourWayPoints.viewWaypoints(fileName, data, temporaryGraph, tmpBcCpp, tempWayPoints);
    }

#ifdef DEBUG
    for (int i = 0; i < tourPoints.size(); ++i)
    {
        cerr << "\n" << "Start Tour " << i << "\n"; 
        std::vector<Point2D> tempPoints = tourPoints.at(i);

        vector<Point2D>::iterator iter;
        for(iter = tempPoints.begin(); iter != tempPoints.end(); ++iter)
        {
            cout << (*iter) << " ";
        }

        cerr << "\n" << "End Tour " << i << "\n";
    }
#endif
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

