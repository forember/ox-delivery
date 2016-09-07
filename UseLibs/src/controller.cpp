#define DEBUG
#include "controller.h"
#include <fstream>

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
//    std::cout << "Controller's run executed\n";
#endif

    checkInputParams(directory, image, k);

    m_directory = directory;
    m_image = image;
    m_k = k;

    //-------------------- BCD call -------------------------

#ifdef DEBUG
//    std::cerr << "BCD is starting...\n";
#endif

    BCD bcd(directory, image, data, graph);

#ifdef DEBUG
 //   std::cerr << "BCD is completed\n"; 
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

#ifdef DEBUG
   //     std::cerr << "k greater \n";
#endif

        runkCPP(m_k, graph, eulerCycle);
        m_kcpp->printEulerianTours();
        m_tours = m_kcpp->getKEulerianTours();
        std::cout << std::endl;

    WayPoints way(data, graph, eulerCycle, wayPoints);

#ifdef DEBUG
    cout << "----------------- Print Original EulerTour ------------";
    cout << "\n";
    std::list<Edge>::iterator itEC;
    for (itEC = eulerCycle.begin(); itEC != eulerCycle.end(); ++itEC) 
    {
        Edge edge = (*itEC);
        Vertex v1,v2;
        tie(v1,v2) = graph.getEndNodes(edge);

        ReebVertex vertexOne = graph.getVProp(v1);
        ReebVertex vertexTwo = graph.getVProp(v2);

        Point2D first = Point2D(vertexOne.x,
            (vertexOne.y1 + vertexOne.y2) / 2);
        Point2D second = Point2D(vertexTwo.x,
            (vertexTwo.y1 + vertexTwo.y2) / 2);

        std::cout << first;
        std::cout << " ";
        std::cout << second;
        ReebEdge redge = graph.getEProp(edge);
        std::cout << " " << redge.Eid << " | ";
    }
    cout << "----------------- End Original EulerTour ------------" << "\n\n";

    std::cout << "----------------- Print Original WayPoints ------------";
    cout << "\n";
    std::vector<Point2D>::iterator itW;
    for (itW = wayPoints.begin(); itW != wayPoints.end(); ++itW) 
    {
        cout << (*itW) << " ";
    }
    cout << "\n";
    std::cout << "----------------- End Original WayPoints ------------";
    cout << "\n\n";

#endif

    //Coverage edges are the ones you need to use to move
    std::vector<std::vector<Point2D> > tourPoints;
    for (size_t i = 0; i < m_tours.size(); ++i)
    {
        EulerTour tour_i = m_tours.at(i);

        std::list<ReebEdge> t1;
        for (EulerTour::iterator it = tour_i.begin();
                it != tour_i.end(); ++it) {
            t1.push_back((m_kcpp->m_graph).getEProp(*it));
        }

#ifdef DEBUG
        std::cout << "----------------- Coverage Edges ------------";
        cout<<"\n";
        cerr << "Tour " << i << ":\n";
        std::list<Edge>::iterator it;
        for (EulerTour::iterator it = tour_i.begin();
                it != tour_i.end(); ++it) 
        {
            Vertex v1,v2;
            tie(v1,v2) = (m_kcpp->m_graph).getEndNodes(*it);

            ReebVertex vertexOne = (m_kcpp->m_graph).getVProp(v1);
            ReebVertex vertexTwo = (m_kcpp->m_graph).getVProp(v2);

            Point2D first = Point2D(vertexOne.x,
                (vertexOne.y1 + vertexOne.y2) / 2);
            Point2D second = Point2D(vertexTwo.x,
                (vertexTwo.y1 + vertexTwo.y2) / 2);

            std::cout << first;
            std::cout << " ";
            std::cout << second;
            ReebEdge edge = (m_kcpp->m_graph).getEProp(*it);
            std::cout << " " << edge.Eid << " | ";
        }
        cout << "\n";
        std::cout << "----------------- End Coverage Edges ------------";        
        cout << "\n";
#endif

        //temporary graph that will be used for converting
        //  reebedges to a new reeb graph for each tour
        ReebGraph temporaryGraph;
        std::vector<Point2D> tempWayPoints;

        //only used in modifying temporaryGraph. Nothing else. 
        way.convertTourToReebGraph(t1, m_kcpp->m_graph, temporaryGraph); 

        std::list<Edge> tmpBcCpp = temporaryGraph.getEdgeList();


        //should generate a new waypoints path for this route
        //  Done like this because the constructor automatically runs
        //  the functions necessary to create waypoints. 
        WayPoints tourWayPoints(data, temporaryGraph, tmpBcCpp, tempWayPoints);

#ifdef DEBUG
        cout<<"\n";
        std::cout << "----------------- Coverage WayPoints ------------";
        cout<<"\n";
        cerr << "Tour " << i << ":\n";
        vector<Point2D>::iterator iter;
        for (iter = tempWayPoints.begin(); iter != tempWayPoints.end(); ++iter)
        {
            cout << (*iter) << " ";
        }
        cout << "\n";
        std::cout << "----------------- End Coverage WayPoints ------------";

        std::cerr << "\n\nVerticies:\n";
        temporaryGraph.printVertex();
        std::cerr << "\nEdges:\n";
        temporaryGraph.printEdges();

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

        //pushes each tours waypoints to store for future use
        tourPoints.push_back(tempWayPoints);

        int rmExt = m_image.find_last_of("."); 
        string img = m_image.substr(0, rmExt); 
        QString imageQS = QString(img.c_str());
        QString fileName = QString("%1.WayGraph.%2.png").arg(imageQS, QString::number(i));
        m_cpp.viewEulerGraph(fileName, data, graph, eulerCycle, wayPoints);
        tourWayPoints.viewWaypoints(fileName, data, temporaryGraph, tmpBcCpp, tempWayPoints);
    }

#ifdef DEBUG
		std::ofstream outputFile;
		outputFile.open("tourLines.txt");
		for (unsigned i = 0; i < tourPoints.size(); ++i)
    {
        outputFile << "\n" << "Start Tour " << i << "\n"; 
        std::vector<Point2D> tempPoints = tourPoints.at(i);

        vector<Point2D>::iterator iter;
        for(iter = tempPoints.begin(); iter != tempPoints.end(); ++iter)
        {
            outputFile << (*iter) << " ";
        }

        outputFile << "\n" << "End Tour " << i << "\n";
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

