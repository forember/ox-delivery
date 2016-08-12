#ifndef ChinesePostman_H_
#define ChinesePostman_H_

/******************************************************************************
 *
 * @author: Kelly Benson
 * @email: bensonke@email.sc.edu
 * @date: May 16, 2015
 *
 * Header file for the ChinesePostman calculation.
 *
**/

#include <iostream>
#include <sstream>
#include <QString>
#include <glpk.h>
#include <setjmp.h>
#include "Leo/RegionData.h"
#include "Leo/ReebGraph.h"
#include "DrawImage.h"

class ChinesePostman 
{

public:

    ChinesePostman();
    ChinesePostman(RegionData& data, ReebGraph& graph,
            std::list<Edge>& eulerCycle, vector<Point2D>& wayPoints);
    ~ChinesePostman(); 

    QImage getImageBuffer(); 
    void setImageBuffer(cv::Mat* const source);
    void setImageBuffer(QImage var);    
    void clearImage();

    void viewEulerGraph(QString fileName, RegionData& data, ReebGraph& graph,
            std::list<Edge>& eulerCycle, vector<Point2D>& wayPoints);
    void printGraphInfo();
    void testValues(RegionData& data, ReebGraph& graph); 

    /**
     * Solves the Chinese Postman Problem using linear programming
     *
     * WARNING: The user should check if eulerCycle->size() > 0 after calling
     * this function to see if linear programming has returned a valid
     * solution.
     */
    void solveCPP(RegionData& data, ReebGraph& graph,
            std::list<Edge>& eulerCycle);

protected:

    ReebVertex vert;

    DrawImage imageDrawer;



    const static unsigned int WAYPOINT_NORMAL_RADIUS = 6;
    const static unsigned int WAYPOINT_HIGHLIGHT_RADIUS = 8;

    // Constants
    const static unsigned char BLACK = 0;
    const static unsigned char WHITE = 255;
    const static double desiredAltitudeM = 200;


    // Helper function: forms an end-pair from two selected edges
    static void addPairToEndPairings(
            Edge e1, Edge e2, Vertex n,
            std::list<Edge> *unpairedEdges,
            std::list<Edge> *pairedEdges,
            std::list< std::pair<std::pair<Edge,Edge>,Vertex> > *EndPairs);


    static glp_prob* lp;
            // linear programming pointer, static due to callback requirement
    static bool glp_successful;
            // deprecated -- flag written to, but not being used

    Edge firstEdge;
    bool forwardEulerTour;

};
#endif
