#ifndef WayPoints_H_
#define WayPoints_H_

/******************************************************************************
 *
 * @author: Kelly Benson
 * @email: bensonke@email.sc.edu
 * @date: May 16, 2015
 *
 * Header file for the WayPoints calculation.
 *
**/

#include <iostream>
#include <sstream>
#include <QString>
#include <utility> 
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "Leo/RegionData.h"
#include "Leo/ReebGraph.h"
#include "Leo/Point2D.h"
#include "DrawImage.h"

class WayPoints 
{

public:

    enum ServiceType {SERVICE_NONE, SERVICE_ONLINE_LOOP, SERVICE_INVESTIGATE};

    WayPoints();
    WayPoints(RegionData& data, ReebGraph& graph, std::list<Edge>& eulerCycle,
            vector<Point2D>& wayPoints);
    ~WayPoints(); 

    void convertTourToReebGraph(std::list<ReebEdge> &tour, ReebGraph &m_graph,
            ReebGraph &dest);

    /// Returns the greedy version starting from a random vertex
    void genPath_All(RegionData& data, ReebGraph& graph,
            std::vector<Point2D>& buffer, double altitude,
            int start_vertex = -1, Edge firstEdge = ReebGraph::nullEdge(),
            boost::optional<std::list<Edge> > tour = boost::optional<std::list<Edge> >())
        throw (const std::string&);

    static std::pair<Vertex, bool> genPath_SeedSpreader(ReebGraph& g,
            std::vector<Point2D>& path, Edge currEdge, Vertex startingVertex,
            Edge nextEdge, bool upDir, double coverWidth, bool completeCover)
        throw (const std::string&);

    // In meters
    double computeTotalPathLength(RegionData& data,
            const std::vector<Point2D>& buffer);

    // In meters
    void computeAllTotalPathLengths(RegionData& data, ReebGraph& graph,
            std::vector<double>& resultBuffer,
            double altitudeM = DEFAULT_UAV_VIEW_ALL_WAYPOINTS_COVERAGE_ALTITUDE_METERS)
        throw (const std::string&);

    static unsigned int computeNumberOfTurns(
            const std::vector<Point2D>& buffer);

    // NOTE: need to pass "algo" since need to run offlineSetup() during
    // analysis, which is polymorphic
    static unsigned int analyzeRotationEffects(
            std::vector<double>& anglesBuffer,
            std::vector<double>& pathLengthsBuffer,
            std::vector<unsigned int>& numTurnsBuffer, double angleIncrDeg = 1,
            double altitudeM = DEFAULT_UAV_VIEW_ALL_WAYPOINTS_COVERAGE_ALTITUDE_METERS);

    /**
     * Returns the closest vertex to the current point (based on the mid-
     * point of the vertex) and also returns the closest edge that is
     * connected to this vertex based on the edge's midpoint.
     *
     * Throws error if Reeb graph is empty
     */
    static std::pair<Vertex, Edge> findClosestGraphPos(ReebGraph& g,
            double xPixel, double yPixel) throw (const std::string&);

    static void pruneDuplicates(std::vector<Point2D>& waypoints);

    // Remove points that are very close (i.e. within 2 pixel radius close)
    // and also remove points on the middle of 2 near-straight lines
    static void prunePathPoints(std::vector<Point2D>& buffer);

    void resetOnlineLoopState();
    void stopServices();
    void startOnlineLoop();
    void stopOnlineLoop();

    void startInvestigate(double centerLat = INVALID_DEGREE_VALUE,
            double centerLon = INVALID_DEGREE_VALUE);

    // Auxiliary functions
    bool isSetupCompleted();
    bool isRunningOnlineLoop();
    bool isRunningInvestigate();
    void stopInvestigate();

    /**
     * Returns a string containing the state variables needed to restart the
     * online loop at the current state
     */
    std::string serializeOnlineLoopState();

    void setOnlineLoopState(const std::string& state)
        throw (const std::string&);

    void viewWaypoints(QString fileName, RegionData& data, ReebGraph& graph,
            std::list<Edge>& eulerCycle, vector<Point2D>& wayPoints);

    /// Returns the greedy version starting from a specific vertex
    void genPath_Ind(RegionData& data, ReebGraph& graph,
            std::vector<Point2D>& buffer, std::list<Edge> eulerCycle,
            double altitude, int start_vertex = -1,
            Edge firstEdge = ReebGraph::nullEdge())
        throw (const std::string&);

    vector<Point2D> getWayPoints();
    void setWayPoints(vector<Point2D> givenPoints);
    void printWayPoints();
    void printWayPoints(vector<Point2D> givenPoints);

private:
    vector<Point2D> points;

protected:
    // Helper function used by findClosestGraphPos()
    static bool updateMinDist(double newDx, double newDy,
            double& prevDistSqrd);

    DrawImage imageDrawer;

    // Variables
    boost::thread serviceThread;
    boost::mutex serviceQueueMutex;
    std::list<enum ServiceType> serviceQueue;

    bool _serviceThreadAlive;
    bool _onlineLoopAlive;
    bool _investigateAlive;
    bool _setupCompleted;

    std::pair<double, double> investigateCenterGCS;
    bool investigateResumeOnlineLoop;

    // Online loop state
    unsigned int numCellsCovered;
    Vertex currVertex;
    Edge currEdge;
    bool upDir;
    unsigned int resumedWaypointIndex;
    unsigned int resumedCommandIndex;
    bool resumeOnlineLoop;

    // Constants
    const static unsigned char BLACK = 0;
    const static unsigned char WHITE = 255;
    const static double desiredAltitudeM = 200;
    const static unsigned int WAYPOINT_NORMAL_RADIUS = 1; // was 6
    const static unsigned int WAYPOINT_HIGHLIGHT_RADIUS = 2; // was 8

    const static double DEG_TO_RAD = M_PI/180.0;
    const static float INVALID_DEGREE_VALUE = 361.0;
 
    const static unsigned int SERVICE_THREAD_IDLE_MSEC = 10;
    const static double FOV_TO_HORIZ_MOVE_RATIO = 1;
    const static double PRUNE_PATH_STRAIGHT_LINE_MIN_COSINE_ANGLE_DEG = 170;
    const static double TURNING_ANGLE_MAX_DEG = 170;

    const static double DEFAULT_UAV_FOV_DEG = 10;//2; //was 43
    const static unsigned int
        DEFAULT_UAV_VIEW_ALL_WAYPOINTS_COVERAGE_ALTITUDE_METERS = 200;
    const static unsigned int DEFAULT_UAV_COVERAGE_ALTITUDE_METERS = 120;
    const static unsigned int DEFAULT_UAV_SPIRAL_ALTITUDE_METERS = 50;
    const static unsigned int DEFAULT_UAV_AIRSPEED_MPS = 15;
    const static unsigned int DEFAULT_UAV_LOITER_RADIUS_METERS = 60;
};
#endif
