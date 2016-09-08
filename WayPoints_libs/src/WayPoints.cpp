//#define DEBUG
#include "WayPoints.h"

/******************************************************************************
 *
 * @maintainer: Kelly Benson
 * @email: bensonke@email.sc.edu
 * @date: May 16, 2015
 *
 * This file is for the WayPoints calculation and work behind that. 
 * There are several different constructors provided to give the user
 * options as to what they want to do. This might be amended at a later 
 * date depending on what is needed and how correct providing these options
 * are to the programs purpose itself. 
 * 
**/


/*************************************************************************
 * Constructor
 *
**/
WayPoints::WayPoints() {};

/*************************************************************************
 * Alternate Constructor - needed for when you pass in BCD info
 *
**/
WayPoints::WayPoints(RegionData& data, ReebGraph& graph,
                std::list<Edge>& eulerCycle, vector<Point2D>& wayPoints)
{

/*
    if (graph.empty() == true) 
    {
        _setupCompleted = false;
        std::cout << "Graph is empty. Did you build the BCD?";
        return;
    }

    if (eulerCycle.size() <= 0) 
    {
        _setupCompleted = false;
        throw string("Offline setup failed: Euler cycle has no entries!");
    }

    _setupCompleted = true;
*/

#ifdef DEBUG    
    cerr << "Start Genpath\n";
#endif 

    boost::optional<std::list<Edge> > tour(eulerCycle);
    genPath_All(data, graph, wayPoints, desiredAltitudeM, -1,
            ReebGraph::nullEdge(), tour);

#ifdef DEBUG
    cerr << "End\n";
#endif 
};


/************************************************************************
 * Destructor
 *
**/
WayPoints::~WayPoints() {};

/*************************************************************************
 * Function 'genPath_SeedSpreader()'
 *
 * Returns:
 *   None 
**/
pair<Vertex, bool> WayPoints::genPath_SeedSpreader(ReebGraph& g,
        std::vector<Point2D>& path, Edge currEdge, Vertex startingVertex,
        Edge nextEdge, bool upDir, double coverWidth, bool completeCover)
    throw (const std::string&)
{

    // Sanity check
    if (currEdge != ReebGraph::nullEdge())
    {
        Vertex endVertexA, endVertexB, endVertexC, endVertexD;
        tie(endVertexA, endVertexB) = g.getEndNodes(currEdge);

        if (nextEdge != ReebGraph::nullEdge())
        {
            tie(endVertexC, endVertexD) = g.getEndNodes(nextEdge);

            if ((endVertexA == endVertexC && endVertexB == endVertexD) ||
                    (endVertexA == endVertexD && endVertexB == endVertexC)) 
            {
                // Both edges share the same end nodes
                if (endVertexA != startingVertex
                        && endVertexB != startingVertex)
                {
                    throw string("getPath_SeedSpreader() - startingVertex"
                            " != endNodes(currEdge)");
                }
            }
            // 2 edges share only 1 common vertex
            else if ((endVertexA == endVertexC && startingVertex != endVertexB)
                    || (endVertexA == endVertexD
                        && startingVertex != endVertexB)
                    || (endVertexB == endVertexC
                        && startingVertex != endVertexA)
                    || (endVertexB == endVertexD
                        && startingVertex != endVertexA))
            {
                throw string("getPath_SeedSpreader() - startingVertex"
                        " != other end node of currEdge)");
            }
        }
        else if (endVertexA != startingVertex && endVertexB != startingVertex)
        {
            throw string("getPath_SeedSpreader() - startingVertex"
                    " != endNodes(currEdge)");
        }
    }
    else
    {
        throw string("getPath_SeedSpreader() - currEdge is NULL");
    }

    Vertex v1,v2;
    tie(v1,v2) = g.getEndNodes(currEdge);
    Vertex endingVertex = (v1 == startingVertex ? v2 : v1);
    ReebVertex _startingVertex = g.getVProp(startingVertex);
    ReebVertex _endingVertex = g.getVProp(endingVertex);
    ReebEdge _currEdge = g.getEProp(currEdge);
    int cellWidth = _currEdge.bottomBoundary.size();
    Point2D startingPos = Point2D(_startingVertex.x,
            (_startingVertex.y1 + _startingVertex.y2) / 2);
    Point2D endingPos = Point2D(_endingVertex.x,
            (_endingVertex.y1 + _endingVertex.y2) / 2);
    bool goRight = (startingPos.xcoord() <= endingPos.xcoord() ? true : false);
    bool preferredEndingDirection;
    bool preferredUp;

    int _size;
    _size = _currEdge.topBoundary.size();

//  double _MidY = (goRight)?
//          (_currEdge.topBoundary[0].ycoord()+_currEdge.bottomBoundary[0].ycoord())/2:
//          (_currEdge.topBoundary[_size-1].ycoord()+_currEdge.bottomBoundary[_size-1].ycoord())/2;
//  upDir = (startingPos.ycoord()>=_MidY ? true:false);

    if(nextEdge != ReebGraph::nullEdge())
    {
        ReebEdge _nextEdge = g.getEProp(nextEdge);
        tie(v1,v2) = g.getEndNodes(nextEdge);
        ReebVertex _nextEdgeEndingVertex = g.getVProp(v1 == endingVertex
                ? v2 : v1);
        bool _nextEdgeGoRight = (endingPos.xcoord() <= _nextEdgeEndingVertex.x
                ? true : false);
        _size = _nextEdge.topBoundary.size();
        double _nextEdgeMidY = (_nextEdgeGoRight)
            ? (_nextEdge.topBoundary[0].ycoord()
                    + _nextEdge.bottomBoundary[0].ycoord()) / 2
            : (_nextEdge.topBoundary[_size-1].ycoord()
                    + _nextEdge.bottomBoundary[_size-1].ycoord()) / 2;
        _size = _currEdge.topBoundary.size();
        double _currEdgeMidY = (!goRight)
            ? (_currEdge.topBoundary[0].ycoord()
                    + _currEdge.bottomBoundary[0].ycoord()) / 2
            : (_currEdge.topBoundary[_size-1].ycoord()
                    + _currEdge.bottomBoundary[_size-1].ycoord()) / 2;

        preferredUp = (_currEdgeMidY > _nextEdgeMidY ? true : false);

        //std::cout<<_currEdgeMidY <<" vs "<< _nextEdgeMidY <<endl;

        preferredEndingDirection = (abs(_currEdgeMidY - _nextEdgeMidY) > 10
                ? true : false);
    }
    else
    {
        preferredEndingDirection = false;
    }

    //calculate steps
    int numStep;
    if(preferredEndingDirection)
    {
        bool isOdd = (upDir && preferredUp) || (!upDir && !preferredUp);
        int originalNumStep = ceil(1.0*cellWidth/coverWidth + 1);
        bool getOdd = (originalNumStep%2);
        numStep = ((isOdd && getOdd) || (!isOdd && !getOdd))
            ? originalNumStep : originalNumStep+1;
    }
    else
    {
        numStep = ceil(1.0*cellWidth/coverWidth + 1);
    }

    bool _upDir = upDir;
    for (int i = 0; i < numStep; ++i, _upDir = !_upDir)
    {
        int index = (numStep != 1
                ? round((cellWidth-1)*i*1.0 / (numStep-1)) : 0);
        if (!goRight)
        {
            index = cellWidth - index - 1;
        }

#ifdef DEBUG
        std::cerr << i << ": " << &(_currEdge.bottomBoundary[index])
            << std::endl;
#endif

        if (_currEdge.bottomBoundary.size() == 0
                || _currEdge.topBoundary.size() == 0) {
            //continue;
        }

        if (i == 0)
        {
            if (_upDir)
            {
                if (completeCover)
                {
                    path.push_back(_currEdge.bottomBoundary[index]);
                }
                path.push_back(_currEdge.topBoundary[index]);
            }
            else
            {
                if (completeCover)
                {
                    path.push_back(_currEdge.topBoundary[index]);
                }
                path.push_back(_currEdge.bottomBoundary[index]);
            }
        }
        else if (i == numStep-1)
        {
            if (_upDir)
            {
                path.push_back(_currEdge.bottomBoundary[index]);

                if (completeCover)
                {
                    path.push_back(_currEdge.topBoundary[index]);
                }
                else
                {
                    path.push_back((_currEdge.topBoundary[index]
                                + _currEdge.bottomBoundary[index]) / 2);
                }
            }
            else
            {
                path.push_back(_currEdge.topBoundary[index]);
                if (completeCover)
                {
                    path.push_back(_currEdge.bottomBoundary[index]);
                }
                else
                {
                    path.push_back((_currEdge.topBoundary[index]
                                + _currEdge.bottomBoundary[index]) / 2);
                }
            }
        }
        else
        {
            if (_upDir)
            {
                path.push_back(_currEdge.bottomBoundary[index]);
                path.push_back(_currEdge.topBoundary[index]);
            }
            else
            {
                path.push_back(_currEdge.topBoundary[index]);
                path.push_back(_currEdge.bottomBoundary[index]);
            }
        }
    }

    return make_pair(endingVertex, !_upDir);
};


/*************************************************************************
 * Function 'convertTourToReebGraph()'
 *
 * Converts a Euler tour to a Reeb Graph usable for genPath_All.
 *
 * Arguments:
 *  tour -          Source Euler tour.
 *  m_graph -       Graph with all the tours from KCPP.
 *  dest -          Reeb Graph to place the edges into.
 *
 * Returns:
 *  None
**/
void WayPoints::convertTourToReebGraph(std::list<ReebEdge> &tour,
        ReebGraph &m_graph, ReebGraph &dest) {

#ifdef DEBUG
    std::cerr << "Converting tour to Reeb Graph\n";
#endif 

    std::list<ReebEdge>::const_iterator it;
    ReebEdge edge, *addedReebEdge;
    Edge addedEdge;
    Vertex v_first, v_second, nvf, nvs;
    bool first = true;
    ReebVertex rvf, rvs, rvs_prev;
    for (it = tour.begin(); it != tour.end(); ++it) {
        edge = *it;
        tie(v_first, v_second) = m_graph.getEndNodes(edge.Eid);
        rvf = m_graph.getVProp(v_first);
        rvs = m_graph.getVProp(v_second);

#ifdef DEBUG
        cerr << "Edge: (Vertex: (" << rvf.x << "," << rvf.y1 << "," << rvf.y2
            << "," << rvf.color << "," << "), Vertex (" << rvs.x << ","
            << rvs.y1 << "," << rvs.y2 << "," << rvs.color << ","
            << "), color: " << edge.color << ")\n";
#endif 

        nvs = dest.addVertex(rvs.x, rvs.y1, rvs.y2, rvs.color);
        nvf = dest.addVertex(rvf.x, rvf.y1, rvf.y2, rvf.color);
#ifndef AFRL_CONTIN
        if (first) {
            addedEdge = dest.addEdge(nvs, nvf, edge.color);
            tie(nvf, nvs) = dest.getEndNodes(addedEdge);
            rvs = dest.getVProp(nvs);
            rvs_prev = rvs;
            first = false;
        } else if (rvs_prev.x == rvf.x
                    && rvs_prev.y1 == rvf.y1 && rvs_prev.y2 == rvf.y2) {
            addedEdge = dest.addEdge(nvf, nvs, edge.color);
            rvs_prev = rvs;
        } else {
            addedEdge = dest.addEdge(nvs, nvf, edge.color);
            rvs_prev = rvf;
        }
#else
        if (first) {
            addedEdge = dest.addEdge(nvs, nvf, edge.color);
            first = false;
        } else {
            addedEdge = dest.addEdge(nvf, nvs, edge.color);
        }
#endif
        addedReebEdge = &(dest.getEProp(addedEdge));
        addedReebEdge->Eid = edge.Eid;
        addedReebEdge->topBoundary = edge.topBoundary;
        addedReebEdge->bottomBoundary = edge.bottomBoundary;
    }
}


/*************************************************************************
 * Function 'genPath_All()'
 *
 * Returns:
 *   None 
**/
void WayPoints::genPath_All(RegionData& data, ReebGraph& graph,
        std::vector<Point2D>& buffer, double altitude, int firstVertexID,
        Edge firstEdge, boost::optional<std::list<Edge> > tour)
    throw (const std::string&) 
{
    bool pickClosest = !tour;
    std::list<Edge>::iterator itTour, itTourEnd;

    Point2D _lastPoint;
    Out_Edge_Iter oi, oi_end;
    Vertex _currVertex;

    if (firstVertexID == -1)
    {
        _currVertex = graph.getFirstVertex();
    }
    else
    {
        _currVertex = graph.getVertex(firstVertexID);
    }

    if (_currVertex == ReebGraph::nullVertex())
    {
        //throw string("genPath_All() - First selected vertex is null");
        return;
    }

    tie(oi, oi_end) = graph.getEdges(_currVertex);
    if (oi == oi_end) 
    {
        //throw string("genPath_All() - First vertex does not have any edges");
        return;
    }


    Edge _currEdge;
    if (pickClosest) {
        _currEdge = (firstEdge == ReebGraph::nullEdge() ? *oi : firstEdge);
    } else {
        itTour = tour->begin();
        itTourEnd = tour->end();
        if (itTour == itTourEnd) {
            return;
        }
        _currEdge = (firstEdge==ReebGraph::nullEdge() ? *itTour++ : firstEdge);
    }
    Edge _closestEdge = ReebGraph::nullEdge();
    Vertex _closestVertex = ReebGraph::nullVertex();
    unsigned int _numCellsCovered = 0;
    bool _upDir = true;
 
    pair<Vertex, bool> v_bool_pair;

    double _horizMovePixel = tan(DEFAULT_UAV_FOV_DEG/2 * DEG_TO_RAD)
        * altitude/2 * 3 * FOV_TO_HORIZ_MOVE_RATIO/data.xRes;
    Edge_Iter ei, ei_end;
    Vertex _v1, _v2;
    double minDistSqrd, currDistSqrd, _c1, _c2;

    graph.resetAllEdgeColor();

    while (_numCellsCovered < graph.numEdges())
    {
        // Generate waypoints for currently selected Reeb graph edge
        v_bool_pair = WayPoints::genPath_SeedSpreader(graph, buffer,
                _currEdge, _currVertex, ReebGraph::nullEdge(), _upDir,
                _horizMovePixel, true);

        // Update variables
        _currVertex = v_bool_pair.first;
        _upDir = v_bool_pair.second;
        graph.getEProp(_currEdge).color = 1;
        _numCellsCovered++;

        if (buffer.size() == 0) {
            //continue;
        }

        // Choose the next unvisited edge closest to the current UAV pos.
        _lastPoint = buffer.back();
        _closestEdge = ReebGraph::nullEdge();
        _closestVertex = ReebGraph::nullVertex();
        minDistSqrd = numeric_limits<double>::max();

        if (pickClosest) {

            // OLD EDGE SELECTION

            for(tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ei++)
            {
                // Only want non-visited edges
                if (graph.getEProp(*ei).color != 0) 
                {
                    continue;
                }

                tie(_v1,_v2) = graph.getEndNodes(*ei);

                if(graph.getVProp(_v1).x > graph.getVProp(_v2).x)
                {
                    swap(_v1,_v2);
                }

                // Scan for closest point on bottom boundary of edge
                _c1 = _lastPoint.sdist(graph.getEProp(*ei).topBoundary.front());
                _c2 = _lastPoint.sdist(graph.getEProp(*ei).topBoundary.back());

                currDistSqrd = min(_c1,_c2);

                if(currDistSqrd < minDistSqrd)
                {
                    _closestEdge = *ei;
                    _closestVertex = (_c1<_c2? _v1:_v2);
                    minDistSqrd = currDistSqrd;
                    _upDir = false;
                }

                // Scan for closest point on top boundary of edge
                _c1 = _lastPoint.sdist(graph.getEProp(*ei).bottomBoundary.front());
                _c2 = _lastPoint.sdist(graph.getEProp(*ei).bottomBoundary.back());

                currDistSqrd = min(_c1,_c2);

                if(currDistSqrd < minDistSqrd) 
                {
                    _closestEdge = *ei;
                    _closestVertex = (_c1 < _c2 ? _v1 : _v2);
                    minDistSqrd = currDistSqrd;
                    _upDir = true;
                }

                // Sanity check
                if (_closestEdge == ReebGraph::nullEdge()) 
                {
                    throw string("genPath_All() - Closest edge in greedy algorithm"
                            " found as a null edge");
                }

                // Update parameters
                _currEdge = _closestEdge;
                _currVertex = _closestVertex;
            }

        } else {

            // NEW EDGE SELECTION

            _currEdge = *itTour;
            tie(_v1,_v2) = graph.getEndNodes(_currEdge);
            _currVertex = _v1;
            _upDir = v_bool_pair.second;
            ++itTour;

        }

    }

    graph.resetAllEdgeColor();

    // Prune nearby points in path
    prunePathPoints(buffer);

    //copies the points
    setWayPoints(buffer);

/* 
    vector<Point2D>::iterator iter;
    for(iter = buffer.begin(); iter != buffer.end(); ++iter)
    {
        Point2D point = (*iter);
        points.push_back(point);
    }
*/

};


/*************************************************************************
 * Function 'setWayPoints()'
 *
 * Sets the points of the current way points object. 
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   vector<Point2D> - vector of way points to set points to
 *
**/
void WayPoints::setWayPoints(vector<Point2D> givenPoints)
{
    vector<Point2D>::iterator iter;
    for(iter = givenPoints.begin(); iter != givenPoints.end(); ++iter)
    {
        Point2D point = (*iter);
        points.push_back(point);
    }

#ifdef DEBUG    
    cout << "Start set\n";
    printWayPoints();
    cout << "End set\n";
#endif
}


/*************************************************************************
 * Function 'getWayPoints()'
 *
 * Returns the way points generated by the genPath_All() method. 
 *
 * Returns:
 *   vector<Point2D> - vector of way points to travel through
 *
 * Parameters:
 *   None
 *
**/
vector<Point2D> WayPoints::getWayPoints()
{
    vector<Point2D> destinationPoints;

    vector<Point2D>::iterator iter;
    for(iter = points.begin(); iter != points.end(); ++iter)
    {
        Point2D point = (*iter);
        destinationPoints.push_back(point);
    }

#ifdef DEBUG    
    cout << "Start get\n";
    printWayPoints();
    cout << "End get\n";
#endif

    return destinationPoints;
}


/*************************************************************************
 * Function 'printWayPoints()'
 *
 * Prints the points of the current way points object. 
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   vector<Point2D> - vector of way points to set points to
 *
**/
void WayPoints::printWayPoints()
{
    vector<Point2D>::iterator iter;
    for(iter = points.begin(); iter != points.end(); ++iter)
    {
        Point2D point = (*iter);
        cout << (*iter) << " ";
    }
}


/*************************************************************************
 * Function 'printWayPoints()'
 *
 * Prints the points of the current way points object. 
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   vector<Point2D> - vector of way points to set points to
 *
**/
void WayPoints::printWayPoints(vector<Point2D> givenPoints)
{
    vector<Point2D>::iterator iter;
    for(iter = givenPoints.begin(); iter != givenPoints.end(); ++iter)
    {
        Point2D point = (*iter);
        cout << (*iter) << " ";
    }
}


/*************************************************************************
 * Function 'prunePathPoints()'
 *
 * Returns:
 *   None 
 *
**/
void WayPoints::prunePathPoints(std::vector<Point2D>& buffer) 
{
    if (buffer.size() >= 4)
    {
        // Remove Go-Return-Go features
        // Note: vector iterators are random-access
        std::vector<Point2D>::iterator frameBack = buffer.begin() + 3;
        while (frameBack != buffer.end())
        {
            if ((frameBack-3)->distance(*(frameBack-1)) <= 8
                    && (frameBack-2)->distance(*frameBack) <= 8)
            {
                frameBack = buffer.erase(frameBack-1, frameBack+1);
                --frameBack;
            }
            else if (frameBack != buffer.end())
            {
                ++frameBack;
            }
        }
    }

    // Boundary condition: need at least 2 points in order to start dist-pruning
    if (buffer.size() <= 1)
    {
        return;
    }

    // Check if consecutive points are sufficiently close
    std::vector<Point2D>::iterator prevIt, currIt;
    prevIt = buffer.begin();
    currIt = prevIt + 1;
    while (currIt != buffer.end())
    {
        // Proximity is defined as within +/- 2 pixels in both x and y axes
        while (currIt != buffer.end()
                && prevIt->distance(*currIt) <= 2*sqrt(2))
        {
            currIt = buffer.erase(currIt);
        }

        if (currIt != buffer.end())
        {
            prevIt = currIt;
            currIt++;
        }
    }

    // Boundary condition: need at least 3 points in order to start angle-pruning
    if (buffer.size() <= 2)
    {
        return;
    }

    prevIt = buffer.begin();
    currIt = prevIt + 1;
    while (currIt + 1 != buffer.end())
    {
        while (currIt + 1 != buffer.end()
                && currIt->cosineAngle(*prevIt, *(currIt + 1))
                > PRUNE_PATH_STRAIGHT_LINE_MIN_COSINE_ANGLE_DEG) 
        {
            currIt = buffer.erase(currIt);
        }

        if (currIt + 1 != buffer.end()) 
        {
            prevIt = currIt;
            currIt++;
        }
    }
};

/*************************************************************************
 * Function 'computeTotalPathLength()'
 *
 * Returns:
 *   None 
 *
**/
double WayPoints::computeTotalPathLength(RegionData& data,
        const std::vector<Point2D>& buffer)
{
    double sum = 0;
    for(unsigned int i = 0; i < buffer.size() - 1; ++i)
    {
        sum += (buffer[i]).distance(buffer[i+1]);
    }

    return sum * data.xRes; // NOTE: assuming that data.xRes == data.yRes
};

/*************************************************************************
 * Function 'computeAllTotalPathLengths()'
 *
 * Returns:
 *   None 
 *
**/
void WayPoints::computeAllTotalPathLengths(RegionData& data, ReebGraph& graph,
        std::vector<double>& resultBuffer, double altitudeM)
    throw (const std::string&) 
{
    std::vector<Point2D> pointBuffer;

    resultBuffer.clear();
    for (unsigned int i = 0; i < graph.numVertices(); i++)
    {
        Out_Edge_Iter oi, oi_end;
        Vertex _currVertex = graph.getVertex(i);
        if (_currVertex == ReebGraph::nullVertex())
        {
            continue;
        }

        tie(oi, oi_end) = graph.getEdges(_currVertex);
        if (oi == oi_end) 
        {
            continue;
        }

        for(; oi != oi_end; ++oi)
        {
            pointBuffer.clear();
            genPath_All(data, graph, pointBuffer, altitudeM, i, *oi);
            resultBuffer.push_back(computeTotalPathLength(data, pointBuffer));
        }
    }
};

/*************************************************************************
 * Function 'computeNumberOfTurns()'
 *
 * Returns:
 *   None 
 *
**/
unsigned int WayPoints::computeNumberOfTurns(
        const std::vector<Point2D>& buffer)
{
    unsigned int count = 0;

    // Boundary condition: need at least 3 points in order to start count angles
    if (buffer.size() <= 2) 
    {
        return 0;
    }

    for (unsigned int currID = 1; currID + 1 < buffer.size(); currID++) 
    {
        if (buffer[currID].cosineAngle(buffer[currID - 1], buffer[currID + 1])
                <= TURNING_ANGLE_MAX_DEG) 
        {
            count++;
        }
    }

    return count;
};

/*************************************************************************
 * Function 'analyzeRotationEffects()'
 *
 * Hasn't been modified. Should work the same.
 *
 * Returns:
 *   None
 *
**/
/*
unsigned int WayPoints::analyzeRotationEffects(\
        std::vector<double>& anglesBuffer, \
        std::vector<double>& pathLengthsBuffer, \
        std::vector<unsigned int>& numTurnsBuffer, \
        double angleIncrDeg, double altitudeM) 
{
    std::vector<Point2D> pointBuffer;
    unsigned int count = 0;
    anglesBuffer.clear();
    pathLengthsBuffer.clear();
    numTurnsBuffer.clear();

    // Sanity check: ensure that angle increment is non-zero (prevent inf. loop)
    if (angleIncrDeg == 0) 
    {
        return 0;
    }

    for (double angle = 0; angle < 180; angle += angleIncrDeg) 
    {
        ostringstream ss;
        pointBuffer.clear();

        try 
        {
            getRegionData().rotateAllData(angle);
            offlineSetup();
            genPath_All(pointBuffer, altitudeM);
            anglesBuffer.push_back(angle);
            pathLengthsBuffer.push_back(computeTotalPathLength(pointBuffer));
            numTurnsBuffer.push_back(computeNumberOfTurns(pointBuffer));
            count++;
            ss << "Analyzed world @ " << angle << " deg\r\n";
        }
 
        catch (const std::string& err) 
        {
            ss << "Error in analysis @ " << angle << " deg: " << err << "\r\n";
        }

        std::cout << ss.str() << endl;
    }

    return count;
};*/

/*************************************************************************
 * Function 'findClosestGraphPos()'
 *
 * Hasn't been modified. Should work the same.
 *
 * Returns:
 *   None 
 *
**/
pair<Vertex, Edge> WayPoints::findClosestGraphPos(ReebGraph& g,
        double xPixel, double yPixel) throw (const std::string&)
{
    // Sanity check
    if (g.numVertices() <= 0 || g.numEdges() <= 0)
    {
        throw string("findClosestGraphPos() - Reeb graph is empty!");
    }

    Vertex closestVertex = ReebGraph::nullVertex();
    Edge closestEdge = ReebGraph::nullEdge();
    ReebVertex* currVProp = NULL;
    ReebEdge* currEProp = NULL;

    // Iterate over all vertices to find the closest
    double closestVertexDistSqrd = numeric_limits<double>::max();
    Vertex_Iter vi, vi_end;

    for (tie(vi, vi_end) = g.getVertices(); vi != vi_end; vi++)
    {
        currVProp = &(g.getVProp(*vi));

        // Compute midpoint of vertex to current point
        if (updateMinDist(currVProp->x - xPixel,
                    (currVProp->y1 + currVProp->y2) / 2 - yPixel,
                    closestVertexDistSqrd)) 
        {
            closestVertex = *vi;
        }
    }

    // Sanity check
    if (closestVertex == ReebGraph::nullVertex()
            || g.degree(closestVertex) <= 0)
    {
        throw string("findClosestGraphPos() - closest vertex is null or alone!");
    }

    // Iterate over all edges connected to the closest vertex to find the closest
    double closestEdgeDistSqrd = numeric_limits<double>::max();
    unsigned int numPoints;
    double midX, midY;
    Out_Edge_Iter ei, ei_end;

    for (tie(ei, ei_end) = g.getEdges(closestVertex); ei != ei_end; ei++)
    {
        currEProp = &(g.getEProp(*ei));
        numPoints = currEProp->topBoundary.size();

        // Compute midpoint of edge
        if (numPoints == 0)
        {
            continue;
        }
        else if (numPoints == 1) 
        {
            midX = currEProp->topBoundary[0].xcoord();
            midY = (currEProp->topBoundary[0].ycoord()
                    + currEProp->bottomBoundary[0].ycoord()) / 2;
        }
        else
        {
            midX = currEProp->topBoundary[numPoints/2-1].xcoord();
            midY = (currEProp->topBoundary[numPoints/2-1].ycoord()
                    + currEProp->bottomBoundary[numPoints/2-1].ycoord()) / 2;
        }

        // Compute midpoint of edge to current point
        if (updateMinDist(midX - xPixel, midY - yPixel, closestEdgeDistSqrd))
        {
            closestEdge = *ei;
        }
    }

    // Sanity check
    if (closestEdge == ReebGraph::nullEdge())
    {
        throw string("findClosestGraphPos() - closest edge is null!");
    }

    return make_pair(closestVertex, closestEdge);
};

/*************************************************************************
 * Function 'pruneDuplicates()'
 *
 * Hasn't been modified. Should work the same.
 *
 * Returns:
 *   None 
 *
**/
void WayPoints::pruneDuplicates(std::vector<Point2D>& waypoints) 
{
    for (int i = 0; i < (int) waypoints.size() - 1;)
    {
        if (waypoints[i] == waypoints[i+1])
        {
            waypoints.erase(waypoints.begin() + i);
        }
        // Only increment if not deleting next entry
        else
        {
            i++;
        }
    }
};


/*************************************************************************
 * Function 'updateMinDist()'
 *
 * Returns:
 *   None 
 *
**/
bool WayPoints::updateMinDist(double newDx, double newDy, double& prevDistSqrd)
{
    double newDistSqrd = newDx*newDx + newDy*newDy;
    if (newDistSqrd < prevDistSqrd)
    {
        prevDistSqrd = newDistSqrd;
        return true;
    }
    else
    {
        return false;
    }
};


//The below functions have not been tested and since they were not
//  originally commented on it is ambiguous as to their purpose. 

/*************************************************************************
 * Function 'startOnlineLoop()'
 *
 * Warning: DO NOT turn on onlineLoopAlive flag here, let onlineLoop() 
 * do it
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void WayPoints::startOnlineLoop() 
{
    // As long as onlineLoop() is not running (its main parts)
    if (!_onlineLoopAlive)
    {
        serviceQueueMutex.lock();
        serviceQueue.push_back(SERVICE_ONLINE_LOOP);
        serviceQueueMutex.unlock();
    }
};


/*************************************************************************
 * Function 'stopOnlineLoop()'
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void WayPoints::stopOnlineLoop()
{
    // Remove all instances of online loop service keys and turn off alive flag
    serviceQueueMutex.lock();

    for (list<enum ServiceType>::iterator it = serviceQueue.begin();
            it != serviceQueue.end(); it++)
    {
        if (*it == SERVICE_ONLINE_LOOP)
        {
            it = serviceQueue.erase(it);
        }
    }

    _onlineLoopAlive = false;
    serviceQueueMutex.unlock();
};


/*************************************************************************
 * Function 'startInvestigate()'
 *
 * Warning: DO NOT turn on investigateAlive flag here, let investigate() 
 * do it
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void WayPoints::startInvestigate(double centerLat, double centerLon) 
{

    // As long as investigate() is not running (its main parts)
    if (!_investigateAlive)
    {
        bool resumeOnlineLoop = _onlineLoopAlive;
        stopOnlineLoop();
        serviceQueueMutex.lock();
        investigateCenterGCS = std::make_pair(centerLat, centerLon);
        serviceQueue.push_back(SERVICE_INVESTIGATE);

        if (resumeOnlineLoop)
        {
            serviceQueue.push_back(SERVICE_ONLINE_LOOP);
        }

        serviceQueueMutex.unlock();
    }
};


/*************************************************************************
 * Function 'stopInvestigate()'
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void WayPoints::stopInvestigate()
{
    // Remove all instances of investigate service keys and turn off alive flag
    serviceQueueMutex.lock();
    for (list<enum ServiceType>::iterator it = serviceQueue.begin();
            it != serviceQueue.end(); it++)
    {
        if (*it == SERVICE_INVESTIGATE)
        {
            it = serviceQueue.erase(it);
        }
    }

    _investigateAlive = false;
    serviceQueueMutex.unlock();
};


/*************************************************************************
 * Function 'viewWayPoints(QString fileName)'
 *
 * Draws the waypoints to an image file. 
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void WayPoints::viewWaypoints(QString fileName, RegionData& data,
        ReebGraph& graph, std::list<Edge>& eulerCycle,
        vector<Point2D>& wayPoints)
{
    if(graph.empty() == true)
    {
        std::cout << "graph is empty";
        return;
    }

    else if(eulerCycle.empty() == true)
    {
        std::cout << "euler is empty";
        return;
    }

    DrawImage placeHolder(graph, data, eulerCycle, wayPoints);
    placeHolder.drawWaypoints(wayPoints, 0, 0);
    placeHolder.saveImageBuffer(fileName);
};























/*************************************************************************
 * Auxiliary functions
**/

/*************************************************************************
 * Function 'isSetupCompleted()'
 *
 * Returns a boolean for if the setup has been run and is complete.  
 *
 * Returns:
 *   bool - setup complete
 *
 * Parameters:
 *   None
 *
**/
/*
bool isSetupCompleted()
{ 
    return _setupCompleted;
};*/

/*************************************************************************
 * Function 'isRunningOnlineLoop()'
 *
 * Returns a boolean for if the loop is still running. 
 *
 * Returns:
 *   bool - loop alive
 *
 * Parameters:
 *   None
 *
**/
/*
bool isRunningOnlineLoop()
{ 
    return _onlineLoopAlive;
};*/

/*************************************************************************
 * Function 'isRunningInvestigate()'
 *
 * Returns a boolean for if the investigation method is running
 *
 * Returns:
 *   bool - investigation running
 *
 * Parameters:
 *   None
 *
**/
/*
bool isRunningInvestigate()
{ 
    return _investigateAlive;
};*/

/*************************************************************************
 * Function 'resetOnlineLoopState()'
 *
 * Resets and cleans up variables
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
/*
void resetOnlineLoopState() 
{
    numCellsCovered = 0;
    currVertex = ReebGraph::nullVertex();
    currEdge = ReebGraph::nullEdge();
    upDir = true;
    resumedWaypointIndex = 0;
    resumedCommandIndex = 1;
    resumeOnlineLoop = false;
};
*/

/*************************************************************************
 * Function 'stopServices()'
 *
 * Stops all running services
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
/*
void stopServices() 
{
    serviceQueueMutex.lock();
    serviceQueue.clear();
    serviceQueueMutex.unlock();
    _onlineLoopAlive = false;
    _investigateAlive = false;
};*/

/*************************************************************************
 * Function 'runServiceThread()'
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/

/*
void WayPoints::runServiceThread() throw (const std::string&) 
{
    enum ServiceType currService = SERVICE_NONE;
    std::list<enum ServiceType>::iterator it;
    while (_serviceThreadAlive) 
    {
        // Obtain next service key from queue
        serviceQueueMutex.lock();
        it = serviceQueue.begin();
        if (it != serviceQueue.end()) 
        {
            currService = *it;
        }
 
        else 
        {
            currService = SERVICE_NONE;
        }

        serviceQueueMutex.unlock();

        // Process service key
        switch (currService) 
        {
            case SERVICE_ONLINE_LOOP:
                try 
                {
                    onlineLoop();
                }
                catch (const std::string& err) 
                {
                    std::cerr << "ERROR > onlineLoop() - " << err << std::endl;
                }
                _onlineLoopAlive = false;
                break;

        case SERVICE_INVESTIGATE:
            try 
            {
                investigate();
            }
            catch (const std::string& err) 
            {
                std::cerr << "ERROR > investigate() - " << err << std::endl;
            }
            _investigateAlive = false;
            break;

        case SERVICE_NONE:

        default:
            boost::this_thread::sleep(boost::posix_time::milliseconds( \
                    SERVICE_THREAD_IDLE_MSEC));
            break;
        }

        // Remove processed service key from queue
        serviceQueueMutex.lock();
        it = serviceQueue.begin();
        if (it != serviceQueue.end() && *it == currService)
        {
            serviceQueue.erase(it);
        }
        serviceQueueMutex.unlock();
    }
};
*/
