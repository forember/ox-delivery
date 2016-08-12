#include "ReebGraph.h"

/*****************************************************************************
 * @Modified date: May 31, 2015
 *
 * Reebgraph class. 
 *
**/


// Declares the singleton null vertex & edge pointers
const Vertex ReebGraph::Vnull = Vertex();
const Edge ReebGraph::Enull = Edge();


/*************************************************************************
 * Function 'equals(ReebGraph Graph)'
 *
 * Method that checks the equality between two ReebGraph objects
 *
 * Returns:
 *   bool
 *
 * Parameters:
 *   None
 *
**/
bool ReebGraph::equals(ReebGraph Graph)
{
    if ((Epointers.size() != Graph.Epointers.size())
            || (Vpointers.size() != Graph.Vpointers.size()))
    {
        return false;
    }

    else if (Vid_free != Graph.Vid_free)
    {
        return false;
    }

    else if (Eid_free != Graph.Eid_free)
    {
        return false;
    }

    else if(numVertices() != Graph.numVertices())
    {
        return false;
    }

    else if(numEdges() != Graph.numEdges())
    {
        return false;
    }

    for(int i = 0; i < Epointers.size(); ++i)
    {
        if(Epointers.at(i) != Graph.Epointers.at(i))
        {
            return false;
        }
    }

    for(int i = 0; i < Vpointers.size(); ++i)
    {
        if(Vpointers.at(i) != Graph.Vpointers.at(i))
        {
            return false;
        }
    }

    return true;
};


/*************************************************************************
 * Operator overload: outstream operator
 *
 * Overloading outstream operator
 *   vertex object v
 *   vertex coordinates x and y1/y2.
 *
 * BGL - boost graph library    
 *   have to use the x,y1,y2,vid,and color conventions
 *   for the constructor
 *
 * printing out the reeb vertex parameters
 *
**/
ostream& operator<< (ostream& out, ReebVertex& v) 
{
#ifdef SHORT_PRINT_OUT
    out << "V" << v.Vid;
#else
    out << "VERTEX [Vid=" << v.Vid << " x=" << v.x << " y1=" << v.y1
        << " y2=" << v.y2 << " color=" << v.color << "]";
#endif
    return out;
};
 

/*************************************************************************
 * Operator overload: outstream operator
 *
 * Method that prints out the reeb edge parameters
 *
 * Returns:
 *   ostream& - contains information about the reeb edges
 *
 * Parameters:
 *   ostream& out
 *   ReebEdge& e
 *
**/
ostream& operator<< (ostream& out, ReebEdge& e) 
{
#ifdef SHORT_PRINT_OUT
    out << "E" << e.Eid;
#else
    out << "EDGE [Eid=" << e.Eid << " color=" << e.color << " cost="
        << e.cost << " area=" << e.area << " topBoundary.size()="
        << e.topBoundary.size() << " bottomBoundary.size()="
        << e.bottomBoundary.size() << "]";
#endif
    return out;
};


/*************************************************************************
 * Operator overload: outstream operator
 *
 * Method that prints out the reeb graph parameters
 *
 * Returns:
 *      ostream& - outstream containing information about the reeb edges
 *          in this graph
 *
 * Parameters:
 *   ostream& out
 *   ReebEdge& e
 *
**/
ostream& operator<< (ostream& out, ReebGraph& g)
{
    // Declares some local variables
    ReebVertex vprop;
    ReebEdge eprop;
    Vertex currVertex;
    Edge currEdge;
    Vertex_Iter vi, vi_end;
    Edge_Iter ei, ei_end;
    Out_Edge_Iter oi, oi_end;
    Vertex v_first, v_second;
    unsigned int currDegree;

    // Accesses general information about the current graph
    out << "The graph has " << g.numVertices() << " vertices and "
        << g.numEdges() << " edges" << endl;
    // Iterates over the vertices &
    // Accesses parameters for each vertex &
    // Accesses the degree of each vertex (NOT A PROPERTY OF THE VERTEX!) &
    // Accesses edges connected to each vertex 
    out << endl << "The vertices of the graph are:" << endl;

    for (tie(vi, vi_end) = g.getVertices(); vi != vi_end; vi++)
    {
        currVertex = *vi;
        vprop = g.getVProp(currVertex);
        currDegree = g.degree(*vi);
        out << vprop;

        if (currDegree > 0)
        {
            out << ", connected to edges: ";
            for (tie(oi, oi_end) = g.getEdges(currVertex); oi != oi_end; oi++)
            {
                eprop = g.getEProp(*oi);
                out << eprop.Eid << " ";
            }
        }

        out << ", is null? " << (currVertex == ReebGraph::nullVertex())
            << endl;
    }

    // Iterates over the edges &
    // Accesses parameters for each edge
    out << endl << "The edges of the graph are:" << endl;
    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
    {
        currEdge = *ei;
        eprop = g.getEProp(currEdge);
        tie(v_first, v_second) = g.getEndNodes(*ei); // <---- IMPORTANT
        out << eprop << ", attached to vertices: " << g.getVProp(v_first).Vid
            << " & " << g.getVProp(v_second).Vid << ", is null? "
            << (currEdge == ReebGraph::nullEdge()) << endl;
    }

    return out;
}


/*************************************************************************
 * Function 'runExample()'
 *
 * Method that runs an example of the ReebGraph.
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void ReebGraph::runExample() 
{
    // Creates a new graph
    ReebGraph g;

    // Adds vertices by specifying vertex properties
    // NOTE: Vid are automatically assigned by addVertex()
    Vertex v0 = g.addVertex(1, 2, 3, 4);        // Vid == 0
    Vertex v1 = g.addVertex(5, 6, 7);           // Vid == 1
    Vertex v2 = g.addVertex(8, 9, 10);          // Vid == 2
    Vertex v3 = g.addVertex(11, 12, 13, 14);    // Vid == 3
    Vertex v4 = g.addVertex(15, 16, 17);        // Vid == 4
    Vertex vnull = ReebGraph::nullVertex();

    // Adds edges by either specifying Vid or Vertex end-points
    // and specifying edge properties
    // NOTE: Eid are automatically assigned by addEdge()
    Edge e0 = g.addEdge(0, 1, 15);              // Eid == 0
    Edge e1 = g.addEdge(v1, v4, 16);            // Eid == 1
    Edge e2 = g.addEdge(0, 3, 17);              // Eid == 2
    Edge e3 = g.addEdge(v0, v3, 18);            // Eid == 3
    Edge e4 = g.cloneEdge(e1);                  // Eid == 4
    Edge e5 = g.addEdge(v0, v0, 19);            // Eid == 5
    Edge enull = ReebGraph::nullEdge();

    // Declares some local variables
    ReebVertex vprop;
    ReebEdge eprop;
    Vertex currVertex;
    Edge currEdge;
    Vertex_Iter vi, vi_end;
    Edge_Iter ei, ei_end;
    Out_Edge_Iter oi, oi_end;
    Vertex v_first, v_second;
    unsigned int currDegree;

    // Accesses general information about the current graph
    std::cout << "The graph has " << g.numVertices() << " vertices and "
        << g.numEdges() << " edges" << std::endl;

    // Accesses the first vertex &
    // Edits parameters of a specific vertex and a specific edge
    g.getVProp(g.getFirstVertex()).color = 20;
    g.getEProp(1).cost = 10;

    // Iterates over the vertices &
    // Accesses parameters for each vertex &
    // Accesses the degree of each vertex (NOT A PROPERTY OF THE VERTEX!) &
    // Accesses edges connected to each vertex
    std::cout << std::endl << "The vertices of the graph are:" << std::endl;

    for (tie(vi, vi_end) = g.getVertices(); vi != vi_end; vi++) 
    {
        currVertex = *vi;
        vprop = g.getVProp(currVertex);
        currDegree = g.degree(*vi);
        std::cout << vprop;

        if (currDegree > 0)
        {
            std::cout << ", connected to edges: ";
            for (tie(oi, oi_end) = g.getEdges(currVertex); oi != oi_end; oi++) 
            {
                eprop = g.getEProp(*oi);
                std::cout << eprop.Eid << " ";
            }
        }

        std::cout << ", is null? " << (currVertex == ReebGraph::nullVertex())
            << std::endl;
    }

    // Iterates over the edges &
    // Accesses parameters for each edge
    std::cout << std::endl << "The edges of the graph are:" << std::endl;

    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
    {
        currEdge = *ei;
        eprop = g.getEProp(currEdge);
        tie(v_first, v_second) = g.getEndNodes(*ei); // <---- IMPORTANT
        std::cout << eprop << ", attached to vertices: "
            << g.getVProp(v_first).Vid << " & "
            << g.getVProp(v_second).Vid << ", is null? "
            << (currEdge == ReebGraph::nullEdge()) << std::endl;
    }

    std::cout << std::endl;

    // Test null vertex and null edge
    std::cout << "Is vnull a null vertex? "
        << (vnull == ReebGraph::nullVertex()) << std::endl;
    std::cout << "Is enull a null edge? "
        << (enull == ReebGraph::nullEdge()) << std::endl;

    // Modifying the end nodes of an edge and re-printing all edges
    e3 = g.modifyEndNodes(e3, v1, v2);
            // Changing from edge(v0, v3) to edge(v1, v2)
    e5 = g.modifyEndNodes(5, 2);
            // Changing from edge(v0, v0) to edge(v2, v0)
    std::cout << std::endl << "The updated edges of the graph are:"
        << std::endl;

    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
    {
        currEdge = *ei;
        eprop = g.getEProp(currEdge);
        tie(v_first, v_second) = g.getEndNodes(*ei);
        std::cout << eprop << ", attached to vertices: "
            << g.getVProp(v_first).Vid << " & " << g.getVProp(v_second).Vid
            << std::endl;
    }

    std::cout << std::endl;
};



/*************************************************************************
 * Function 'getGraphEdges()'
 *
 * Method that gets the edges of a graph such that it can be returned
 * as a vector. 
 *
 * Returns:
 *   std::vector<Edge> - edges to the current graph 
 *
 * Parameters:
 *   None
 *
**/
std::vector<Edge> ReebGraph::getGraphEdges()
{
    return Epointers;
};


/*************************************************************************
 * Function 'setGraphEdges()'
 *
 * Method that sets the edges of a graph.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   edges - edges of the new graph
 *
**/
void ReebGraph::setGraphEdges(std::vector<Edge> edges)
{
    Epointers = edges;
};

/*************************************************************************
 * Function 'getGraphVertex()'
 *
 * Method that gets the edges of a graph such that it can be returned
 * as a vector.
 *
 * Returns:
 *   std::vector<Vertex> - edges to the current graph 
 *
 * Parameters:
 *   None
 *
**/
std::vector<Vertex> ReebGraph::getGraphVertex()
{
    return Vpointers;
};


/*************************************************************************
 * Function 'setGraphVertex()'
 *
 * Method that sets the edges of a graph.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   vertex - Vertex of the new graph
 *
**/
void ReebGraph::setGraphVertex(std::vector<Vertex> vertex)
{
    Vpointers = vertex;
};

/*************************************************************************
 * Function 'printEdges()'
 *
 * Method that prints the edges of a graph.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void ReebGraph::printEdges()
{
    vector<Edge>::iterator iter;
    for(iter = Epointers.begin(); iter != Epointers.end(); ++iter)
    {
        cout << (*iter) << " ";;
    }
}

/*************************************************************************
 * Function 'printVertex()'
 *
 * Method that prints the vertices of a graph.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void ReebGraph::printVertex()
{
    vector<Vertex>::iterator iter;
    for(iter = Vpointers.begin(); iter != Vpointers.end(); ++iter)
    {
        cout << (*iter) << " ";;
    }
}

