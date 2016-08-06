#ifndef REEBGRAPH_H_
#define REEBGRAPH_H_

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <list>
#include <vector>
#include <ostream>
#include "Point2D.h"

#include <boost/graph/graph_utility.hpp>

using namespace boost;


/**
* Defines the object type for vertices in the Reeb graph
**/
struct ReebVertex 
{
  double x;
  double y1;
  double y2;
  int color;
  unsigned int Vid;

  // Custom constructors
  // NOTE: use this instead of defining params. manually
  ReebVertex(double _x, double _y1, double _y2, int _color, unsigned int _Vid):
    x(_x), y1(_y1), y2(_y2), color(_color), Vid(_Vid) {};

  // Default constructor
  // NOTE: this is provided to make BGL happy...
  //       try to use custom constructor if possible
  ReebVertex() :
    x(-1), y1(-1), y2(-1), color(-1), Vid(0) {};
};


/**
* Defines the object type for edges in the Reeb graph
**/
struct ReebEdge 
{
  std::vector<Point2D> topBoundary;
  std::vector<Point2D> bottomBoundary;
  int color;
  double cost;
  unsigned int Eid;
  int area;
  double travelCost; //FIXME <N> length of edge - euclidean distance between the source and target vertices

  // Custom constructor
  // NOTE: use this instead of defining params. manually
  ReebEdge(int _color, unsigned int _Eid):
    topBoundary(), bottomBoundary(), color(_color), cost(-1), Eid(_Eid), travelCost(-1) {};

  // Default constructor
  // NOTE: this is provided to make BGL happy...
  //       try to use custom constructor if possible
  ReebEdge() :
    topBoundary(), bottomBoundary(), color(-1), cost(-1), Eid(0), travelCost(-1) {};

  /*Compute Area*/
  void updateArea()
  {
    area  = 0;
    for(unsigned int i = 0;i<topBoundary.size();i++)
    {
      area += abs(topBoundary[i].ycoord() - bottomBoundary[i].xcoord());
    }
  }

  //FIXME: <N> set travel cost
  void setTravelCost(const double tCost) {
      travelCost = tCost;
  }

};


// Defines type aliases for graph-related structures
// JUSTIFICATIONS:
// Using std::list for edge container: O(log(E/V)) add_edge() insertion, O(E/V) edge() access
// Using std::list for vertex container: sacrifice space to get better time performance
typedef adjacency_list<listS, listS, undirectedS, ReebVertex, ReebEdge> Graph;//FIXME: listS is changed to vecS
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

typedef boost::graph_traits<Graph>::vertex_iterator Vertex_Iter;
typedef boost::graph_traits<Graph>::edge_iterator Edge_Iter;
typedef boost::graph_traits<Graph>::adjacency_iterator Adjacency_Iter;
typedef boost::graph_traits<Graph>::out_edge_iterator Out_Edge_Iter;


// This class provides a thin wrapper for Boost.Graph class
class ReebGraph 
{

public:

  // Initializes member variables
  ReebGraph() : g(), Vpointers(), Epointers(), Vid_free(0), Eid_free(0) {
  };

  ReebGraph(const ReebGraph& _g) : g(), Vpointers(), Epointers(), Vid_free(0), Eid_free(0)
  {
    g = _g.g;
    Vpointers.resize(_g.Vpointers.size());
    copy(_g.Vpointers.begin(),_g.Vpointers.end(),Vpointers.begin());
    Epointers.resize(_g.Epointers.size());
    copy(_g.Epointers.begin(),_g.Epointers.end(),Epointers.begin());
    Vid_free = _g.Vid_free;
    Eid_free = _g.Eid_free;
  };

  ReebGraph& operator=( const ReebGraph& _g ) 
  {
    g = _g.g;
    Vpointers.resize(_g.Vpointers.size());
    copy(_g.Vpointers.begin(),_g.Vpointers.end(),Vpointers.begin());
    Epointers.resize(_g.Epointers.size());
    copy(_g.Epointers.begin(),_g.Epointers.end(),Epointers.begin());
    Vid_free = _g.Vid_free;
    Eid_free = _g.Eid_free;
    return *this;
  }

  bool equals(ReebGraph Graph);

  // Cleans up member variables
  ~ReebGraph() 
  {
    clear();
  };


  // Cleans up member variables
  void clear() 
  {
    Vpointers.clear();
    Epointers.clear();
    g.clear();
    Vid_free = 0;
    Eid_free = 0;
  };

  std::list<Edge> getEdgeList()
  {
    std::list<Edge> temp(Epointers.begin(), Epointers.end());
    return temp;
  }

  /*FIXME: >N<
  void printGraph()
  {
      boost::print_graph(g, boost::get(&ReebEdge::Eid,g));
  };
  //>N<!*/


  // Adds a new vertex into the current graph : g(), Vpointers(), Epointers(), Vid_free(0), Eid_free(0)
  Vertex addVertex(double x, double y1, double y2, int color) 
  {
    Vertex v = add_vertex(ReebVertex(x, y1, y2, color, Vid_free++), g);
    Vpointers.push_back(v);
    return v;
  };

  Vertex addVertex(double x, double y, int color) 
  {
    return addVertex(x, y, y, color);
  };


  // Adds a new edge into the current graph
  Edge addEdge(Vertex V1, Vertex V2, int color) 
  {
    Edge e = add_edge(V1, V2, ReebEdge(color, Eid_free++), g).first;
    Epointers.push_back(e);
    return e;
  };

  Edge addEdge(unsigned int Vid1, unsigned int Vid2, int color) 
  {
    return addEdge(Vpointers[Vid1], Vpointers[Vid2], color);
  };


  // Makes a clone of a particular edge (deep copy)
  Edge cloneEdge(Edge old_e) 
  {
    // Clone properties of the existing edge
    ReebEdge old_eprop = g[old_e];
    ReebEdge new_eprop(old_eprop.color, Eid_free++);
    new_eprop.cost = old_eprop.cost;
    new_eprop.topBoundary = old_eprop.topBoundary;
    new_eprop.bottomBoundary = old_eprop.bottomBoundary;

    // Access the vertices attached to the existing edge
    Vertex V1 = source(old_e, g);
    Vertex V2 = target(old_e, g);

    // Create new edge with the cloned properties and the same vertices
    Edge new_e = add_edge(V1, V2, new_eprop, g).first;
    Epointers.push_back(new_e);
    return new_e;
  };

  Edge cloneEdge(unsigned int Eid) 
  {
    return cloneEdge(Epointers[Eid]);
  };

  // Makes a clone of a particular edge (deep copy)
  void updateCellArea() 
  {
    for(unsigned int i = 0; i < Epointers.size(); i++)
    {
      g[Epointers[i]].updateArea();
    }
  };

  //FIXME <N>: Must be modified to assign shortest geometrical distance between critical points
  /* Computes the travel - Euclidean Distance, for each edge in the graph
   * */
  void updateTravelDistances()
  {
    for(unsigned int i = 0;i<Epointers.size();i++)
    {
	Edge e = Epointers[i];
	ReebVertex V1 = getVProp( source(e, g));
	ReebVertex V2 = getVProp( target(e, g));
	Point2D p1(V1.x, (V1.y1 + V1.y2)/2);
	Point2D p2(V2.x, (V2.y1 + V2.y2)/2);
	g[e].setTravelCost(p1.distance(p2));
	/*FIXME <N> debug
	std::cout << "distance for edge -> " << g[e].Eid << " " << g[e].travelCost << std::endl;
	*/
    }
  }


  // Returns the common vertex shared by 2 edges; if not applicable
  // then return ReebGraph::nullVertex()
  Vertex findCommonVertex(Edge e1, Edge e2) 
  {
    if (e1 != ReebGraph::nullEdge() && e2 != ReebGraph::nullEdge()) 
    {
      Vertex v1 = source(e1, g), v2 = target(e1, g);
      Vertex v3 = source(e2, g), v4 = target(e2, g);
      
      if (v1 == v3 || v2 == v3) 
      {
        return v3;
      }
 
      else if (v1 == v4 || v2 == v4) 
      {
        return v4;
      }
    }
    return ReebGraph::nullVertex();
  };


  // Returns the vertex / edge object for a specific vertex / edge pointer
  // WARNING: check if agument is valid BEFORE calling the following functions
  ReebVertex& getVProp(Vertex v) 
  {
    return g[v];
  };

  ReebVertex& getVProp(unsigned int Vid) 
  {
    return g[Vpointers[Vid]];
  };

  ReebEdge& getEProp(Edge e) 
  {
    return g[e];
  };

  ReebEdge& getEProp(unsigned int Eid) 
  {
    return g[Epointers[Eid]];
  };


  // Returns the 2 vertices attached to a specific edge
  std::pair<Vertex, Vertex> getEndNodes(Edge e) 
  {
    if (e != nullEdge()) 
    {
      return std::make_pair(source(e, g), target(e, g));
    } 

    else 
    {
      return std::make_pair(nullVertex(), nullVertex());
    }
  };


  std::pair<Vertex, Vertex> getEndNodes(unsigned int Eid) 
  {
    return getEndNodes(Epointers[Eid]);
  };


  // Modifies the end node(s) attached to a specific edge
  // NOTE: if newV2 == ReebGraph::Vnull, then newV2 will be assigned to
  //       target(e, graph), i.e. one of the end nodes of the previously
  //       specified edge
  Edge modifyEndNodes(Edge e, Vertex newV1, Vertex newV2 = ReebGraph::Vnull) 
  {
    ReebEdge eData = getEProp(e);

    if (newV2 == ReebGraph::Vnull) 
    {
      newV2 = target(e, g);
    }

    remove_edge(e, g);
    Edge newE = add_edge(newV1, newV2, eData, g).first;
    Epointers[eData.Eid] = newE;
    return newE;
  };

  Edge modifyEndNodes(unsigned int Eid, unsigned int newV1id, \
      unsigned int newV2id = 0) 
  {
    return modifyEndNodes(Epointers[Eid], Vpointers[newV1id], \
      ((newV2id == 0) ? ReebGraph::Vnull : Vpointers[newV2id]));
  };


  // Returns the first vertex in the graph
  // NOTE: If no vertices exist in graph, then this function returns NULL
  Vertex getFirstVertex() 
  {
    if (num_vertices(g) > 0) 
    {
      //return *(vertices(g).first);
      return Vpointers[0]; // Hopefully this is more predictable/repeatable than the above line
    }
 
    else 
    {
      return NULL;
    }
  };


  // Returns start and past-the-end iterators for vertices / edges
  std::pair<Vertex_Iter, Vertex_Iter> getVertices() 
  {
    return vertices(g);
  };

  std::pair<Edge_Iter, Edge_Iter> getEdges() 
  {
    return edges(g);
  };


  // Returns start and past-the-end iterators for edges attached to
  // a specific vertex
  std::pair<Out_Edge_Iter, Out_Edge_Iter> getEdges(Vertex v) 
  {
    return out_edges(v, g);
  };

  std::pair<Out_Edge_Iter, Out_Edge_Iter> getEdges(unsigned int Vid) 
  {
    return getEdges(Vpointers[Vid]);
  };

  Vertex getVertex(unsigned int Vid) 
  {
    if (Vid >= Vpointers.size()) 
    {
      return nullVertex();
    } 

    else 
    {
      return Vpointers[Vid];
    }
  };

  Edge getEdge(unsigned int Eid) 
  {
    if (Eid >= Epointers.size()) 
    {
      return nullEdge();
    } 

    else 
    {
      return Epointers[Eid];
    }
  };


  // Provides auxiliary informations regarding graphs, vertices, and edges
  unsigned int numVertices() 
  {
    return num_vertices(g);
  };

  unsigned int numEdges() 
  {
    return num_edges(g);
  };

  unsigned int degree(Vertex v) 
  {
    return out_degree(v, g);
  };

  unsigned int degree(int Vid) 
  {
    return degree(Vpointers[Vid]);
  };

  bool empty() 
  {
    return ((num_vertices(g) == 0) || (num_edges(g) == 0));
  };

  // Mass-reset member variables
  void resetAllVertexColor(int newColor = 0) 
  {
    Vertex_Iter vi, vi_end;
    for (tie(vi, vi_end) = vertices(g); vi != vi_end; vi++) 
    {
      g[*vi].color = newColor;
    }
  };

  void resetAllEdgeColor(int newColor = 0) 
  {
    Edge_Iter ei, ei_end;
    for (tie(ei, ei_end) = edges(g); ei != ei_end; ei++) 
    {
      g[*ei].color = newColor;
    }
  };

  void resetAllColor(int newColor = 0) 
  {
    resetAllVertexColor(newColor);
    resetAllEdgeColor(newColor);
  };

  // Returns singleton objects for null vertex & null edge pointers
  static Vertex nullVertex() 
  {
    return ReebGraph::Vnull;
  };

  static Edge nullEdge() 
  {
    return ReebGraph::Enull;
  };


  // Provides example code for how to use this class (see ReebGraph.cpp)
  static void runExample();

  void setGraphEdges(std::vector<Edge> edges);
  std::vector<Edge> getGraphEdges();

  void setGraphVertex(std::vector<Vertex> vertex);
  std::vector<Vertex> getGraphVertex();

  void printEdges();
  void printVertex();

private:

  // Boost graph object
  Graph g;

  // Internal structures that facilitate accessing vertex and edge pointers
  // when addressed using their IDs
  std::vector<Vertex> Vpointers;
  std::vector<Edge> Epointers;
  //std::pair<Edge_Iter,Edge_Iter> p;

  // Next free vertex & edge ID values
  unsigned int Vid_free;
  unsigned int Eid_free;

  // Singleton objects for null vertex and null edge
  const static Vertex Vnull;
  const static Edge Enull;
};


// Defines ostream debug-print functions
ostream& operator<< (ostream& out, ReebVertex& v);
ostream& operator<< (ostream& out, ReebEdge& e);
ostream& operator<< (ostream& out, ReebGraph& g);

#endif /* REEBGRAPH_H_ */
