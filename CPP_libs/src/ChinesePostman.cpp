#include "ChinesePostman.h"

/******************************************************************************
 *
 * @maintainer: Kelly Benson
 * @email: bensonke@email.sc.edu
 * @date: May 16, 2015
 *
 * This file is for the ChinesePostman calculation and work behind that. 
 * There are several different constructors provided to give the user
 * options as to what they want to do. This might be amended at a later 
 * date depending on what is needed and how correct providing these options
 * are to the programs purpose itself. 
 * 
**/

glp_prob* ChinesePostman::lp = NULL;
bool ChinesePostman::glp_successful = true;


/*************************************************************************
 * Constructor
 *
**/
ChinesePostman::ChinesePostman() {};

/*************************************************************************
 * Alternate Constructor - needed for when you pass in BCD info
 *
**/
ChinesePostman::ChinesePostman(RegionData& data, ReebGraph& graph,
        std::list<Edge>& eulerCycle, vector<Point2D>& wayPoints)
{
    lp = NULL;
    glp_successful = false;

    if (graph.empty() == true) 
    {
        std::cout << "Graph is empty. Did you build the BCD?";
        return;
    }

    solveCPP(data, graph, eulerCycle);
};


/************************************************************************
 * Destructor
 *
**/
ChinesePostman::~ChinesePostman() {};


/*************************************************************************
 * Author: Patrick Virie
 * Function 'addPairToEndPairings()'
 *
 * Returns:
 *   None
**/
void ChinesePostman::addPairToEndPairings(Edge e1, Edge e2, Vertex n,
        std::list<Edge> *unpairedEdges, std::list<Edge> *pairedEdges,
        std::list< std::pair<std::pair<Edge,Edge>,Vertex> > *EndPairs) 
{
    EndPairs->push_back(make_pair(make_pair(e1, e2), n));

    for (list<Edge>::iterator eit=unpairedEdges->begin();
            eit!=unpairedEdges->end(); ++eit)
    {
        if (*eit==e1)
        {
            pairedEdges->splice(pairedEdges->end(), *unpairedEdges, eit);
            break;
        }
    }
};


/**
 * Author : Patrick Virie
 * Ported and edited from Raphael's code.
 *
 * Solve the Chinese Postman Problem on the Reeb graph.
 *
 * (1)  Adding certain edges to the graph to turn into a Euler Graph
 *          (where all nodes have even degree)
 *          if it isn't one. The added edges should minimize the cumulative
 *          edge cost of the resulting Euler graph.
 *
 *  ALGO::
 *      1. Express the CPP as a Linear Programming Problem (LPP) using the GLPK API...
 *          The LPP is structured as follows
 *
 *              CONSTRAINTS
 *                  Sum_over_e(Ane * Xe) - 2Wn = Bn     for each node n
 *                  Xe >= 0                             for each edge e
 *                  Xe is integer                       for each edge e
 *                  Wn >= 0                             for each node n
 *                  Wn is integer                       for each node n
 *
 *                  where
 *                      Sum(...) represents the number of added edges to node n in the
 *                                       solution... for the solution to be a Euler Graph, we
 *                                       expect an odd number of edges to be added to odd nodes and
 *                                       an even number of edges to be added to even nodes
 *                                       (where odd nodes have odd degree...)
 *                      Ane         is 1 if node n meets edge e, and 0 otherwise
 *                      Xe          is a variable that will contain the number of added "copies"
 *                                          of edge e in the solution
 *                      Wn          is a variable that will force the preceding sum to be odd
 *                                          for odd nodes and even for even nodes
 *                      Bn          is 1 for odd nodes and 0 for even nodes
 *
 *              OBJECTIVE FUNCTION
 *                  Min( Sum_over_e(Ce * Xe) )
 *
 *                  where
 *                      Ce          is the cost of edge e
 *
 *      2. Solve the LPP *
 *
 * (2)  Compute a Euler Cycle of the resulting Euler Graph
 *
 *  ALGO::
 *      See Matching, Euler Tours and the Chinese Postman by Edmonds and Johnson, p.23
 *      (End-Pairing Algorithm)
 *
 * NOTE 1 :: -- At the prof's request -- This solver doesn't use any of the properties
 * of a Reeb Graph : it is a generic solver for the CPP that can be used on any graph.
 *
 * NOTE 2 :: A useful simplex applet can be found here http://vinci.inesc.pt/lp/
 *
 * NOTE 3 :: After discovering a problem in the integer solver (it sometimes produces
 * non-integer values for Wn), I broke NOTE 1 by forcing Wn to always be 0. This might
 * screw up the generality of the code but it still works for us due to certain
 * properties of our graphs (reeb graphs). << I have found the cause of the problem.
 * Raphael used a wrong function to get answers from MIP.
 */
void ChinesePostman::solveCPP(RegionData& data, ReebGraph& graph,
        std::list<Edge>& eulerCycle) 
{
    cv::Mat map = data.map;
    eulerCycle.clear();

    // Create jump structures for REALLY UGLY STYLE C error handling
    //
    // Based on: http://www.di.unipi.it/~nids/docs/longjump_try_trow_catch.html
    jmp_buf glpErrorJmpBuf;
    if (setjmp(glpErrorJmpBuf))
    {
        // setjmp returned non-zero value == error
        eulerCycle.clear();
        return;
    }

    /************** PART 1 : Turn the provided graph into a Euler Graph **************/
    /** Init the LPP **/
    if (lp != NULL)
    {
        glp_delete_prob(lp);
        lp = NULL;
    }

    lp = glp_create_prob();
    //glp_error_hook(CoverAlgo_Optimal::glpErrorCallback, (void*) &glpErrorJmpBuf);
    glp_term_out(GLP_OFF);
    glp_successful = true;
    glp_set_prob_name(lp, "ChinesePostmanProblem");
    glp_set_obj_dir(lp, GLP_MIN);

    ReebVertex vprop;
    ReebEdge eprop;
    Vertex_Iter vi, vi_end;
    Edge_Iter ei, ei_end;
    Out_Edge_Iter oi, oi_end;
    Vertex _v1, _v2;
    Edge _e1, _e2;

    // Sanity check for existence of BCD
    if (graph.numVertices() == 0 || graph.numEdges() == 0) 
    {
        //std::cout << "CODING ERROR @ solveCPP() > GRAPH IS EMPTY!"<< endl;
        return;
    }


    /** Setup the LPP based on the graph's nodes and edges **/
    /* Setup right hand side of the summation constraints */
    int nb_nodes = graph.numVertices();
    glp_add_rows(lp, nb_nodes);
    tie(vi, vi_end) = graph.getVertices();
    for (int i = 1; vi != vi_end; ++vi, ++i)
    {
        int mod_degree = graph.degree(*vi) % 2;
        glp_set_row_bnds(lp, i, GLP_FX, mod_degree, mod_degree);
    }

    /* Setup non-negativty constraints on Xe and Wn variables and
     * coefficients of Xe variables in the objective function */
    int nb_edges = graph.numEdges();
    glp_add_cols(lp, nb_nodes+nb_edges);
    stringstream obj_func;
    stringstream Xe_constraints;
    tie(ei, ei_end) = graph.getEdges();


    for( int i = 1; ei != ei_end; ++ei, ++i) //set Xe
    {
        glp_set_col_kind(lp, i, GLP_IV);
        glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);
        glp_set_obj_coef(lp, i, (graph.getEProp(*ei)).cost);
        //glp_set_obj_coef(lp, i, 1 );

        obj_func << (graph.getEProp(*ei)).cost << "*X" << i << " + ";
        //obj_func << 1 << "*X" << i << " + ";
        Xe_constraints << "X" << i << ">=0 (int);  ";
    }

    stringstream Wn_constraints;

    for( int i = nb_edges+1; i<=nb_nodes+nb_edges; ++i ) //set Wn
    {
        glp_set_col_kind(lp, i, GLP_IV);
        //glp_set_col_kind(lp, i, GLP_BV);
        glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);
        //glp_set_col_bnds(lp, i, GLP_FX, 0.0, 0.0);        /* SEE NOTE 3 !! */
        glp_set_obj_coef(lp, i, 0);
        Wn_constraints << "W" << i << ">=0 (int);  ";
    }

    /* Setup the coefficients in the left hand side of the summation constraints */
    /* TBI: 1000 is just some arbitrary high number that may not be high enough...
     * a more robust version should compute the correct value to put there */
    int ia[1+1000], ja[1+1000]; double ar[1+1000];
    stringstream sum_constraints;
    int constraint_index = 1;
    tie(vi, vi_end) = graph.getVertices();
    for (int i = 1; vi != vi_end; ++vi, ++i)
    {
        /* Xe coefficients */
        tie(ei, ei_end) = graph.getEdges();
        for (int j = 1; ei != ei_end; ++ei, ++j)
        {
            tie(_v1, _v2) = graph.getEndNodes(*ei);
            if (_v1 == *vi || _v2 == *vi)
            {
                ia[constraint_index] = i;
                ja[constraint_index] = j;
                ar[constraint_index] = 1;
                ++constraint_index;
                sum_constraints << "1*X" << j << " + ";
            }
        }
        /* Wn coefficients */
        ia[constraint_index] = i;
        ja[constraint_index] = i+nb_edges;
        ar[constraint_index] = -2;
        ++constraint_index;
        sum_constraints << "-2W" << (i+nb_edges) << " = "
            << (graph.degree(*vi) % 2) << endl << "\t";
    }


    glp_load_matrix(lp, --constraint_index, ia, ja, ar);

    /* Output the LPP */
/*
    //std::cerr << endl << "\t CPP as LPP" << endl;
    //std::cerr << "\tOBJ. FUNC:: MIN( " << obj_func.str() << " )" << endl;
    //std::cerr << "\tXe constraints:: " << Xe_constraints.str() << endl;
    //std::cerr << "\tWn constraints:: " << Wn_constraints.str() << endl;
    //std::cerr << "\t" << sum_constraints.str() << endl;
*/

    /** Use presolver instead **/
//  /** Solve the LPP **/
//  /* Must solve the non-integer LPP first */
//  glp_smcp gs;
//  glp_init_smcp(&gs);
//  gs.msg_lev = GLP_MSG_ERR;
//  glp_simplex(lp, &gs);

    /* Now solve the integer LPP */
    glp_iocp gi;
    glp_init_iocp(&gi);
    gi.msg_lev = GLP_MSG_ERR;
    gi.presolve = GLP_ON;
    glp_intopt(lp, &gi);

    /** Output solution and add appropriate "new" edges to *edges std::vector **/
    stringstream Xe_values;
    stringstream Wn_values;
    tie(ei, ei_end) = graph.getEdges();
    for (int i = 1, n_Edges = graph.numEdges(); i<=n_Edges; ++ei, ++i)
    {
        if (glp_mip_col_val(lp, i) > 0)
        {
/*
            //std::cerr << "\tAdding " << glp_mip_col_val(lp, i) << \
                    " new copies of edge " << graph.getEProp(*ei) << endl;
*/

            Edge new_edge = graph.cloneEdge(*ei);
            tie(_v1, _v2) = graph.getEndNodes(*ei);

            /** Split the cell into two regions **/
            double sr,er;
            if (graph.getVProp(_v1).x > graph.getVProp(_v2).x)
                swap(_v1,_v2);

            double _t,_b,_vy;
            _t = graph.getEProp(*ei).topBoundary[0].ycoord();
            _b = graph.getEProp(*ei).bottomBoundary[0].ycoord();
            _vy = (graph.getVProp(_v1).y1 + graph.getVProp(_v1).y2) / 2;

            if(abs(_b - _t) >= 1e-4)
            {
                sr = min(max((_vy-_t) / (_b-_t), 0.0), 1.0);
            }
            else
            {
                sr = 0.5;
            }

            _t = graph.getEProp(*ei).topBoundary[graph.getEProp(*ei)
                .topBoundary.size() - 1].ycoord();
            _b = graph.getEProp(*ei).bottomBoundary[graph.getEProp(*ei)
                .topBoundary.size() - 1].ycoord();
            _vy = (graph.getVProp(_v2).y1 + graph.getVProp(_v2).y2) / 2;

            if (abs(_b - _t) >= 1e-4)
            {
                er = min(max((_vy-_t) / (_b-_t), 0.0), 1.0);
            }
            else
            {
                er = 0.5;
            }

            double step = (er-sr) / (graph.getEProp(*ei).topBoundary.size());
            std::vector<Point2D> midBoundary;

            for (unsigned int j = 0; j<graph.getEProp(*ei).topBoundary.size();
                    ++j, sr += step)
            {
                midBoundary.push_back(
                        ((graph.getEProp(*ei).topBoundary[j])*(1-sr))
                        + ((graph.getEProp(*ei).bottomBoundary[j])*(sr)));
            }
            graph.getEProp(*ei).bottomBoundary = midBoundary;
            graph.getEProp(new_edge).topBoundary = midBoundary;

        }
        else
        {
            //std::cerr <<    "\t\tNot adding any copies of edge " << graph.getEProp(*ei) << endl;
        }
        Xe_values << "X" << i << "=" << glp_mip_col_val(lp, i) << ";  ";
    }

    for (int i = nb_edges+1; i <= nb_nodes+nb_edges; ++i)
        Wn_values << "W" << i << "=" << glp_mip_col_val(lp, i) << ";  ";

    glp_delete_prob(lp);
    lp = NULL;


    graph.updateCellArea();
    graph.updateTravelDistances(); //added

#ifdef P_PRINT_DEBUG
#ifdef ENABLE_IMSHOW
    cv::imshow("Image View", image);
    cv::waitKey(16);
#endif
#endif

    /************** PART 2 : Compute a Euler Cycle from the Eulerized graph **************/

    /** Using the End-Pairing Algorithm frpm Matching, Euler Tours and the Chinese Postman
     * by Edmonds and Johnson, p.23, compute a list of edge pairs from which a Euler cycle
     * can be extracted **/
    /* Step 0 : Initializations */

    /* Cpp data structures */
    std::list<Edge> unpairedEdges;
    std::list<Edge> pairedEdges;
    std::list< std::pair<std::pair<Edge,Edge>,Vertex> > EndPairs;

    Vertex r = graph.getFirstVertex();
    Vertex n0 = r;
    Vertex n = r;

        // FIXME: >N< 
    std::cerr << "r -> " << r << "\n";
    std::cerr << "n0 -> " << n0 << "\n";;
    std::cerr << "n -> " << n << "\n";
    // FIXME: >N<! 

    tie(oi,oi_end) = graph.getEdges(r);

    Edge e = *oi;
    //std::cerr << "Edge oi: " << *oi << "\n";
    //std::cerr << "Edge oi_end: " << *oi_end << "\n";
    Edge e0 = *oi;
    Edge e1 = ReebGraph::nullEdge();
    Edge e2 = ReebGraph::nullEdge();

    unpairedEdges.clear();
    pairedEdges.clear();
    EndPairs.clear();


    for( tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ++ei)
    {
        unpairedEdges.push_back(*ei);
        //std::cerr << "Unpaired edge: " << (*ei) << "\n";
    }


    while(1)
    {
        /* Step 1 :
         * Let n' be the node other than n incident to edge e. If there is an
         * edge e' with an end meeting n' which is not yet paired, go to Step
         * 2. Otherwise, n' must be equal to n0. In that case, form the edge
         * pairs (el,e0) and (e,e2) meeting node n0. Go to Step 3. */
        tie(_v1, _v2) = graph.getEndNodes(e);

        //std::cerr << "\n" << "\n";

        // vert = graph.getVProp(n);
        //std::cout << "Vert n: " << n << " : "<< vert;
        //std::cerr << "\n";

        //vert = graph.getVProp(_v1);
        //std::cout << "Vert _v1: " << _v1 << " : "<< vert;
        //std::cerr << "\n";

        //vert = graph.getVProp(_v2);
        //std::cout << "Vert _v2: " << _v2 << " : "<< vert;
        //std::cerr << "\n";

        //Cant just change the expression to !=
        //n_ need to be the second vertex then.
        //  IF n == v1, then n_ = _v2.
        //  IF n != v1, then n_ = _v1
        Vertex n_ = (_v1 == n ? _v2 : _v1);

        //vert = graph.getVProp(n_);
        //std::cout << "Vert n_: " << n_ << " : "<< vert;
        //std::cerr << "\n";

        //vert = graph.getVProp(n0);
        //std::cout << "Vert n0: " << n0 << " : "<< vert;
        //std::cerr << "\n";

        // Iterates over the vertices &
        // Accesses parameters for each vertex &
        // Accesses the degree of each vertex (NOT A PROPERTY OF THE VERTEX!) &
        // Accesses edges connected to each vertex
        //std::cout << std::endl << "The vertices of the graph are:" << std::endl;
        //testValues(data, graph);
        //std::cerr << "\n";

        Edge e_ = ReebGraph::nullEdge();

        /* finding the closest edge */
        double min_dist = (map.cols + map.rows)*(map.cols + map.rows);
        //cerr << "min_dist: " << min_dist << "\n";
        for (list<Edge>::iterator eit = unpairedEdges.begin();
                eit != unpairedEdges.end(); ++eit )
        {
            tie(_v1, _v2) = graph.getEndNodes(*eit);
            ////std::cerr << "Edge eit: " << *eit << "\n";

            if ((_v1 == n_ || _v2 == n_) && e!=*eit )
            {
                ReebEdge _prob_e = graph.getEProp(e);
                ReebEdge _prob_eit = graph.getEProp(*eit);
                Point2D _me0 = (_prob_e.topBoundary.front()
                        + _prob_e.bottomBoundary.front()) / 2;
                Point2D _me1 = (_prob_e.topBoundary.back()
                        + _prob_e.bottomBoundary.back()) / 2;
                Point2D _met0 = (_prob_eit.topBoundary.front()
                        + _prob_eit.bottomBoundary.front()) / 2;
                Point2D _met1 = (_prob_eit.topBoundary.back()
                        + _prob_eit.bottomBoundary.back()) / 2;
                double _d = min(min(_me0.sdist(_met0), _me0.sdist(_met1)),
                        min(_me1.sdist(_met0), _me1.sdist(_met1)));
                if (_d < min_dist)
                {
                    e_ = *eit;
                    min_dist = _d;
                }
            }
        }


        if (e_ !=  ReebGraph::nullEdge())
        {
            /* Step 2 :
             * Pair the edges e and e' meeting n'. Change n to be n' and e to
             * be e'. Go to Step 1. */
            n = n_;
            addPairToEndPairings(e, e_, n , &unpairedEdges, &pairedEdges,
                    &EndPairs);
            e = e_;
            /* Goto Step 1 */
        }
        else
        {
            //vert = graph.getVProp(n_);
            //std::cout << "Vert n_: " << n_ << " : "<< vert;
            //std::cerr << "\n";

            //vert = graph.getVProp(n0);
            //std::cout << "Vert n0: " << n0 << " : "<< vert;
            //std::cerr << "\n" << "\n";

            assert( n_ == n0 );

            addPairToEndPairings(e1, e0, n , &unpairedEdges, &pairedEdges,
                    &EndPairs);
            addPairToEndPairings(e, e2, n , &unpairedEdges, &pairedEdges,
                    &EndPairs);

            /* Step 3 :
             * Change n0 to be any node which has at least one pair (e1,e2) of
             * edges meeting it and at least one unpaired edge e0 also meeting
             * it. Let n be set equal to n0 and e be set equal to e0, and go to
             * Step 1. If no such node n0 exists, terminate.
             */

            bool found_new_n0 = false;
                    // And add two previous last points for continuity reasons.
            list< pair<pair<Edge,Edge>,Vertex> >::iterator bestPair;
            double min_dist = (map.cols + map.rows)*(map.cols + map.rows);
            for (list< pair<pair<Edge,Edge>,Vertex> >::iterator pit
                    = EndPairs.begin(); pit!=EndPairs.end(); ++pit )
            {
                if ((*pit).first.first != ReebGraph::nullEdge()
                        && (*pit).first.second != ReebGraph::nullEdge())
                {
                    for (list<Edge>::iterator eit = unpairedEdges.begin();
                            eit!=unpairedEdges.end(); ++eit)
                    {
                        tie(_v1, _v2) = graph.getEndNodes(*eit);
                        if (_v1 == (*pit).second || _v2 == (*pit).second)
                        {
                            ReebEdge _prob_e = graph.getEProp((*pit).first.first);
                            ReebEdge _prob_eit = graph.getEProp(*eit);
                            Point2D _me0 = (_prob_e.topBoundary.front()
                                    + _prob_e.bottomBoundary.front()) / 2;
                            Point2D _me1 = (_prob_e.topBoundary.back()
                                    + _prob_e.bottomBoundary.back()) / 2;
                            Point2D _met0 = (_prob_eit.topBoundary.front()
                                    + _prob_eit.bottomBoundary.front()) / 2;
                            Point2D _met1 = (_prob_eit.topBoundary.back()
                                    + _prob_eit.bottomBoundary.back()) / 2;
                            double _d = min(min(_me0.sdist(_met0), _me0.sdist(_met1)),
                                    min(_me1.sdist(_met0), _me1.sdist(_met1)));

                            if(_d < min_dist)
                            {
                                n0 = (*pit).second;
                                e1 = (*pit).first.first;
                                e2 = (*pit).first.second;
                                e0 = *eit;
                                found_new_n0 = true;
                                min_dist = _d;
                                bestPair = pit;
                            }
                        }
                    }
                }
            }

            if (found_new_n0)
            {
                /* Remove the current pair to attach a new cycle, not necessary
                 * but it's easier for debugging. */
                EndPairs.erase(bestPair);
                n = n0;
                e = e0;
                /* Goto Step 1 */
            }
            else
                break;
                /* Terminate */
        }
    }

    /** Extract (and save) Euler cycle from edge pairings **/
    eulerCycle.clear();
    Edge lastEdge = ReebGraph::nullEdge();
    do
    {
        for (list< pair<pair<Edge,Edge>,Vertex> >::reverse_iterator pit
                = EndPairs.rbegin(); pit!=EndPairs.rend(); ++pit)
        {
            if ((*pit).first.first == lastEdge)
            {
                lastEdge = (*pit).first.second;
                if (lastEdge != ReebGraph::nullEdge())
                    eulerCycle.push_back(lastEdge);
                break;
            }
        }
    }
    while (lastEdge != ReebGraph::nullEdge());

    //cerr << "\n" << "Euler size: " << eulerCycle.size() << "\n" ;
};


/*************************************************************************
 * Function 'testValues()'
 *
 * Test method.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void ChinesePostman::testValues(RegionData& data, ReebGraph& graph)
{
    ReebGraph g = graph;

    cerr << "Test Values: \n";
    Vertex r = graph.getFirstVertex();

    std::cerr << "First Vertex: " << r << "\n";
    graph.printVertex();
    std::cerr << "\n";

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

    for (tie(vi, vi_end) = g.getVertices(); vi != vi_end; vi++)
    {
        currVertex = *vi;
        vprop = g.getVProp(currVertex);
        currDegree = g.degree(*vi);
        std::cout << "VProp: " << vprop;

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
};


/*************************************************************************
 * Function 'viewEulerGraph(QString fileName)'
 *
 * Method that created a EulerImage file.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void ChinesePostman::viewEulerGraph(QString fileName, RegionData& data,
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
    placeHolder.drawReebGraph();
    placeHolder.drawEulerTour();
    placeHolder.saveImageBuffer(fileName);
};
