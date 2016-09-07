./Leo_libs/include/ReebGraph.h:51:    double travelCost;
./Leo_libs/include/ReebGraph.h:59:        travelCost(-1) {};
./Leo_libs/include/ReebGraph.h:66:        travelCost(-1) {};
./Leo_libs/include/ReebGraph.h:78:    //FIXME: <N> set travel cost
./Leo_libs/include/ReebGraph.h:79:    void setTravelCost(const double tCost) {
./Leo_libs/include/ReebGraph.h:80:        travelCost = tCost;
./Leo_libs/include/ReebGraph.h:133:    /* Computes the travel - Euclidean Distance, for each edge in the graph
./Leo_libs/include/ReebGraph.h:135:    void updateTravelDistances();
./Leo_libs/src/ReebGraph.cpp:143:/* Computes the travel - Euclidean Distance, for each edge in the graph
./Leo_libs/src/ReebGraph.cpp:145:void ReebGraph::updateTravelDistances()
./Leo_libs/src/ReebGraph.cpp:154:        g[e].setTravelCost(p1.distance(p2));
./Leo_libs/src/ReebGraph.cpp:156:        std::cout << "distance for edge -> " << g[e].Eid << " " << g[e].travelCost << std::endl;
./CPP_libs/src/ChinesePostman.cpp:369:    graph.updateTravelDistances(); //added
./WayPoints_libs/src/WayPoints.cpp:520: *   vector<Point2D> - vector of way points to travel through
grep: input file ‘./f’ is also the output
./KCPP/include/KChinesePostmen.h:112:    /*predecessors computed accounting the travel weight, i.g. by euclidean distance*/
./KCPP/include/KChinesePostmen.h:113:    std::vector<kcpp::Vertex> m_travelPredecessors;
./KCPP/include/KChinesePostmen.h:114:    std::vector<double> m_shortTravelDistances; // s(v1, i) = m_..[i]
./KCPP/include/KChinesePostmen.h:115:    std::vector<kcpp::Edge> m_travelEdgePredecessor;
./KCPP/src/KChinesePostmen.cpp:99: * based on area and travel costs 
./KCPP/src/KChinesePostmen.cpp:110:    m_shortTravelDistances.resize(boost::num_vertices(graph));
./KCPP/src/KChinesePostmen.cpp:113:    m_travelEdgePredecessor.resize(boost::num_vertices(graph));
./KCPP/src/KChinesePostmen.cpp:116:    m_travelPredecessors.resize(boost::num_vertices(graph));
./KCPP/src/KChinesePostmen.cpp:121:    /**Computing shortest paths based on area and travel costs*/
./KCPP/src/KChinesePostmen.cpp:132:    //"-------------------------Based on Travel Cost----------------------------------\n";
./KCPP/src/KChinesePostmen.cpp:134:            boost::weight_map(boost::get(&ReebEdge::travelCost,graph))
./KCPP/src/KChinesePostmen.cpp:135:            .distance_map(boost::make_iterator_property_map(m_shortTravelDistances.begin(), boost::get(boost::vertex_index,graph)))
./KCPP/src/KChinesePostmen.cpp:136:            .visitor(make_shortest_path_recorder(&m_travelEdgePredecessor[0]))
./KCPP/src/KChinesePostmen.cpp:137:            .predecessor_map(boost::make_iterator_property_map(m_travelPredecessors.begin(), boost::get(boost::vertex_index,graph)))
./KCPP/src/KChinesePostmen.cpp:209:            //otherwise replace it with an edge that has min travel cost
./KCPP/src/KChinesePostmen.cpp:214:            //if (rEdge.travelCost < retrREdge.travelCost) { 
./KCPP/src/KChinesePostmen.cpp:247:        curr = m_travelPredecessors.at(curr);
./KCPP/src/KChinesePostmen.cpp:309:        std::cout << " travel edges ( ";
./KCPP/src/KChinesePostmen.cpp:328:        std::cout << " travel edges ( ";
./KCPP/src/KChinesePostmen.cpp:355:        std::cout << edge.Eid << " area " << edge.area << " travel cost " << edge.travelCost << std::endl;
./KCPP/src/CAC.cpp:65:        double C_R_Vl_j = m_shortTravelDistances.at(v_last);
./KCPP/src/CAC.cpp:80:            c_R = c_subR + m_shortTravelDistances.at(v_last) + m_shortTravelDistances.at(v2); 
./KCPP/src/CAC.cpp:91:                double sum_1 = r_j + m_shortTravelDistances.at(v1);
./KCPP/src/CAC.cpp:92:                double sum_2 = edge.area  - r_j + m_shortTravelDistances.at(v2);
./KCPP/src/CAC.cpp:138:        // FIXME should be changed to m_shorTravelDistance
./KCPP/src/CAC.cpp:149:        double S_V1_Vij = m_shortTravelDistances.at(v1);
./KCPP/src/CAC.cpp:150:        double S_Vij_1_V1 = m_shortTravelDistances.at(v2);
./KCPP/src/CAC.cpp:172:        double C_R_Vl_j = m_shortTravelDistance->at(lastVertex);
./KCPP/src/CAC.cpp:190:        // FIXME should be changed to m_shorTravelDistance
./KCPP/src/CAC.cpp:192:        if(2*r_j + m_shortTravelDistance->at(lastVertex)
./KCPP/src/CAC.cpp:194:                + m_shortTravelDistance->at(m_optimalPath.at(lastPathIndex).endNode)) {
./KCPP/src/FredericksonKCPP.cpp:82:    double pathTravelCost = 0.0;
./KCPP/src/FredericksonKCPP.cpp:92://      pathTravelCost += edge.travelCost;
./KCPP/src/FredericksonKCPP.cpp:100:    std::cout << pathTravelCost << std::endl;
./KCPP/src/FredericksonKCPP.cpp:132:        double C_R_Vl_j = m_shortTravelDistances.at(v_last);
./KCPP/src/FredericksonKCPP.cpp:147:            c_R = c_subR + m_shortTravelDistances.at(v_last) + m_shortTravelDistances.at(v2); 
./KCPP/src/FredericksonKCPP.cpp:158:                double sum_1 = r_j + m_shortTravelDistances.at(v1);
./KCPP/src/FredericksonKCPP.cpp:159:                double sum_2 = edge.area  - r_j + m_shortTravelDistances.at(v2);
./KCPP/src/FredericksonKCPP.cpp:213:        double S_V1_Vij = m_shortTravelDistances.at(v1);
./KCPP/src/FredericksonKCPP.cpp:214:        double S_Vij_1_V1 = m_shortTravelDistances.at(v2);
./KCPP/src/FredericksonKCPP.cpp:236:        double C_R_Vl_j = m_shortTravelDistance->at(lastVertex);
./KCPP/src/FredericksonKCPP.cpp:254:        // FIXME should be changed to m_shorTravelDistance
./KCPP/src/FredericksonKCPP.cpp:256:        if(2*r_j + m_shortTravelDistance->at(lastVertex)
./KCPP/src/FredericksonKCPP.cpp:258:                + m_shortTravelDistance->at(m_optimalPath.at(lastPathIndex).endNode)) {
