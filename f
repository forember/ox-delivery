./Leo_libs/include/ReebGraph.h:165:    std::pair<Edge_Iter, Edge_Iter> getEdges();
./Leo_libs/include/ReebGraph.h:170:    std::pair<Out_Edge_Iter, Out_Edge_Iter> getEdges(Vertex v);
./Leo_libs/include/ReebGraph.h:171:    std::pair<Out_Edge_Iter, Out_Edge_Iter> getEdges(unsigned int Vid);
./Leo_libs/src/ReebGraph.cpp:278:std::pair<Edge_Iter, Edge_Iter> ReebGraph::getEdges() 
./Leo_libs/src/ReebGraph.cpp:285: * Function 'getEdges(Vertex v)'
./Leo_libs/src/ReebGraph.cpp:297:std::pair<Out_Edge_Iter, Out_Edge_Iter> ReebGraph::getEdges(Vertex v)
./Leo_libs/src/ReebGraph.cpp:304: * Function 'getEdges(unsigned int Vid)'
./Leo_libs/src/ReebGraph.cpp:316:std::pair<Out_Edge_Iter, Out_Edge_Iter> ReebGraph::getEdges(unsigned int Vid)
./Leo_libs/src/ReebGraph.cpp:318:    return getEdges(Vpointers[Vid]);
./Leo_libs/src/ReebGraph.cpp:670:            for (tie(oi, oi_end) = g.getEdges(currVertex); oi != oi_end; oi++)
./Leo_libs/src/ReebGraph.cpp:684:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
./Leo_libs/src/ReebGraph.cpp:979:            for (tie(oi, oi_end) = g.getEdges(currVertex); oi != oi_end; oi++) 
./Leo_libs/src/ReebGraph.cpp:994:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
./Leo_libs/src/ReebGraph.cpp:1021:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
./CPP_libs/src/ChinesePostman.cpp:204:    tie(ei, ei_end) = graph.getEdges();
./CPP_libs/src/ChinesePostman.cpp:241:        tie(ei, ei_end) = graph.getEdges();
./CPP_libs/src/ChinesePostman.cpp:293:    tie(ei, ei_end) = graph.getEdges();
./CPP_libs/src/ChinesePostman.cpp:400:    tie(oi,oi_end) = graph.getEdges(r);
./CPP_libs/src/ChinesePostman.cpp:414:    for( tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ++ei)
./CPP_libs/src/ChinesePostman.cpp:661:            for (tie(oi, oi_end) = g.getEdges(currVertex); oi != oi_end; oi++)
./CPP_libs/src/DrawImage.cpp:223:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++)
./CPP_libs/src/DrawImage.cpp:317:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ++ei)
./WayPoints_libs/src/WayPoints.cpp:378:    tie(oi, oi_end) = graph.getEdges(_currVertex);
./WayPoints_libs/src/WayPoints.cpp:425:        for(tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ei++)
./WayPoints_libs/src/WayPoints.cpp:711:        tie(oi, oi_end) = graph.getEdges(_currVertex);
./WayPoints_libs/src/WayPoints.cpp:865:    for (tie(ei, ei_end) = g.getEdges(closestVertex); ei != ei_end; ei++)
grep: input file ‘./f’ is also the output
./BCD_libs/src/BCD.cpp:229:    for (tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ++ei)
./BCD_libs/src/BCD.cpp:251:    for (tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ei++)
./BCD_libs/src/BCD.cpp:259:        tie(oi, oi_end) = graph.getEdges(*vi);
./BCD_libs/src/BCD.cpp:292:    for (tie(ei, ei_end) = graph.getEdges(); ei != ei_end; ei++)
./BCD_libs/src/DrawImage.cpp:210:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++)
./BCD_libs/src/DrawImage.cpp:304:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ++ei)
./KCPP/src/KChinesePostmen.cpp:191:    for (tie(ei, ei_end) = g.getEdges(); ei != ei_end; ei++) 
