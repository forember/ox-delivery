#ifndef BCD_H_
#define BCD_H_

/******************************************************************************
 *
 * @author: Kelly Benson
 * @email: bensonke@email.sc.edu
 * @date: May 16, 2015
 *
 * Header file for the BCD calculation.
 *
**/

#include <iostream>
#include <sstream> 
#include <QString>
#include "Leo/ReebGraph.h"
#include "Leo/RegionData.h"
#include "OpenImage.h"
#include "DrawImage.h"

class BCD {

public:

    BCD();
    BCD(string directory, string fileName, RegionData& data, ReebGraph& graph);
    ~BCD(); 

    void buildBCD(RegionData& data, ReebGraph& graph)
        throw (const std::string&);
    void viewReebGraph(QString fileName, RegionData data, ReebGraph graph);
    void printBCDInfo(ReebGraph graph);

private:

    OpenImage imageLoader;

    // Constants
    const static unsigned char BLACK = 0;
    const static unsigned char WHITE = 255;
};
#endif
