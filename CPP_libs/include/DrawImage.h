#ifndef DrawImage_H_
#define DrawImage_H_

/******************************************************************************
 *
 * @author: Kelly Benson
 * @email: bensonke@email.sc.edu
 * @date: May 16, 2015
 *
 * Header file for the DrawImage calculation.
 *
**/

#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <QImage>
#include <QString>
#include <QPaintDevice>
#include <QPainter>

#include "Leo/ReebGraph.h"
#include "Leo/RegionData.h"


class DrawImage 
{

public:

    DrawImage();
    DrawImage(ReebGraph graph, RegionData data,
            std::list<Edge> eulerCycle, std::vector<Point2D> wayPoints);
    ~DrawImage(); 

    void setImageBuffer(cv::Mat* const source);
    void setImageBuffer(QImage var);    
    QImage getImageBuffer(); 

    void clearImage();
    void clearDevice();

    void drawBCDRegions();
    void drawReebGraph();
    void drawEulerTour();
		void drawWaypoints(std::vector<Point2D> wpPixels,
				int highlightID, unsigned int highlightCount, QColor color = Qt::green);

    void saveImageBuffer(QString fileName);

private:

    cv::Mat matBuffer;
    QImage imageBuffer;

    RegionData d;
    ReebGraph g;
    std::list<Edge> e;
    vector<Point2D> w;
    QPaintDevice* device;

    const static unsigned int WAYPOINT_NORMAL_RADIUS = 2; // FIXME: was 6
    const static unsigned int WAYPOINT_HIGHLIGHT_RADIUS = 4; // FIXME: was 8

    // Constants
    const static unsigned char BLACK = 0;
    const static unsigned char WHITE = 255;

};
#endif
