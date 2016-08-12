#include "DrawImage.h"

/******************************************************************************
 *
 * @maintainer: Kelly Benson
 * @email: bensonke@email.sc.edu
 * @date: May 16, 2015
 *
 * This file is for the DrawImage calculation and work behind that. When a 
 * DrawImage object is created it should ask for an image file and then 
 * calculate the DrawImage/draw it.
 * 
**/


/*************************************************************************
 * Constructor
 *
**/
DrawImage::DrawImage() {};


/*************************************************************************
 * Alt Constructor
 *
**/
DrawImage::DrawImage(ReebGraph graph, RegionData data,
        std::list<Edge> eulerCycle, std::vector<Point2D> wayPoints)
{
    g = graph;
    d = data;
    e = eulerCycle;
    w = wayPoints;

    // If the device is empty set it to the image buffer. QPaintDevice as a
    // class takes in things like widgets and QImages to draw overtop. So the
    // paint device has to be set to a QImage. Especially since QPainter draws
    // overtop paint devices.
    setImageBuffer(&(d.image)); 
    device = &imageBuffer;
};


/*************************************************************************
 * Destructor
 *
**/
DrawImage::~DrawImage() 
{
/*
    clearDevice();
    delete device;
*/
};


/*************************************************************************
 * Function 'setImageBuffer()'
 *
 * Method that sets the QImage variable imageBuffer from the image file. 
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   cv::Mat* source - Image being converted to QImage format
 *
**/
void DrawImage::setImageBuffer(cv::Mat* const source)
{
    matBuffer = cv::Mat();
    QImage::Format imgFormat = QImage::Format_Invalid;
    if (source == NULL) 
    {
        return;
    }

    if ((source->depth() != CV_8U) && (source->depth() != CV_8S)) 
    {
        std::cout<< "ERROR > Image depth is not 8U/8S!";
        return;
    }

    else 
    {
        switch(source->channels()) 
        {
            case 1:
                cv::cvtColor(*source, matBuffer, CV_GRAY2RGB);
                imgFormat = QImage::Format_RGB888;
                break;

            case 3:
                cv::cvtColor(*source, matBuffer, CV_BGR2RGB);
                imgFormat = QImage::Format_RGB888;
                break;

            case 4:
                cv::cvtColor(*source, matBuffer, CV_BGR2RGB);
                imgFormat = QImage::Format_ARGB32;
                break;

            default:
                imgFormat = QImage::Format_Invalid;
                std::cout << "Format Invalid";
                return;
        }
    }

    imageBuffer = QImage(matBuffer.data, matBuffer.cols, matBuffer.rows,
            matBuffer.step, imgFormat);
};


/*************************************************************************
 * Function 'setImageBuffer(QImage)'
 *
 * Method that can initialize the QImage imageBuffer. Allows QImages
 * to be passed into this class.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   var - QImage being taken in and set to the imageBuffer
 *
**/
void DrawImage::setImageBuffer(QImage var) 
{
    imageBuffer = var;
}


/*************************************************************************
 * Function 'getImageBuffer()'
 *
 * Method that gets the QImage variable imageBuffer and returns it so it
 * can be used elsewhere.
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   source - Image source being converted to QImage format
 *
**/
QImage DrawImage::getImageBuffer() 
{
    return imageBuffer;
};


/*************************************************************************
 * Function 'clearImage()'
 *
 * Method that clears the imageBuffer
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void DrawImage::clearImage() 
{
    imageBuffer = QImage();
};


/*************************************************************************
 * Function 'clearDevice()'
 *
 * Method that clears the QPaintDevice
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void DrawImage::clearDevice() 
{
    device = NULL;
};

/*************************************************************************
 * Function 'drawWaypoints()'
 *
 * Method that makes the waypoints and returns them. 
 *
 * Returns:
 *   None
 *
 * Parameters:
 *   None
 *
**/
void DrawImage::drawWaypoints(std::vector<Point2D> wpPixels, int highlightID,
        unsigned int highlightCount)
{
    QColor lightGreen = QColor(128, 255, 128);
    QColor green = Qt::green;
    QColor yellow = Qt::yellow;
    QColor orange = QColor(255, 128, 0);

    QPainter painter;
    QPolygon path;
    painter.begin(device);

    QPen pen;
    pen.setColor(green);
    pen.setWidth(3);
    painter.setPen(pen);
    painter.setBrush(green);

    // Draw all waypoints
    double currXPixel, currYPixel;
    QPoint currPoint;

    //Loops through the passed in Points
    std::vector<Point2D>::iterator wit;
    for (wit = wpPixels.begin(); wit != wpPixels.end(); wit++)
    {
        currXPixel = round(wit->xcoord());
        currYPixel = round(wit->ycoord());
        currPoint = QPoint(currXPixel, currYPixel);
        path << currPoint;

        pen.setColor(yellow);
        painter.setPen(pen);
        painter.setBrush(yellow);

        //drawing the yellow circles for the waypoints
        painter.drawEllipse(currPoint, WAYPOINT_NORMAL_RADIUS,
                WAYPOINT_NORMAL_RADIUS);
        pen.setColor(orange);
        painter.setPen(pen);
        painter.setBrush(orange);

        //drawing the orange outlines to those circles
        painter.drawEllipse(currPoint, WAYPOINT_NORMAL_RADIUS+2,
                WAYPOINT_NORMAL_RADIUS+2);
        pen.setColor(green);
        painter.setPen(pen);
        painter.setBrush(green);


        //Don't actually see this codes effect anywhere?
        //  Maybe i need to try a differernt image?
        if (wit == wpPixels.begin())
        {
                pen.setColor(Qt::darkBlue);
                painter.setPen(pen);
                painter.setBrush(Qt::darkBlue);
                painter.drawEllipse(currPoint, WAYPOINT_NORMAL_RADIUS-2,
                        WAYPOINT_NORMAL_RADIUS-2);
                pen.setColor(Qt::cyan);
                painter.setPen(pen);
                painter.setBrush(Qt::cyan);
        }
    }

    pen.setColor(green);
    pen.setWidth(8);
    painter.setPen(pen);
    painter.setBrush(green);
    painter.drawPolyline(path);
    pen.setColor(QColor(lightGreen));
    pen.setWidth(4);
    painter.setPen(pen);
    painter.setBrush(lightGreen);
    painter.drawPolyline(path);

    //Looping through passed in points
    for (wit = wpPixels.begin(); wit != wpPixels.end(); wit++) 
    {
        currXPixel = round(wit->xcoord());
        currYPixel = round(wit->ycoord());
        currPoint = QPoint(currXPixel, currYPixel);

        pen.setColor(orange);
        painter.setPen(pen);
        painter.setBrush(orange);

        //drawing orange outer circle
        painter.drawEllipse(currPoint, WAYPOINT_NORMAL_RADIUS+2,
                WAYPOINT_NORMAL_RADIUS+2);
        pen.setColor(yellow);
        painter.setPen(pen);
        painter.setBrush(yellow);

        //drawing yellow inner circle
        painter.drawEllipse(currPoint, WAYPOINT_NORMAL_RADIUS-2,
                WAYPOINT_NORMAL_RADIUS-2);
    }

    // Draw highlighted waypoints if necessary
    if (highlightID >= 0 && highlightID < (int) wpPixels.size())
    {
        if (highlightID+highlightCount >= wpPixels.size())
        {
            highlightCount = wpPixels.size() - highlightID;
        }

        painter.setPen(Qt::white);
        painter.setBrush(Qt::white);
        path.clear();

        for (int currID = highlightID; highlightCount > 0;
                currID++, highlightCount--) 
        {
            currXPixel = round((wpPixels)[currID].xcoord());
            currYPixel = round((wpPixels)[currID].ycoord());
            currPoint = QPoint(currXPixel, currYPixel);
            path << currPoint;
            painter.drawEllipse(currPoint, WAYPOINT_HIGHLIGHT_RADIUS, \
                    WAYPOINT_HIGHLIGHT_RADIUS);
            if (currID == highlightID) 
            {
                // Draw first waypoint with different color
                painter.setPen(Qt::darkGray);
                painter.setBrush(Qt::darkGray);
                painter.drawEllipse(currPoint, WAYPOINT_HIGHLIGHT_RADIUS-2, \
                                WAYPOINT_HIGHLIGHT_RADIUS-2);
                painter.setPen(Qt::white);
                painter.setBrush(Qt::white);
            }
        }

        painter.drawPolyline(path);
    }

    painter.end();
};


/*************************************************************************
 * Function 'saveImageBuffer(QString)'
 *
 * Method saves the current QImage to a file in the output folder.
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   fileName - name of the file to be saved as
 *
**/
void DrawImage::saveImageBuffer(QString fileName) 
{
    if (imageBuffer.isNull()) 
    {
        return;
    }

    imageBuffer.save(fileName);
};
