/********************************************************************
 * @author: Kelly Benson
 *

Commands to remember:

roscore 
rosrun stage_ros stageros src/efficient_coverage/world/img0.world 
rosrun efficient_coverage efficient_coverage src/efficient_coverage/inputs img0.png 2

**/

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <cmath>
#include <sstream>
#include <fstream>
#include <set>
#include <time.h>
#include <ctime>
#include <cstdlib>
#include <QImage>
#include <QString>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

#include <ChinesePostman/ChinesePostman.h>
#include <ChinesePostman/DrawImage.h>
#include <Boustrophedon/BCD.h>
#include <Boustrophedon/DrawImage.h>
#include <Leo/Point2D.h>
#include <Leo/ReebGraph.h>
#include <Leo/RegionData.h>
#include <WayPoints/WayPoints.h>
#include <WayPoints/DrawImage.h>
#include <KCPP/KChinesePostmen.h>
#include <KCPP/FredericksonKCPP.h>

using namespace Qt;
using namespace boost::posix_time;
using namespace std;

class EfficientCoverage
{

public:

    const static double MIN_SCAN_ANGLE_RAD = -60.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +60.0/180*M_PI;

    int id;
    ros::Publisher commandPub;
    ros::Subscriber poseSub; 

    double x; 
    double y; 
    double heading; 
    Point2D currentPoint;

    ros::Time rotateStartTime;
    ros::Duration rotateDuration; 

    ros::Time moveStart;
    ros::Duration moveDuration; 

    vector<Point2D> wayPoints;

    int count;

    // Tunable motion controller parameters
    const static double PROXIMITY_RANGE_M = 1.0;
    const static double FORWARD_SPEED_MPS = 10.0;
    const static double ROTATE_SPEED_RADPS = M_PI/2;
    const static int SPIN_RATE_HZ = 15;
 
    const static char CELL_OCCUPIED = 0;
    const static char CELL_UNKNOWN = 86;
    const static char CELL_FREE = 172;
    const static char CELL_ROBOT = 255;


/*************************************************************************
 * Constructor
 *
 * Construct a new EfficientCoverage object and hook up
 * this ROS node to the simulated robot's pose and velocity control.
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   ros::NodeHandle& nh - node handler
 *   vector<Point2D>& points - points to follow
 *
**/
EfficientCoverage(ros::NodeHandle& nh, vector<Point2D>& points)
{

    //Publisher for velocity commands
    commandPub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);
    /*commandPub1 = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);
    commandPub2 = nh.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 1);
    commandPub3 = nh.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 1);
*/

    //Subscriber to robot position
    poseSub = nh.subscribe("/robot_0/base_pose_ground_truth", 1, \
        &EfficientCoverage::poseCallback, this);
    /*
    poseSub1 = nh.subscribe("/robot_1/base_pose_ground_truth", 1, \
        &EfficientCoverage::poseCallback, this);
    poseSub2 = nh.subscribe("/robot_2/base_pose_ground_truth", 1, \
        &EfficientCoverage::poseCallback, this);
    poseSub3 = nh.subscribe("/robot_3/base_pose_ground_truth", 1, \
        &EfficientCoverage::poseCallback, this);*/
 
    wayPoints = points;
    count = 0;

};


/*************************************************************************
 * Function 'poseCallback()'
 *
 * Processes incoming ground truth robot pose message
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   nav_msgs::Odometry::ConstPtr& msg - the position of the robot
 *
**/
 //Think the issue is the posecallback not being called right
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    heading = tf::getYaw(msg->pose.pose.orientation);
/*
    cerr << "x: " << x << "\n";
    cerr << "y: " << y << "\n";
    cerr << "z: " << heading * 180 / M_PI << "\n";
*/
    currentPoint = Point2D(x,y);
};

        
/*************************************************************************
 * Function 'move()'
 *
 * Sends a velocity command to the robot. 
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   double linearVelMPS - how fast to move forward
 *   double angularVelRadPS - how fast to turn
 *
**/
void move(double linearVelMPS, double angularVelRadPS)
{
    geometry_msgs::Twist msg; 

    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
};


/*************************************************************************
 * Function 'move()'
 *
 * Sends a velocity command to the robot. 
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   double linearVelMPS - how fast to move forward
 *   double angularVelRadPS - how fast to turn
 *
**/
 /*
void move_mult(int robot_id, double linearVelMPS, double angularVelRadPS)
{
    geometry_msgs::Twist msg; 

    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
};*/



/*************************************************************************
 * Function 'moveTo(double, double, double)'
 *
 * Method that takes in a way points coordinates and the distance to
 * it and then tells the robot to move there. 
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   double wayX - x coordinate of the waypoint
 *   double wayY - y coordinate of the waypoint
 *   double distance - distance to the waypoint
 *
**/
void moveTo(double wayX, double wayY, double distance)
{

    double dx = wayX - x;
    double dy = wayY - y;

    cerr << "MOVETO:" << endl;
    cerr << "way: (" << wayX << ", " << wayY << ")" << endl;
    cerr << "distance: " << distance << endl;
    cerr << "calculated: " << sqrt(dx*dx + dy*dy) << endl;

    //using atan2 to find the angle between two points in degrees. 
    //angle in radians
    //wrong in that it only considers based on where 0 is. If its
    //  already facing 0 then theres an issue. 
    double angle = atan2(dy,dx);

    cerr << "Angle Degrees: " << angle * 180 / M_PI << endl;
    cerr << "Angle Radians: " << angle << endl << endl;

    rotateStartTime = ros::Time::now();
    rotateDuration = ros::Duration(abs(angle) / ROTATE_SPEED_RADPS);

    //Angle not working right
    //angle is positive spin in the positive direction
    if (angle > 0)
    {
        while (ros::Time::now() - rotateStartTime < rotateDuration)
        {
            move(0, ROTATE_SPEED_RADPS);
            ros::spinOnce();
        }
    }
    //angle is negative spin in the negative direction.
    else if (angle < 0)
    {
        while (ros::Time::now() - rotateStartTime < rotateDuration)
        {
            move(0, -ROTATE_SPEED_RADPS);
            ros::spinOnce();
        }
    }

    moveStart = ros::Time::now();
    moveDuration = ros::Duration(distance / FORWARD_SPEED_MPS);

    //move forward - done
    while (ros::Time::now() - moveStart < moveDuration)
    {
        move(FORWARD_SPEED_MPS, 0);
        ros::spinOnce();
    }
}


/*************************************************************************
 * Function 'followPath()'
 *
 * Method that gets a robot to follow a path of waypoints. 
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void followPath()
{
    double wayX, wayY, distance;
    Point2D nextPoint;

    //robot is in position and ready to move around the waypoints
    if(count < wayPoints.size())
    {           
        nextPoint = wayPoints.at(count);    
        wayX = nextPoint.xcoord();
        wayY = nextPoint.ycoord();
        distance = currentPoint.distance(nextPoint);

        cerr << "Current Position: " << currentPoint.xcoord() << ", " << currentPoint.ycoord() << "\n";
        cerr << "Next WayPoint: Count " << count << " (" << wayX << ", " << wayY << ") \n";
        cerr << "Distance" << ": " << distance << "\n";

        //tell the robot to move to the waypoint
        moveTo(wayX, wayY, distance);        
        count++;
    }
};


/*************************************************************************
 * Function 'spin()'
 *
 * Main FSM loop for ensuring that ROS messages are
 * processed in a timely manner, and also for sending
 * velocity controls to the simulated robot based on 
 * the FSM state
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void spin() 
{
    sleep(10);

    ros::Rate rate(1); // Specify the FSM looprate in Hz


    //to clear up the position data
    for(int i = 0; i < 100; i++)
    {
        ros::spinOnce();
    }

    // Keep spinning loop until user presses Ctrl+C
    while (ros::ok()) 
    {
        ros::spinOnce();
        followPath();
        rate.sleep(); 
    }

};


/*************************************************************************
 * Function 'spin_multiple()'
 *
 * Main FSM loop for ensuring that ROS messages are
 * processed in a timely manner, and also for sending
 * velocity controls to the simulated robot based on 
 * the FSM state. Alternate function for multiple robots. 
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void spin_multiple() 
{
    ros::Rate rate(1); // Specify the FSM looprate in Hz

    //to clear up the position data
    for(int i = 0; i < 100; i++)
    {
        ros::spinOnce();
    }

    // Keep spinning loop until user presses Ctrl+C
    while (ros::ok()) 
    {
        ros::spinOnce();
        followPath();
        rate.sleep(); 
    }

};


};


/*************************************************************************
 * Begin Main
**/

//number of robots
int m_k;
int m_vertices;
bool m_cppSolved;

std::string m_image; 
std::string m_directory;

std::vector<EulerTour> m_tours;
std::vector<Graph> m_graph;

KChinesePostmen* m_kcpp;
ChinesePostman* m_cpp;


int parseInputArgs(int argc, char **argv, std::string& directory, std::string& image, int& k)
{
    bool printUsage = false;
    // Parse and validate command line input arguments
    if (argc <= 3) 
    {
        printUsage = true;
    }

    else 
    {
        try 
        {
            directory = boost::lexical_cast<string>(argv[1]);
            image = boost::lexical_cast<string>(argv[2]);
            k = boost::lexical_cast<int>(argv[3]);

            if (directory.size() <= 0) 
            {
                printUsage = true; 
            }

            else if (image.size() <= 0) 
            {
                printUsage = true;
            }
        }

        catch (std::exception err)
        {
            printUsage = true;
        }
    }

    if (printUsage)
    {
        std::cerr << "Usage: " << argv[0] << " [IMAGE_DIRECTORY] [IMAGE_NAME] [NUMBER_OF_ROBOTS]" << std::endl;
        return -1;
    }

    return 0;
}


/*************************************************************************
 * Function 'runkCPP'
 *
 * Runs the kcpp algorithm. Taken from the controller file from UseLibs 
 * and KCPP.
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void runkCPP(int k, ReebGraph& graph, std::list<Edge>& eulerCycle)
{
    std::cerr << "runkCPP 1 \n";
    assert(m_cppSolved && "runCPP must be called before runkCPP");
    std::cerr << "runkCPP 2 \n";

    //runs out of memory here for some reason
    try
    {
        std::cerr << "k? \n";
        m_kcpp = new FredericksonKCPP(eulerCycle, graph, k);
        std::cerr << "runkCPP 3 \n";
        m_kcpp->solve();
    }

    catch (const std::string& err) 
    {
        std::cout << "runkCPP meh";
    }

    std::cerr << "runkCPP 4 \n";
}


/*************************************************************************
 * Function 'checkInputParams()'
 *
 * Checks the input parameters. Taken from the controller file from UseLibs 
 * and KCPP.
 *
 * Returns:
 *   None 
 *
 * Parameters:
 *   None
 *
**/
void checkInputParams(const std::string& directory, const std::string& image, int k)
{
    std::string message = "";

    if(k<1) 
    {
        message = "ERR:Number of robots must be positive integer!";
        throw std::invalid_argument(message);
    }

    /**TODO: Add also check for valid image format*/
    if( !std::ifstream((directory + "/" + image).c_str())) 
    {
        message = "ERR:There is no image at this path:  " + directory + "/" +  image;
        throw std::invalid_argument(message);
    }
}


int main(int argc, char **argv) 
{
    string image;
    string directory;
    QImage imageBuffer;
    int k = 1;

    RegionData data;
    ReebGraph graph;
    std::list<Edge> eulerCycle;
    vector<Point2D> wayPoints;


    if(parseInputArgs(argc, argv, directory, image, k) == -1) 
    {
        return EXIT_FAILURE;
    }

    std::cout << "Controller's run executed\n";
    checkInputParams(directory, image, k);
    std::cerr << "here...\n";

    m_directory = directory;
    m_image = image;
    m_k = k;

    //-------------------- BCD call -------------------------
    std::cerr << "BCD is starting...\n";
#ifdef DEBUG
    std::cerr << "BCD is starting...\n";
#endif

    BCD bcd(directory, image, data, graph);

#ifdef DEBUG
        std::cerr << "BCD is completed\n"; 
#endif
        std::cerr << "BCD is completed\n"; 

        //----------------- BCD call ended ----------------------

    ChinesePostman m_cpp(data, graph, eulerCycle, wayPoints);
    
std::cerr << "After running cpp, checking the m_cpp var ...\n";
#ifdef DEBUG

    std::cout << "After running cpp, checking the m_cpp var ...\n";
    std::cout << eulerCycle;
    std::cout << graph;
    std::cout << std::endl;
    std::cout << "Checking is passed!\n";
    m_cppSolved = true;

#endif
    std::cerr << "Checking is passed!\n";

    m_cppSolved = true;
    if(k > 1) 
    {
        std::cerr << "k greater \n";
        runkCPP(m_k, graph, eulerCycle);        
        std::cout << std::endl;
        m_kcpp->printEulerianTours();
        std::cout << std::endl;
    }
 
    else 
    {
        std::cerr << "k not greater \n";
        m_tours.push_back(eulerCycle);
    }

    WayPoints waypoints(data, graph, eulerCycle, wayPoints);

    cerr << "\n";
    cerr << "START" << "\n";
    vector<Point2D>::iterator iter;
    for(iter = wayPoints.begin(); iter != wayPoints.end(); ++iter)
    {
        cout << (*iter) << " ";;
    }
    cerr << "\n";
    cerr << "END" << "\n"<< "\n";

    ros::init(argc, argv, "efficient_coverage"); // Initiate ROS node
    ros::NodeHandle nh; // Create default handle

    // Create new EfficientCoverage object
    EfficientCoverage robbie(nh, wayPoints); 
    robbie.spin();   

    return 0;
};
