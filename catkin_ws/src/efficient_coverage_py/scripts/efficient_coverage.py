#!/usr/bin/env python

import rospy, os, math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from nav2d_navigator.msg import MoveToPosition2DActionResult
from tf import transformations
from PIL import Image

SCALE_FACTOR = 0.1
WALL_WIDTH_METERS = 1
WAYPOINT_RADIUS = 1

_WP_R_SQ = WAYPOINT_RADIUS*WAYPOINT_RADIUS

LEFT = math.pi
RIGHT = 0

def get_quaternion_yaw(ros_q):
    numpy_q = (ros_q.x, ros_q.y, ros_q.z, ros_q.w)
    return transformations.euler_from_quaternion(numpy_q, 'sxyz')[2]

def get_yaw_quaternion(yaw):
    numpy_q = transformations.quaternion_from_euler(0, 0, yaw)
    ros_q = Quaternion()
    ros_q.x = numpy_q[0]
    ros_q.y = numpy_q[1]
    ros_q.z = numpy_q[2]
    ros_q.w = numpy_q[3]
    return ros_q

class EffCovRobot (object):
    '''WARNING: Only create one instance per process!'''

    def __init__(self, robot_id, tour, top_wall_y):
        self.robot_id = robot_id
        self.tour = tour
        self.top_wall_y = top_wall_y
        # Init pose members
        self.x = self.y = self.yaw = None
        self.goal_x = self.goal_y = -65535
        self.going = RIGHT
        self.current_waypoint = -1
        self.done = False
        # Publish to command the stage robot
        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=1)
        # Initialize node
        rospy.init_node('ECRobot_{}'.format(robot_id))
        # Subscribe to movement results
        self.result_sub = rospy.Subscriber('MoveTo/result',
                MoveToPosition2DActionResult, self.result_callback)
        # Subscribe to pose of stage robot
        self.pose_counter = 0
        self.pose_sub = rospy.Subscriber('base_pose_ground_truth', Odometry,
                self.pose_callback, queue_size=1)

    def result_callback(self, msg):
        print('Got Result!')
        #self.send_next_waypoint()

    def pose_callback(self, msg):
        # Collect pose from message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = get_quaternion_yaw(msg.pose.pose.orientation)
        if x == self.x and y == self.y and yaw == self.yaw:
            self.stay_counter += 1
        else:
            self.stay_counter = 0
        if self.stay_counter >= 16:
            print("I can't stay here no more!")
            self.send_next_waypoint()
        self.x = x
        self.y = y
        self.yaw = yaw
        if self.done or self.stay_counter >= 16:
            self.stay_counter = 1
            return
        self.pose_counter = (self.pose_counter + 1) % 128
        if self.pose_counter == 0:
            print('x: {}, y: {}, yaw: {}'.format(self.x, self.y, self.yaw))
        dx = self.x - self.goal_x
        dy = self.y - self.goal_y
        if dx*dx + dy*dy < _WP_R_SQ:
            print('Good enough for government work!')
            self.send_next_waypoint()

    def send_goal(self, gx, gy, gyaw):
        self.goal_x = gx
        self.goal_y = gy
        m = PoseStamped()
        m.header = Header()
        m.header.stamp = rospy.Time.now()
        m.pose = Pose()
        p = Point()
        p.x, p.y = gx, gy
        m.pose.position = p
        m.pose.orientation = get_yaw_quaternion(gyaw)
        print('Sending Goal: ({}, {}); {}'.format(gx, gy, gyaw))
        self.goal_pub.publish(m)

    def send_goal_waypoint(self, i):
        tour = self.tour
        wp = tour[i]
        if i + 1 < len(tour):
            if tour[i + 1][0] > tour[i][0]:
                self.going = RIGHT
            elif tour[i + 1][0] < tour[i][0]:
                self.going = LEFT
        self.send_goal(wp[0]/10 + 1, self.top_wall_y - wp[1]/10, self.going)

    def send_next_waypoint(self):
        self.current_waypoint += 1
        if self.current_waypoint < len(self.tour):
            self.send_goal_waypoint(self.current_waypoint)
        else:
            self.done = True
            print('Tour Completed!')

    def spin(self):
        rospy.sleep(2)
        self.send_next_waypoint()
        rospy.spin()


def parse_tours(tour_file):
    import json
    tours = []
    current_tour = None
    for line in tour_file:
        line = line.strip()
        if line.startswith('Start'):
            # Initialize current tour
            current_tour = []
        elif line.startswith('End'):
            # Push current tour
            tours.append(current_tour)
            current_tour = None
        elif line:
            # Hacky. Parse line of points
            line = line.replace(') (', '), (')
            line = '[' + line + ']'
            line = line.replace('(', '[').replace(')', ']')
            current_tour.extend(json.loads(line))
    return tours

def generate_tours(input_dir, input_name, robot_count=1):
    import tempfile, subprocess, shutil
    # Create a temporary directory for afrl-oxdel to use
    tmpdir = tempfile.mkdtemp()
    tours = None
    try:
        # Call afrl-oxdel in the temp directory
        cmd_spec = ['/usr/bin/env', 'afrl-oxdel',
            os.path.realpath(input_dir), input_name, robot_count]
        print('Calling {}\n  in {}:'.format(cmd_spec, tmpdir))
        subprocess.call(cmd_spec, cwd=tmpdir)
        # Parse the tour lines file
        tour_file_path = os.path.join(tmpdir, 'tourLines.txt')
        with open(tour_file_path) as tour_file:
            tours = parse_tours(tour_file)
    finally:
        # Always remove the temporary directory
        shutil.rmtree(tmpdir)
    return tours

def main():
    # Get arguments without ROS's extra args
    argv = rospy.myargv()
    if len(argv) != 5:
        print(('Usage: {} <input dir> <input name> <total robot #>'
            ' <this robot #>').format(argv[0]))
        return 1
    # Parse arguments
    robot_id = int(argv[4])
    image_path = os.path.join(argv[1], argv[2])
    top_wall_y = Image.open(image_path).size[1]/10 + 1
    # Call afrl-oxdel and parse tour
    tour = generate_tours(*argv[1:4])[robot_id]
    print('{}: TOUR:\n  {}'.format(argv[0], tour))
    # Create ROS node
    ecr = EffCovRobot(robot_id, tour, top_wall_y)
    # Go! Main loop.
    ecr.spin()

if __name__ == '__main__':
    raise SystemExit(main())
