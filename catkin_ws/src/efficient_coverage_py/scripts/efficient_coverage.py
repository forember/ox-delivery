#!/usr/bin/env python

import rospy, os, math, collections
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from nav2d_navigator.msg import MoveToPosition2DActionResult
from tf import transformations
from PIL import Image

WALL_WIDTH_PIXELS = 10
WAYPOINT_RADIUS = 1.5
ROBOT_BERTH_RADIUS = 1

_WP_R_SQ = WAYPOINT_RADIUS*WAYPOINT_RADIUS
_RB_R_SQ = ROBOT_BERTH_RADIUS*ROBOT_BERTH_RADIUS

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


PoseTuple = collections.namedtuple('PoseTuple', ['x', 'y', 'yaw'])


class PoseCallback (object):
    def __init__(self, i, _pose_callback):
        self.i = i
        self._pose_callback = _pose_callback

    def pose_callback(self, msg):
        self._pose_callback(self.i, msg)


class EffCovRobot (object):
    '''WARNING: Only create one instance per process!'''

    def __init__(self, robot_id, tour, top_wall_y, scale_factor):
        self.robot_id = robot_id
        self.tour = tour
        self.top_wall_y = top_wall_y
        self.scale_factor = scale_factor
        # Init pose members
        self.x = (tour[0][0] + WALL_WIDTH_PIXELS)*self.scale_factor
        self.y = self.top_wall_y - (tour[0][1]
                + WALL_WIDTH_PIXELS)*self.scale_factor
        self.yaw = 0
        self.poses = [PoseTuple(-8, -8, 0)] * 4
        self.too_close = False
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
        self.pose_subs = []
        for i in range(4):
            print('/robot_{}/base_pose_ground_truth'.format(i))
            pc_obj = PoseCallback(i, self.pose_callback)
            self.pose_subs.append(rospy.Subscriber(
                    '/robot_{}/base_pose_ground_truth'.format(i),
                    Odometry, pc_obj.pose_callback, queue_size=1))
        print('My ID is {}.'.format(self.robot_id))

    def result_callback(self, msg):
        print('Got Result!')
        #self.send_next_waypoint()

    def pose_callback(self, i, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = get_quaternion_yaw(msg.pose.pose.orientation)
        self.poses[i] = PoseTuple(x, y, yaw)
        if i == self.robot_id:
            if self.my_pose_callback(msg):
                for i in range(4):
                    if i == self.robot_id:
                        continue
                    dx = self.x - self.poses[i].x
                    dy = self.y - self.poses[i].y
                    if dx*dx + dy*dy < _RB_R_SQ:
                        if not self.too_close:
                            print('Oh noes get away!')
                            self.current_waypoint -= 1
                            if self.current_waypoint >= 0:
                                self.send_goal_waypoint(self.current_waypoint)
                        self.too_close = 16
                    elif self.too_close:
                            self.too_close -= 1
                            if not self.too_close:
                                print('Resuming normal navigation.')
                                self.send_next_waypoint()

    def my_pose_callback(self, msg):
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
            return False
        return True

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
                print('Going RIGHT')
            elif tour[i + 1][0] < tour[i][0]:
                self.going = LEFT
                print('Going LEFT')
        self.send_goal((wp[0] + WALL_WIDTH_PIXELS)*self.scale_factor,
                self.top_wall_y - (wp[1]
                    + WALL_WIDTH_PIXELS)*self.scale_factor,
                self.going)

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
            os.path.realpath(input_dir), input_name, robot_count, '0']
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
    if len(argv) != 6:
        print(('Usage: {} <input dir> <input name> <total robot #>'
            ' <this robot #> <map file>').format(argv[0]))
        return 1
    # Parse arguments
    robot_id = int(argv[4])
    image_path = os.path.join(argv[1], argv[2])
    scale_factor = 0.1
    with open(argv[5]) as map_file:
        for line in map_file:
            line = line.strip()
            if line.startswith('resolution:'):
                scale_factor = float(line[11:].strip())
    top_wall_y = scale_factor*(Image.open(image_path).size[1]
            + WALL_WIDTH_PIXELS) + 0.5
    # Call afrl-oxdel and parse tour
    tour = generate_tours(*argv[1:4])[robot_id]
    if argv[2] == 'Compare_1' and robot_id == 0:
        tour.append([750, 136])
    print('{}: TOUR:\n  {}'.format(argv[0], tour))
    # Create ROS node
    ecr = EffCovRobot(robot_id, tour, top_wall_y, scale_factor)
    # Go! Main loop.
    ecr.spin()

if __name__ == '__main__':
    raise SystemExit(main())
