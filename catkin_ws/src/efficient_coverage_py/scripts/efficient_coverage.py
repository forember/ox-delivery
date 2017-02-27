#!/usr/bin/env python
'''Simulation controller for oxdel.'''

import os
import math
import collections
from PIL import Image
from scipy import ndimage
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from nav2d_navigator.msg import MoveToPosition2DActionResult
from tf import transformations

WALL_WIDTH_PIXELS = 10
WAYPOINT_RADIUS = 1.25
ROBOT_BERTH_RADIUS = 3

_WP_R_SQ = WAYPOINT_RADIUS*WAYPOINT_RADIUS
_RB_R_SQ = ROBOT_BERTH_RADIUS*ROBOT_BERTH_RADIUS

LEFT = math.pi
RIGHT = 0.0

DO_DETAIL_TOUR = False

def get_quaternion_yaw(ros_q):
    '''Retrieves the yaw Euler angle from a quaternion.'''
    numpy_q = (ros_q.x, ros_q.y, ros_q.z, ros_q.w)
    return transformations.euler_from_quaternion(numpy_q, 'sxyz')[2]

def get_yaw_quaternion(yaw):
    '''Creates a quaternion from Euler angles (0, 0, yaw).'''
    numpy_q = transformations.quaternion_from_euler(0, 0, yaw)
    ros_q = Quaternion()
    ros_q.x = numpy_q[0]
    ros_q.y = numpy_q[1]
    ros_q.z = numpy_q[2]
    ros_q.w = numpy_q[3]
    return ros_q


PoseTuple = collections.namedtuple('PoseTuple', ['x', 'y', 'yaw'])


class PoseCallback(object):
    '''Wrapper for pose callback function, storing source robot id.'''

    def __init__(self, i, _pose_callback):
        '''Create wrapper with source robot id and callback function.'''
        self.i = i
        self._pose_callback = _pose_callback

    def pose_callback(self, msg):
        '''Call the callback function.'''
        self._pose_callback(self.i, msg)


class EffCovRobot(object):
    '''WARNING: Only create one instance per process!'''

    def __init__(self, robot_id, tour, top_wall_y, scale_factor, image_path):
        self.robot_id = robot_id
        self.tour = tour
        self.top_wall_y = top_wall_y
        self.scale_factor = scale_factor
        self.image = ndimage.imread(image_path, mode='L')
        self.gauss_image = ndimage.gaussian_filter(self.image, 64, truncate=4)
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
        self.op_coupled = not DO_DETAIL_TOUR
        self.stay_counter = 0
        # Publish to command the stage robot
        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # Initialize node
        rospy.init_node('ECRobot_{}'.format(robot_id))
        # Subscribe to the operator output
        self.op_cmd_vel_sub = rospy.Subscriber('op_cmd_vel', Twist,
                                               self.op_cmd_vel_callback)
        # Subscribe to movement results
        self.result_sub = rospy.Subscriber('MoveTo/result',
                                           MoveToPosition2DActionResult,
                                           self.result_callback)
        # Subscribe to pose of stage robot
        self.pose_counter = 0
        self.pose_subs = []
        for i in range(4):
            print '/robot_{}/base_pose_ground_truth'.format(i)
            pc_obj = PoseCallback(i, self.pose_callback)
            self.pose_subs.append(rospy.Subscriber(
                '/robot_{}/base_pose_ground_truth'.format(i),
                Odometry, pc_obj.pose_callback, queue_size=1))
        print 'My ID is {}.'.format(self.robot_id)

    def op_cmd_vel_callback(self, msg):
        if self.op_coupled:
            self.cmd_vel_pub.publish(msg)

    def result_callback(self, msg):
        print 'Got Result!'
        #self.send_next_waypoint()

    def pose_callback(self, i, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = get_quaternion_yaw(msg.pose.pose.orientation)
        self.poses[i] = PoseTuple(x, y, yaw)
        if i == self.robot_id and self.my_pose_callback(msg):
            for i in range(4):
                if i == self.robot_id:
                    continue
                dx = self.x - self.poses[i].x
                dy = self.y - self.poses[i].y
                if dx*dx + dy*dy < _RB_R_SQ:
                    if not self.too_close:
                        print 'Oh noes get away!'
                        self.current_waypoint -= 1
                        if self.current_waypoint >= 0:
                            self.send_goal_waypoint(self.current_waypoint)
                    self.too_close = 16
                elif self.too_close:
                    self.too_close -= 1
                    if not self.too_close:
                        print 'Resuming normal navigation.'
                        self.send_next_waypoint()

    def my_pose_callback(self, msg):
        # Collect pose from message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = get_quaternion_yaw(msg.pose.pose.orientation)
        if self.op_coupled and x == self.x and y == self.y and yaw == self.yaw:
            self.stay_counter += 1
        else:
            self.stay_counter = 0
        if self.stay_counter >= 16:
            print "I can't stay here no more!"
            self.send_next_waypoint()
        self.x = x
        self.y = y
        self.yaw = yaw
        if self.done or self.stay_counter >= 16:
            self.stay_counter = 1
            return False
        self.pose_counter = (self.pose_counter + 1) % 128
        if self.pose_counter == 0:
            print 'x: {}, y: {}, yaw: {}'.format(self.x, self.y, self.yaw)
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        if dx*dx + dy*dy < _WP_R_SQ:
            print 'Good enough for government work!'
            self.send_next_waypoint()
            en_route = False
        else:
            en_route = True
        if not self.op_coupled:
            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            dyaw = math.atan2(dy, dx) - self.yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            if math.fabs(dyaw) > math.pi / 16:
                self.send_cmd_vel(0, 0, math.copysign(1, dyaw))
            else:
                dm = math.sqrt(dx*dx + dy*dy)
                self.send_cmd_vel(dx/dm, dy/dm, 0) # TODO super wrong
        return en_route

    def send_cmd_vel(self, vx, vy, vyaw):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = vyaw
        self.cmd_vel_pub.publish(msg)

    def is_goal_line_clear(self):
        irx = self.x/self.scale_factor - WALL_WIDTH_PIXELS
        iry = (self.top_wall_y - self.y)/self.scale_factor - WALL_WIDTH_PIXELS
        igx = self.goal_x/self.scale_factor - WALL_WIDTH_PIXELS
        igy = ((self.top_wall_y - self.goal_y)/self.scale_factor
               - WALL_WIDTH_PIXELS)

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
        print 'Sending Goal: ({}, {}); {}'.format(gx, gy, gyaw)
        self.goal_pub.publish(m)

    def send_goal_waypoint(self, i):
        tour = self.tour
        wp = tour[i]
        if i + 1 < len(tour):
            if tour[i + 1][0] > tour[i][0]:
                self.going = RIGHT
                print 'Going RIGHT'
            elif tour[i + 1][0] < tour[i][0]:
                self.going = LEFT
                print 'Going LEFT'
        gx = (wp[0] + WALL_WIDTH_PIXELS)*self.scale_factor
        gy = self.top_wall_y - (wp[1] + WALL_WIDTH_PIXELS)*self.scale_factor
        if DO_DETAIL_TOUR and i != 0:
            self.send_goal(gx, gy, math.atan2(-(wp[1] - tour[i - 1][1]),
                                              wp[0] - tour[i - 1][0]))
        else:
            self.send_goal(gx, gy, self.going)

    def send_next_waypoint(self):
        self.current_waypoint += 1
        if self.current_waypoint < len(self.tour):
            self.send_goal_waypoint(self.current_waypoint)
        else:
            self.done = True
            print 'Tour Completed!'

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

def expand_image(input_dir, input_name, output_dir):
    import shutil
    import cv2
    import numpy as np
    from PIL import Image
    root, ext = os.path.splitext(input_name)
    shutil.copy2(os.path.join(input_dir, input_name),
                 os.path.join(output_dir, input_name))
    shutil.copy2(os.path.join(input_dir, root + '.wf'),
                 os.path.join(output_dir, root + '.wf'))
    output_path1 = os.path.join(output_dir, '.' + root + '.map' + ext)
    output_path2 = os.path.join(output_dir, root + '.map' + ext)
    img = cv2.imread(os.path.join(input_dir, root + '.map' + ext), 0)
    img = cv2.copyMakeBorder(img, 1, 1, 1, 1,  cv2.BORDER_CONSTANT, value=0)
    kernel = np.ones((9,9),np.uint8)
    expansion = cv2.erode(img, kernel, iterations=2)
    expansion = expansion[1:-1, 1:-1]
    cv2.imwrite(output_path1, expansion)
    (Image.open(output_path1)
          .convert('1', dither=Image.NONE)
          .convert('RGB')
          .save(output_path2))

def generate_tours(input_dir, input_name, robot_count=1):
    import tempfile
    import subprocess
    import shutil
    # Create a temporary directory for afrl-oxdel to use
    tmpdir = tempfile.mkdtemp()
    tours = None
    try:
        expand_image(input_dir, input_name, tmpdir)
        # Call afrl-oxdel in the temp directory
        #cmd_spec = ['/usr/bin/env', 'afrl-oxdel',
        #            os.path.realpath(input_dir), input_name, robot_count, '0']
        cmd_spec = ['/usr/bin/env', 'afrl-oxdel',
                    tmpdir, input_name, robot_count, '0']
        print 'Calling {}\n  in {}:'.format(cmd_spec, tmpdir)
        subprocess.call(cmd_spec, cwd=tmpdir)
        # Parse the tour lines file
        tour_file_path = os.path.join(tmpdir, 'tourLines.txt')
        with open(tour_file_path) as tour_file:
            tours = parse_tours(tour_file)
    finally:
        # Always remove the temporary directory
        shutil.rmtree(tmpdir)
    return tours

def detail_tour(tour, image_path, granularity=32):
    new_tour = []
    im = Image.open(image_path).convert('1', dither=Image.NONE)
    for i in xrange(1, len(tour)):
        start = tour[i - 1]
        end = tour[i]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.sqrt(dx*dx + dy*dy)
        if distance == 0:
            continue
        ux = dx / distance
        uy = dy / distance
        mag_list = range(0, int(distance), granularity)
        if mag_list[-1] != int(distance):
            mag_list.append(int(distance))
        for mag in mag_list:
            x = start[0] + mag*ux
            y = start[1] + mag*uy
            (nx, ny) = (int(x), int(y))
            ''' # This part does terrible inflation. Made OpenCV do inflation.
            #print start, end, mag, distance, (nx, ny), (ux, uy)
            theta_ctr = 0
            pixel = im.getpixel((nx, ny))
            while pixel == 0:
                theta = (theta_ctr%16) * math.pi/8
                r = 1 + theta_ctr//16
                nx = int(x + r*math.cos(theta))
                ny = int(y + r*math.sin(theta))
                try:
                    pixel = im.getpixel((nx, ny))
                except IndexError:
                    pixel = 0
                theta_ctr += 1
            if theta_ctr == 0:
                while pixel != 0 and theta_ctr < 256:
                    theta = (theta_ctr%16) * math.pi/8
                    r = 1 + theta_ctr//16
                    nx = int(x + r*math.cos(theta))
                    ny = int(y + r*math.sin(theta))
                    try:
                        pixel = im.getpixel((nx, ny))
                    except IndexError:
                        pixel = 0
                    theta_ctr += 1
                r = -r
            if theta_ctr < 256:
                r += 4
            nx = int(x + r*math.cos(theta))
            ny = int(y + r*math.sin(theta))
            if nx < 4:
                nx = 4
            if nx >= im.size[0] - 4:
                nx = im.size[0] - 4
            if ny < 4:
                ny = 4
            if ny >= im.size[1] - 4:
                ny = im.size[1] - 4
            '''
            new_tour.append([nx, ny])
    return new_tour

def main():
    # Get arguments without ROS's extra args
    argv = rospy.myargv()
    if len(argv) != 6 and len(argv) != 7:
        print ('Usage: {} <input dir> <input name> <total robot #>'
               ' <this robot #> <map file> [tour lines file]').format(argv[0])
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
    if len(argv) == 6:
        tour = generate_tours(*argv[1:4])[robot_id]
    else:
        with open(argv[6]) as tour_file:
            tour = parse_tours(tour_file)[robot_id]
    if DO_DETAIL_TOUR:
        tour = detail_tour(tour, image_path)
    for i, waypoint in enumerate(tour):
        offset = 1 / scale_factor
        tour[i] = (waypoint[0] + offset, waypoint[1] - offset)
    print '{}: TOUR:\n  {}'.format(argv[0], tour)
    # Create ROS node
    ecr = EffCovRobot(robot_id, tour, top_wall_y, scale_factor, image_path)
    # Go! Main loop.
    ecr.spin()

if __name__ == '__main__':
    raise SystemExit(main())
