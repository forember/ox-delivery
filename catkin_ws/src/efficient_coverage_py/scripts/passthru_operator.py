#!/usr/bin/env python
'''Passes nav2d_operator/cmd messages through to geometry/Twist.

NOT ANYMORE. Follows the navigator's plan.'''

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav2d_operator.msg import cmd as OpCmd
from nav_msgs.msg import GridCells, Odometry
from tf import transformations


LINEAR_VEL_MPS = 2.0
ANGULAR_VEL_RADPS = math.pi / 2


def get_quaternion_yaw(ros_q):
    '''Retrieves the yaw Euler angle from a quaternion.'''
    numpy_q = (ros_q.x, ros_q.y, ros_q.z, ros_q.w)
    return transformations.euler_from_quaternion(numpy_q, 'sxyz')[2]


# As per PEP 485 -- A Function for testing approximate equality
def isclose(a, b, rel_tol=1e-9, abs_tol=0.0):
    return abs(a-b) <= max( rel_tol * max(abs(a), abs(b)), abs_tol )


def calc_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


class Vertex(object):
    def __init__(self, x, y, yaw_to_next=None, next_cell=None):
        self.x = x
        self.y = y
        self.on_cell = 1
        if yaw_to_next is not None:
            self.yaw_to_next = yaw_to_next
        if next_cell is not None:
            self.yaw_to_next = self.yaw_to(*next_cell)
        else:
            self.yaw_to_next = None
            self.on_cell = 0

    def yaw_to(self, x, y):
        return calc_angle(self.x, self.y, x, y)

    def __repr__(self):
        return 'Vertex({}, {}, {})'.format(self.x, self.y, self.yaw_to_next)


def convert_plan_to_path(plan):
    cell_w = plan.cell_width
    cell_h = plan.cell_height
    x_offset = plan.cell_width / 2
    y_offset = plan.cell_height / 2
    cell_radius_sq = cell_w*cell_w + cell_h*cell_h
    prev_x = None
    prev_y = None
    path = None
    for cell in plan.cells:
        try:
            x = cell.x + x_offset
            y = cell.y + y_offset
            if path is None:
                path = [Vertex(x, y)]
                continue
            curr_vertex = path[-1]
            yaw_to_next = curr_vertex.yaw_to_next
            if (yaw_to_next is not None and curr_vertex.on_cell >= 8
                    and not isclose(yaw_to_next, curr_vertex.yaw_to(x, y),
                                    abs_tol=math.pi/32)):
                path.append(Vertex(prev_x, prev_y, next_cell=(x, y)))
            elif curr_vertex.on_cell < 16:
                curr_vertex.yaw_to_next = curr_vertex.yaw_to(x, y)
                curr_vertex.on_cell += 1
        finally:
            prev_x = x
            prev_y = y
    if prev_y is not None:
        path.append(Vertex(x, y))
        if path[0].yaw_to_next is None:
            path[0].yaw_to_next = path[0].yaw_to(x, y)
    return path


def find_next_on_path(path, x, y):
    out_vert = None
    min_distance_sq = float('inf')
    for i, vert in enumerate(path[:-1]):
        dx = x - vert.x
        dy = y - vert.y
        distance_sq = dx*dx + dy*dy
        if (distance_sq < min_distance_sq and
            isclose(vert.yaw_to(x, y), vert.yaw_to_next, abs_tol=math.pi/2)):
            min_distance_sq = distance_sq
            out_vert = path[i + 1]
    if out_vert is None:
        out_vert = path[0]
    return out_vert


class PassThruOperator(object):
    def __init__(self):
        rospy.init_node('Operator')
        self.x = None
        self.y = None
        self.yaw = None
        self.goal_x = None
        self.goal_y = None
        self.path = None
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('base_pose_ground_truth', Odometry,
                                         self.pose_callback)
        #self.plan_sub = rospy.Subscriber('Navigator/plan', GridCells,
        #                                 self.plan_callback)
        self.op_cmd_sub = rospy.Subscriber('cmd', OpCmd, self.op_cmd_callback)
        #self.goal_sub = rospy.Subscriber('goal', PoseStamped,
        #                                 self.goal_callback)

    def move(self, linear, angular):
        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self.cmd_vel_pub.publish(out)

    def _old_pose_callback(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = get_quaternion_yaw(pose.orientation)
        if self.path is not None:
            self.next_on_path = find_next_on_path(self.path, self.x, self.y)
            if self.next_on_path is None:
                self.move(0, 0)
                return
            angle = calc_angle(self.x, self.y,
                               self.next_on_path.x, self.next_on_path.y)
            delta_angle = angle - self.yaw
            if abs(delta_angle) <= math.pi / 8:
                dx = self.x - self.next_on_path.x
                dy = self.y - self.next_on_path.y
                distance_sq = dx*dx + dy*dy
                self.move(min(LINEAR_VEL_MPS, max(distance_sq, 0.5)), delta_angle)
            else:
                self.move(0, math.copysign(min(ANGULAR_VEL_RADPS,
                                               max(abs(delta_angle),
                                                   math.pi/64)), delta_angle))

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = get_quaternion_yaw(pose.orientation)

    def plan_callback(self, msg):
        self.path = convert_plan_to_path(msg)
        print 'New path: {}'.format(self.path)

    def op_cmd_callback(self, msg):
        if abs(msg.Turn) > math.pi / 8:
            self.move(0, -math.copysign(ANGULAR_VEL_RADPS, msg.Turn))
        else:
            near_up = abs(self.yaw - math.pi/2)
            near_down = abs(self.yaw + math.pi/2)
            if min(near_up, near_down) < math.pi / 8:
                self.move(LINEAR_VEL_MPS*msg.Velocity,
                          -math.copysign(min(near_up, near_down), msg.Turn))
            else:
                self.move(LINEAR_VEL_MPS*msg.Velocity, -ANGULAR_VEL_RADPS*msg.Turn)

    def goal_callback(self, msg):
        position = msg.pose.position
        self.goal_x = position.x
        self.goal_y = position.y

    def spin(self):
        rospy.spin()

def main():
    pto = PassThruOperator()
    pto.spin()

if __name__ == '__main__':
    import sys
    sys.stdout = sys.stderr
    main()
