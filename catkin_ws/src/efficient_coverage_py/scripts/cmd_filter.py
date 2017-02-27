#!/usr/bin/env python

import math
import rospy
from nav2d_operator.msg import cmd as OpCmd

LINEAR_VEL_MPS = 2.0
ANGULAR_VEL_RADPS = math.pi / 2

class CmdFilter(object):
    def __init__(self):
        rospy.init_node('CmdFilter')
        self.cmd_sub = rospy.Subscriber('cmd', OpCmd, self.cmd_callback)
        self.op_cmd_pub = rospy.Publisher('op_cmd', OpCmd, queue_size=1)

    def move(self, linear, angular):
        out = OpCmd()
        out.Velocity = linear
        out.Turn = angular
        self.op_cmd_pub.publish(out)

    def cmd_callback(self, msg):
        if abs(msg.Turn) > math.pi / 8:
            self.move(0, math.copysign(ANGULAR_VEL_RADPS, msg.Turn))
        else:
            self.move(LINEAR_VEL_MPS*msg.Velocity, 0)

    def spin(self):
        rospy.spin()

def main():
    cf = CmdFilter()
    cf.spin()

if __name__ == '__main__':
    import sys
    sys.stdout = sys.stderr
    main()
