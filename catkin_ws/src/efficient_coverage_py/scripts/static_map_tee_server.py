#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

def tee_static_map(req):
    print('Call!')
    rospy.wait_for_service('/static_map')
    try:
        static_map = rospy.ServiceProxy('/static_map', GetMap)
        res = static_map(req)
        #print(res)
        return res
    except rospy.ServiceException as e:
        print('Call Failure: {}'.format(e))

def main():
    rospy.init_node('static_map_tee_server')
    s = rospy.Service('tee_static_map', GetMap, tee_static_map)
    print('Server Up')
    rospy.spin()

if __name__ == '__main__':
    main()
