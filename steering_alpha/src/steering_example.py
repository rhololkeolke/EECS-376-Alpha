#!/usr/bin/env python
# encoding: utf-8

import roslib
roslib.load_manifest('alpha_steering')

import rospy

if __name__ == '__main__':
    main()

def main():
    """
    Main NL steering method

    """
    rospy.spin()
