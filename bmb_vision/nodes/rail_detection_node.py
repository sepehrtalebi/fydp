#!/usr/bin/env python3

import rospy
from bmb_vision.rail_detection_helper import RailDetectionHelper


def main():
    rospy.init_node("/rail_detection_node")
    rail_detection_helper = RailDetectionHelper()
    rail_detection_helper.spin()


if __name__ == '__main__':
    main()
