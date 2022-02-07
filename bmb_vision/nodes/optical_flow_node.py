#!/usr/bin/env python3

import rospy
from bmb_vision.optical_flow import OpticalFlow


def main():
    rospy.init_node("optical_flow_node")
    optical_flow = OpticalFlow()
    optical_flow.spin()


if __name__ == '__main__':
    main()
