#!/usr/bin/env python3

import rospy
from bmb_vision.defisheye import Defisheye


def main():
    rospy.init_node("defisheye_node")
    defisheye = Defisheye()
    defisheye.spin()


if __name__ == '__main__':
    main()
