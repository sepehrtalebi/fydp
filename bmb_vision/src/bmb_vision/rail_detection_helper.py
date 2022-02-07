import rospy
from bmb_msgs.msg import RailDetection
from sensor_msgs.msg import Image


class RailDetectionHelper:
    def __init__(self):
        self.raw_image_sub = rospy.Subscriber("/rectified_image", Image, self.image_callback)
        self.rail_detection_pub = rospy.Publisher("/rail_detection", RailDetection, queue_size=1)

    def image_callback(self, image):
        # TODO: perform rail detection
        rail_detection = RailDetection()

        self.rail_detection_pub.publish(rail_detection)

    def spin(self):
        rospy.spin()
