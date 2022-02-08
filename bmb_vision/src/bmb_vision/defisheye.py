import rospy
from sensor_msgs.msg import Image


class Defisheye:
    def __init__(self):
        self.raw_image_sub = rospy.Subscriber("/raw_image", Image, self.raw_image_callback)
        self.rectified_image_pub = rospy.Publisher("/rectified_image", Image, queue_size=1)

    def raw_image_callback(self, raw_image):
        # TODO: compute defisheyed image
        rectified_image = raw_image

        self.rectified_image_pub.publish(rectified_image)

    def spin(self):
        rospy.spin()
