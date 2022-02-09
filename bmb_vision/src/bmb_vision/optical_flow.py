import rospy
from bmb_msgs.msg import OpticalFlowReading
from sensor_msgs.msg import Image


class OpticalFlow:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rectified_image", Image, self.image_callback)
        self.optical_flow_pub = rospy.Publisher("/optical_flow_reading", OpticalFlowReading, queue_size=1)

    def image_callback(self, image):
        # TODO: compute optical_flow_reading based on reading
        optical_flow_reading = OpticalFlowReading()

        self.optical_flow_pub.publish(optical_flow_reading)

    def spin(self):
        rospy.spin()
