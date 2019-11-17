import rospy
from sensor_msgs.msg import Image

sub_image = rospy.Subscriber("/mynteye/left/image_color", Image, image_callback)
print(sub_image)