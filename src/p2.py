import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
    self.lower_yellow = numpy.array([20, 100, 100])
    self.upper_yellow = numpy.array([30, 255, 255])
    self.lower_green = numpy.array([45, 100, 100])
    self.upper_green = numpy.array([75, 255, 255])
    self.lower_general = numpy.array([10, 10, 10])
    self.upper_general = numpy.array([255, 255, 250])
  def get_mask_for_color(self, hsv, color=""):
    if color == 'green':
	lower = self.lower_green
	upper = self.upper_green
    elif color == 'yellow':
	lower = self.lower_yellow
	upper = self.upper_yellow
    else:
	lower = self.lower_general
	upper = self.upper_general
    mask = cv2.inRange(hsv, lower, upper)
    cv2.imshow("window", mask)
    h, w, d = hsv.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    h, w, d = image.shape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = self.get_mask_for_color(hsv)
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])

      green_mask = self.get_mask_for_color(hsv, 'green')
      print(green_mask[cy, cx])
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      # END CONTROL
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
