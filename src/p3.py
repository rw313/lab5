import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
	
    self.rate = 50
    self.r = rospy.Rate(self.rate)
    self.angular_speed = 1.0

    self.lower_yellow = numpy.array([20, 100, 100])
    self.upper_yellow = numpy.array([30, 255, 255])
    self.lower_red = numpy.array([0, 100, 100])
    self.upper_red = numpy.array([15, 255, 255])

    self.lower_general = numpy.array([10, 10, 10])
    self.upper_general = numpy.array([255, 255, 250])
  def get_mask_for_color(self, hsv, color=""):
    if color == 'yellow':
	lower = self.lower_yellow
	upper = self.upper_yellow
    elif color == 'red':
	lower = self.lower_red
	upper = self.upper_red
	mask = cv2.inRange(hsv, lower, upper)
	cv2.imshow("window", mask)
	h, w, d = hsv.shape
	search_top = 2*h/3
	search_bot = h
	mask[0:search_top, 0:w] = 0 #0 til top of the search and 0 til width = 0
	mask[search_bot:h, 0:w] = 0
	return mask
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

  def rotate(self, angle_in_degrees):
	goal_angle = angle_in_degrees * float(math.pi/180) # convert to radians
	angular_duration = goal_angle / self.angular_speed
	angular_duration = math.fabs(angular_duration)
	move_cmd = Twist()
	move_cmd.linear.x = 0.8
	if goal_angle < 0:
		move_cmd.angular.z = self.angular_speed * -1.0
	else:
		move_cmd.angular.z = self.angular_speed 
	ticks = int(angular_duration * self.rate)
	
	for t in range(ticks):
		self.cmd_vel_pub.publish(move_cmd)
		self.r.sleep()

	self.cmd_vel_pub.publish(move_cmd)

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    h, w, d = image.shape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = self.get_mask_for_color(hsv)
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])

      red_mask = self.get_mask_for_color(hsv, 'red')
      if red_mask[cy, cx] > 200:
	contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(image, contours, -1, (1,255,0), 3)
	M2 = cv2.moments(contours[0])
	redcx = int(M2["m10"] / M2["m00"])#also works with M
	redcy = int(M2["m01"] / M2["m00"])#also works with m
	
	left_max = 0
	right_max = 0
	for pt in contours[0]:
	    x = pt[0][0]
   	    y = pt[0][1]
	    dist = math.sqrt((x-redcx)*(x-redcx)+(y-redcy)*(y-redcy))
	    if dist > right_max and (x > w/2): 
		right_max = dist
		continue
	    if dist > left_max and (x < w/2):
		left_max = dist
	    #iterate through contour points which is a np array. 
	    #get the max euclidean distance on left and right side, if left then turn left
	#print("L, R: " + str(left_max) + ", " + str(right_max))
	#cv2.imshow("window", image)
	#cv2.waitKey(0)
	print("P to A:" )
	print(math.fabs(math.sqrt(cv2.contourArea(contours[0])) / cv2.arcLength(contours[0], True))) 
	if math.fabs(math.sqrt(cv2.contourArea(contours[0])) / cv2.arcLength(contours[0], True) - 0.066) < 0.05:
	   self.twist.linear.x = 1.3
	   self.twist.angular.z = 0.0
	   self.cmd_vel_pub.publish(self.twist)
	   self.r.sleep()
	   rospy.signal_shutdown("reached the end")
	   return  
	elif left_max > right_max: #left turn 
	    self.rotate(10)
    	elif right_max > left_max: # right turn aka blue
	    self.rotate(-10) 
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      # END CONTROL
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower', disable_signals=True)
follower = Follower()
rospy.spin()
# END ALL
