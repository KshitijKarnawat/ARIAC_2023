#!/usr/bin/env python3

# # Credits:
# - Addison Sears-Collins
# - https://automaticaddison.com
   
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import time
from group3.msg import Part as Partmsg
from group3.msg import Parts as Partsmsg

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('part_detector')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription1 = self.create_subscription(
      Image, 
      # '/ariac/sensors/kts1_rgb_camera/rgb_image',
      '/ariac/sensors/right_bins_rgb_camera/rgb_image',
      self.listener_callback1, 
      10)
    self.subscription1 # prevent unused variable warning

    self.subscription2 = self.create_subscription(
      Image, 
      # '/ariac/sensors/kts1_rgb_camera/rgb_image',
      '/ariac/sensors/left_bins_rgb_camera/rgb_image',
      self.listener_callback2, 
      10)
    self.subscription2 # prevent unused variable warning

    self.subscription3 = self.create_subscription(
      Image, 
      # '/ariac/sensors/kts1_rgb_camera/rgb_image',
      '/ariac/sensors/conv_rgb_camera/rgb_image',
      self.listener_callback3, 
      10)
    self.subscription3 # prevent unused variable warning

    self.publisher1_ = self.create_publisher(Partsmsg, 'right_bin_part_detector', 10)
    self.publisher2_ = self.create_publisher(Partsmsg, 'left_bin_part_detector', 10)
    self.publisher3_ = self.create_publisher(Partmsg, 'conveyor_part_detector', 10)
       
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
  def listener_callback1(self, data):
    msg2 = Partsmsg()
    parts_ = []
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
     
    # Display image
    img = current_frame

    img_bin_tr = img[28:223, 333:530]
    img_bin_tl = img[28:223, 92:285]
    img_bin_br = img[267:458, 333:530]
    img_bin_bl = img[267:458, 92:285]

    image = [img_bin_br, img_bin_bl, img_bin_tl, img_bin_tr]
    i = 0

    for img in image:
      img_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
      lower_red = np.array([0, 25, 25])
      upper_red = np.array([10, 255, 255])
      lower_red1 = np.array([170, 25, 25])
      upper_red1 = np.array([180, 255, 255])


      lower_green = np.array([36, 25, 25])
      upper_green = np.array([70, 255, 255])

      lower_blue = np.array([110, 25, 25])
      upper_blue = np.array([130, 255, 255])


      lower_orange = np.array([6, 25, 25])
      upper_orange = np.array([26, 255, 255])


      lower_purple = np.array([128, 25, 25])
      upper_purple = np.array([148, 255, 255])

      lower_gray = np.array([0, 0, 32])
      upper_gray = np.array([0, 255, 117])
      
      mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
      mask11 = cv2.inRange(img_hsv, lower_red1, upper_red1)
      mask2 = cv2.inRange(img_hsv, lower_green, upper_green)
      mask3 = cv2.inRange(img_hsv, lower_blue, upper_blue)
      mask4 = cv2.inRange(img_hsv, lower_orange, upper_orange)
      mask5 = cv2.inRange(img_hsv, lower_purple, upper_purple)
      mask6 = cv2.inRange(img_hsv, lower_gray, upper_gray)

      mask = mask1 + mask11 + mask2 + mask3 + mask4 + mask5 + mask6
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((3, 3), np.uint8), iterations=2)
      mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3), np.uint8), iterations=1)

      new_image = cv2.bitwise_and(img, img, mask = mask)

      blur = cv2.GaussianBlur(mask,(5,5), 0)
      contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

      for c in contours:
        msg = Partmsg()
        area_cnt = cv2.contourArea(c)
        M = cv2.moments(c)
        if M['m00'] != 0.0:
          x_m = int(M['m10']/M['m00'])
          y_m = int(M['m01']/M['m00'])
        if area_cnt > 500 and area_cnt < 3000:
          quad_ = self.rightbin_detect_quadrant(x_m, y_m, i)
          if quad_ != 100:
            msg.quad = quad_
          cv2.drawContours(img, c, -1, (0,255,0), 2)
          img_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
          hsv = img_hsv[int(y_m),int(x_m)]

          if (hsv[0]>10 and hsv[0]<25):
            msg.color = 3
            # self.get_logger().info("Orange")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>=130 and hsv[0]<170):
            msg.color = 4
            # self.get_logger().info("Purple")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>=90 and hsv[0]<130):
            msg.color = 2
            # self.get_logger().info("Blue")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>=0 and hsv[0]<=10):
            msg.color = 0
            # self.get_logger().info("Red")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>36 and hsv[0]<89):
            msg.color = 1
            # self.get_logger().info("Green")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          parts_.append(msg)
      i +=1
    # self.get_logger().info("\n")
    parts_ = sorted(parts_, key=lambda x: x.quad)
    msg2.parts = parts_
    self.publisher1_.publish(msg2)
    # self.destroy_node()

  def listener_callback2(self, data):
    msg2 = Partsmsg()
    parts_ = []
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
     
    # Display image
    img = current_frame

    img_bin_tr = img[28:223, 333:530]
    img_bin_tl = img[28:223, 92:285]
    img_bin_br = img[267:458, 333:530]
    img_bin_bl = img[267:458, 92:285]

    image = [img_bin_bl, img_bin_br, img_bin_tr, img_bin_tl]
    i = 0

    for img in image:
      img_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
      lower_red = np.array([0, 25, 25])
      upper_red = np.array([10, 255, 255])
      lower_red1 = np.array([170, 25, 25])
      upper_red1 = np.array([180, 255, 255])


      lower_green = np.array([36, 25, 25])
      upper_green = np.array([70, 255, 255])

      lower_blue = np.array([110, 25, 25])
      upper_blue = np.array([130, 255, 255])


      lower_orange = np.array([6, 25, 25])
      upper_orange = np.array([26, 255, 255])


      lower_purple = np.array([128, 25, 25])
      upper_purple = np.array([148, 255, 255])

      lower_gray = np.array([0, 0, 32])
      upper_gray = np.array([0, 255, 117])
      
      mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
      mask11 = cv2.inRange(img_hsv, lower_red1, upper_red1)
      mask2 = cv2.inRange(img_hsv, lower_green, upper_green)
      mask3 = cv2.inRange(img_hsv, lower_blue, upper_blue)
      mask4 = cv2.inRange(img_hsv, lower_orange, upper_orange)
      mask5 = cv2.inRange(img_hsv, lower_purple, upper_purple)
      mask6 = cv2.inRange(img_hsv, lower_gray, upper_gray)

      mask = mask1 + mask11 + mask2 + mask3 + mask4 + mask5 + mask6
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((3, 3), np.uint8), iterations=2)
      mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3), np.uint8), iterations=1)

      new_image = cv2.bitwise_and(img, img, mask = mask)

      blur = cv2.GaussianBlur(mask,(5,5), 0)
      contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

      for c in contours:
        msg = Partmsg()
        area_cnt = cv2.contourArea(c)
        M = cv2.moments(c)
        if M['m00'] != 0.0:
          x_m = int(M['m10']/M['m00'])
          y_m = int(M['m01']/M['m00'])
        if area_cnt > 500 and area_cnt < 3000:
          quad_  = self.leftbin_detect_quadrant(x_m, y_m, i)
          if quad_ != 100:
            msg.quad = quad_
          cv2.drawContours(img, c, -1, (0,255,0), 2)
          img_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
          hsv = img_hsv[int(y_m),int(x_m)]

          if (hsv[0]>10 and hsv[0]<25):
            msg.color = 3
            # self.get_logger().info("Orange")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>=130 and hsv[0]<170):
            msg.color = 4
            # self.get_logger().info("Purple")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>=90 and hsv[0]<130):
            msg.color = 2
            # self.get_logger().info("Blue")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>=0 and hsv[0]<=10):
            msg.color = 0
            # self.get_logger().info("Red")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          elif(hsv[0]>36 and hsv[0]<89):
            msg.color = 1
            # self.get_logger().info("Green")
            type_ = int(self.detect_type(new_image.copy(), c))
            if type != 0:
              msg.type = type_
          parts_.append(msg)
      i +=1
    # self.get_logger().info("\n")
    parts_ = sorted(parts_, key=lambda x: x.quad)
    msg2.parts = parts_
    self.publisher2_.publish(msg2)
    # self.destroy_node()

  def listener_callback3(self, data):
    # msg3 = Partmsg()
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame conv')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    # Display image
    img = current_frame
    img = img[180:479, 223:443]
    img_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 55, 25])
    upper_red = np.array([10, 255, 255])
    lower_red1 = np.array([170, 55, 25])
    upper_red1 = np.array([180, 255, 255])

    lower_green = np.array([36, 55, 25])
    upper_green = np.array([70, 255, 255])

    lower_blue = np.array([110, 55, 25])
    upper_blue = np.array([130, 255, 255])

    lower_orange = np.array([6, 55, 25])
    upper_orange = np.array([26, 255, 255])

    lower_purple = np.array([128, 55, 25])
    upper_purple = np.array([148, 255, 255])

    lower_gray = np.array([0, 0, 32])
    upper_gray = np.array([0, 255, 117])
    
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask11 = cv2.inRange(img_hsv, lower_red1, upper_red1)

    mask2 = cv2.inRange(img_hsv, lower_green, upper_green)

    mask3 = cv2.inRange(img_hsv, lower_blue, upper_blue)

    mask4 = cv2.inRange(img_hsv, lower_orange, upper_orange)

    mask5 = cv2.inRange(img_hsv, lower_purple, upper_purple)

    mask6 = cv2.inRange(img_hsv, lower_gray, upper_gray)

    mask = mask1 + mask2 + mask3 + mask4 + mask5 + mask6 + mask11
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((3, 3), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3), np.uint8), iterations=1)

    new_image = cv2.bitwise_and(img, img, mask = mask)
    blur = cv2.GaussianBlur(mask,(5,5), 0)
    contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contours:
      msg = Partmsg()
      M = cv2.moments(c)
      if M['m00'] != 0.0:
        x_m = int(M['m10']/M['m00'])
        y_m = int(M['m01']/M['m00'])

      msg.quad = 0
      img_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
      hsv = img_hsv[int(y_m),int(x_m)]
      # self.get_logger().info("Im Here3")
      # cv2.imshow("Detetct_type_conv", img)
      # cv2.waitKey(0)

      if (hsv[0]>10 and hsv[0]<25):
        msg.color = 3
        # self.get_logger().info("Orange")
        msg.type = int(self.conveyor_detect_type(new_image.copy(), c))
      elif(hsv[0]>=130 and hsv[0]<170):
        msg.color = 4
        # self.get_logger().info("Purple")
        msg.type = int(self.conveyor_detect_type(new_image.copy(), c))
      elif(hsv[0]>=90 and hsv[0]<130):
        msg.color = 2
        # self.get_logger().info("Blue")
        msg.type = int(self.conveyor_detect_type(new_image.copy(), c))
      elif(hsv[0]>=0 and hsv[0]<=10):
        msg.color = 0
        # self.get_logger().info("Red")
        msg.type = int(self.conveyor_detect_type(new_image.copy(), c))
      elif(hsv[0]>36 and hsv[0]<89):
        msg.color = 1
        # self.get_logger().info("Green")
        msg.type = int(self.conveyor_detect_type(new_image.copy(), c))
      # parts_.append(msg)
      # self.get_logger().info("\n")
      self.publisher3_.publish(msg)
      # self.destroy_node()

  def detect_type(self,img, cnt):
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    im = img.copy()
    im = im[y+1:y+h+1, x+1:x+w+1] # changed from 2
    im_hsv = cv2.cvtColor(im.copy(), cv2.COLOR_BGR2HSV)

    lower_gray = np.array([0, 0, 32])
    upper_gray = np.array([0, 0, 117])
    mask = cv2.inRange(im_hsv, lower_gray, upper_gray)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3), np.uint8), iterations=2)
    
    contours, __ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    count = 0

    for c in contours:
      area_gray = cv2.contourArea(c)
      perimeter = cv2.arcLength(cnt,True)
      if area_gray > 10:
          count += 1 
    if (count == 2):
      # self.get_logger().info("Regulator")
      return 13
    elif (count == 1):
      if perimeter < 151 and 57<area_gray<214:
        # self.get_logger().info("Battery")
        return 10
      elif area_gray < 20:
        # self.get_logger().info("Pump")
        return 11
      elif perimeter > 151 and 23<area_gray<203:
        # self.get_logger().info("Sensor")
        return 12
    else:
      # self.get_logger().info("Pump")
      return 11
    return 0

  def rightbin_detect_quadrant(self,x_m, y_m, i):
    if 0< x_m <70 and 0 < y_m < 65:
      q = 9*i + 1
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 71< x_m <135 and 0 < y_m < 65:
      q = 9*i + 2
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 137< x_m <195 and 0 < y_m < 65:
      q = 9*i + 3
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 0< x_m <70 and 69 < y_m < 126:
      q = 9*i + 4
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 71< x_m <135 and 69 < y_m < 126:
      q = 9*i + 5
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 137< x_m <195 and 69 < y_m < 126:
      q = 9*i + 6
      # self.get_logger().info("Quadrant is ""%i" %q)
    
    elif 0< x_m <70 and 130 < y_m < 192:
      q = 9*i + 7
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 71< x_m <135 and 130 < y_m < 192:
      q = 9*i + 8
      # self.get_logger().info("Quadrant is ""%i" %q)
    
    elif 137< x_m <195 and 130 < y_m < 192:
      q = 9*i + 9
      # self.get_logger().info("Quadrant is ""%i" %q)
    
    else :
      q = 100

    return q

  def leftbin_detect_quadrant(self,x_m, y_m, i):
    if 0< x_m <70 and 0 < y_m < 65:
      q = 9*i + 37
      # self.get_logger().info("Quadrant is ""%i" %q)
    
    elif 71< x_m <135 and 0 < y_m < 65:
      q = 9*i + 38
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 137< x_m <195 and 0 < y_m < 65:
      q = 9*i + 39
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 0< x_m <70 and 69 < y_m < 126:
      q = 9*i + 40
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 71< x_m <135 and 69 < y_m < 126:
      q = 9*i + 41
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 137< x_m <195 and 69 < y_m < 126:
      q = 9*i + 42
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 0< x_m <70 and 130 < y_m < 192:
      q = 9*i + 43
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 71< x_m <135 and 130 < y_m < 192:
      q = 9*i + 44
      # self.get_logger().info("Quadrant is ""%i" %q)

    elif 137< x_m <195 and 130 < y_m < 192:
      q = 9*i + 45
      # self.get_logger().info("Quadrant is ""%i" %q)

    else :
      q = 100

    return q
 
  def conveyor_detect_type(self,img, cnt):
    # self.get_logger().info("Im Here2")
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # cv2.imshow("Detetct_type_conv", img)
    # cv2.waitKey(0)
    im_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)

    lower_gray = np.array([0, 0, 32])
    upper_gray = np.array([0, 0, 117])
    mask = cv2.inRange(im_hsv, lower_gray, upper_gray)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3), np.uint8), iterations=2)
    
    contours, __ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    count = 0

    for c in contours:
      area_gray = cv2.contourArea(c)
      perimeter = cv2.arcLength(cnt,True)
      if area_gray > 10:
        count += 1 
    if (count == 2):
      # self.get_logger().info("Regulator")
      return 13
    elif (count == 1):
      if perimeter < 250:
        # self.get_logger().info("Battery")
        return 10
      elif area_gray < 20:
        # self.get_logger().info("Pump")
        return 11
      elif perimeter > 250:
        # self.get_logger().info("Sensor")
        return 12
    else:
      # self.get_logger().info("Pump")
      return 11

def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  image_subscriber_publisher = ImageSubscriber()
   
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber_publisher)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber_publisher.destroy_node()
  cv2.destroyAllWindows()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
