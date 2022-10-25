# Information: https://clover.coex.tech/programming

from pyclbr import Function
import cv2
import math
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from clover import srv

from sensor_msgs.msg import Image

from std_srvs.srv import Trigger

THRESHOLD = 10000
THRESHOLD_IMAGE = 20
ADDITION = 1
REDUCTION = 0.75

FOCUS_X = -670
FOCUS_Y = -670

WIDTH = 640
HEIGHT = 360

STARTING = 0
SEARCHING = 1
CENTERING = 2
CENTERED = 3
WAITING = 4
LANDING = 5


def is_close_enough_2d(x_1: float, y_1: float, x_2: float, y_2: float, dist: float) -> bool:
    real_dist = np.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)
    return real_dist <= dist

def distance_2d(x_1: float, y_1: float, x_2: float, y_2: float) -> float:
    return np.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)

def bounding_box_vertices(img_mask: np.ndarray) -> tuple:
    '''
        Bounding box of detected region (ex: red area)
    '''
    xys_pts = np.nonzero(img_mask)
    x_of_interest = xys_pts[1]
    y_of_interest = xys_pts[0]
    
    xmin, xmax = min(x_of_interest), max(x_of_interest)
    ymin, ymax = min(y_of_interest), max(y_of_interest)
    
    return xmin, ymin, xmax, ymax

def estimate_3d_coordinates(x_pixel: int, y_pixel: int, z_gps: float) -> np.ndarray:

    cx = WIDTH//2
    cy = HEIGHT//2

    Z =  z_gps #altura em metros
    X = Z * (x_pixel - cx)/FOCUS_X
    Y = Z * (y_pixel - cy)/FOCUS_Y
    return X, Y

def color_detection(img_src: np.array) -> None:
    
    lower_values_red = np.array([0, 103, 45])
    upper_values_red = np.array([12, 255, 255])
    
    _img = np.copy(img_src)
    
    _blurred_img = cv2.GaussianBlur( _img, (5,5), 0)

    _cv_img_hsv = cv2.cvtColor(_blurred_img, cv2.COLOR_BGR2HSV)

    return cv2.inRange( _cv_img_hsv, lower_values_red, upper_values_red)

def find_dimensions_of_fire(red_mask: np.ndarray) -> tuple:
    x_min, y_min, x_max, y_max = bounding_box_vertices(red_mask)

    cv2.rectangle(red_mask, (x_min, y_min), (x_max, y_max), (255, 0, 0), 3)
    cv2.circle(red_mask, ((x_max + x_min) // 2, (y_min + y_max) // 2), 7, (0, 0, 0), -1)

    return x_min, y_min, x_max, y_max

def find_centroid(cv_img: np.ndarray, red_mask: np.ndarray) -> float or None:
    max_area = None
    area = 0
    
    countours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(red_mask, countours, 0, (0,255,0), 3)
    
    for countour in countours: 
        area = cv2.contourArea(countour)
        #print(self.area)
        if area > 10000: 

            if max_area:
                if area > max_area:
                    max_area = area
            else:
                max_area = area
            
            red_moment = cv2.moments(countour)
            # position of the centroid
            red_cx = int(red_moment["m10"]/red_moment["m00"]) 
            red_cy = int(red_moment["m01"]/red_moment["m00"])
            #  Creating a point to represent the centroid 
            cv2.circle(cv_img, (red_cx, red_cy), 7, (0, 255, 255), -1)

    return max_area

def is_centered(x: float, y: float, z: float, red_mask: np.ndarray, goto: Function):
    
    x_min, y_min, x_max, y_max = find_dimensions_of_fire(red_mask)

    center_pixel_x = (x_min + x_max) / 2
    center_pixel_y = (y_min + y_max) / 2

    camera_center_x, camera_center_y = WIDTH//2, HEIGHT//2

    is_on_center = is_close_enough_2d(center_pixel_x, center_pixel_y, camera_center_x, camera_center_y, THRESHOLD_IMAGE)

    if is_on_center:
        camera_width, camera_height = WIDTH, HEIGHT
        
        if x_min >= camera_width*0.1 and x_max <= camera_width*0.9 and y_min >= camera_height*0.1 and y_max <= camera_height*0.9:
            return True

        z += ADDITION
    else:
        real_x, real_y = estimate_3d_coordinates(center_pixel_x, center_pixel_y, z)

        dist = distance_2d(center_pixel_x, center_pixel_y, camera_center_x, camera_center_y)
        max_dist = min(camera_center_x, camera_center_y)

        reduction = min(max(REDUCTION * dist / max_dist, REDUCTION), REDUCTION)

        real_x *= reduction
        real_y *= reduction

        x += real_y
        y += real_x

    goto(x=x, y=y, z=z)
    return False

class Controller:
    def __init__(self) -> None:
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)
        self.set_effect = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)

        self.bridge = CvBridge()

        self.detected = True
        self.state = STARTING

        rospy.Subscriber("/main_camera/image_raw", Image, self.callback)

        self.started = False

        while not self.started:
            pass

        if not self.armed():
            self.goto(z=1, auto_arm=True, frame_id='body')

        input("Start?")

    def goto(self, x=0, y=0, z=0, yaw=0, speed=0.5, frame_id='map', auto_arm=False):
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    def arrived(self, tolerance=0.2):
        telem = self.get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return True

        return False

    def armed(self):
        if self.get_telemetry().armed:
            return True

        return False

    def callback(self, img_msg):
        cv_img_msg = self.bridge.imgmsg_to_cv2(img_msg, 'passthrough')

        self.cv_img = cv2.resize(cv_img_msg, (WIDTH, HEIGHT))

        self.detected = False
        self.started = True

    def display(self):
        cv2.imshow("UAV View", self.cv_img)
        cv2.imshow('Mask Red', self.mask_red)

        k = cv2.waitKey(33)

    def update(self):
        self.update_camera()

        if self.state == STARTING:
            if self.arrived():
                self.goto(x=5, y=0, z=1)
                self.set_effect(effect='blink', r=255, g=0, b=0)
                print("Searching")
                self.state = SEARCHING
        elif self.state == SEARCHING:
            if self.max_fire_area and self.max_fire_area > THRESHOLD:
                position = self.get_telemetry(frame_id='map')
                self.goto(position.x, position.y, position.z)
                self.set_effect(effect='blink', r=255, g=255, b=255)
                print("Centering")
                self.state = CENTERING
        elif self.state == CENTERING:
            position = self.get_telemetry(frame_id='map')
            if is_centered(position.x, position.y, position.z, self.mask_red, self.goto):
                position = self.get_telemetry(frame_id='map')
                self.goto(position.x, position.y, position.z)
                self.state = CENTERED
                self.set_effect(effect='rainbow')
                print("Centered")
        elif self.state == CENTERED:
            pass

    def update_camera(self):

        if self.started and not self.detected:
            self.mask_red = color_detection(self.cv_img)
            self.max_fire_area = find_centroid(self.cv_img, self.mask_red)
            self.display()

            self.detected = True

rospy.init_node('flight')

#print("start")

control = Controller()

try:
    while not rospy.is_shutdown():
        control.update()
except:
    control.land()
    print("\nLanding")

print("end")

'''
center_pixel_x = (x_min + x_max) / 2
center_pixel_y = (y_min + y_max) / 2

camera_center_x, camera_center_y = uav.camera.get_camera_centers()

is_on_center = helper.is_close_enough_2d(center_pixel_x, center_pixel_y, camera_center_x, camera_center_y, self.fire_center_image_size_threshold)

if is_on_center:
    camera_width, camera_height = uav.camera.get_camera_dimensions()
    
    if x_min >= camera_width*0.1 and x_max <= camera_width*0.9 and y_min >= camera_height*0.1 and y_max <= camera_height*0.9:
        return True

    new_z = uav.pos_z+self.fire_centralizing_altitude_addition

    position = [uav.pos_x, uav.pos_y, new_z, 0.0]

    rospy.loginfo('Elevating to {:.0f} m'.format(new_z))
else:
    real_x, real_y = uav.camera.estimate_3d_coordinates(center_pixel_x, center_pixel_y, uav.pos_z)

    dist = helper.distance_2d(center_pixel_x, center_pixel_y, camera_center_x, camera_center_y)
    max_dist = min(camera_center_x, camera_center_y)

    reduction = min(max(self.fire_centralizing_max_reduction * dist / max_dist,
                        self.fire_centralizing_min_reduction),
                    self.fire_centralizing_max_reduction)

    real_x *= reduction
    real_y *= reduction

    position = [uav.pos_x+real_y, uav.pos_y+real_x, uav.pos_z, 0.0]

uav.go_to_point(position)
self.last_sent_fire_position = position

return False
'''
