#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
from PIL import Image as pilImage
from timeit import default_timer as timer

STATE_COUNT_THRESHOLD = 3
FREQUENCY_IN_HERTZ = 25.0

class TLDetector(object):
    def __init__(self):
        
        rospy.init_node('tl_detector')
        self.waypoints_2d = None
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.waypoint_tree = None
        self.lights = []
        self.img_proc_time = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        rospy.spin()
        
        
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.img_proc_time > 0:
            self.img_proc_time -= FREQUENCY_IN_HERTZ / 100
            return
        
        self.has_image = True
        self.camera_image = msg
        #
        
        #print("Image type at get_classification:",type(self.camera_image))
        #print("Sensor image:", type(msg))
        #('Sensor image:', <class 'sensor_msgs.msg._Image.Image'>)
        #print('inside image clalback :: ', camera_image)
        starttime = timer()
        light_wp, state = self.process_traffic_lights()
        endtime = timer()
        self.img_proc_time = endtime - starttime
        #print(self.img_proc_time)
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        ######### Light state as sent by the simulator
        #print("Light:", light.state)
        #return light.state
        #########
        
        #print('does this have image :: ', self.has_image)
#         if(not self.has_image):
#             self.prev_light_loc = None
#             return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
          #Call Classifier's code
         #print('just before classification')
         #self.light_classifier = TLClassifier()
        test_light = self.light_classifier.get_classification(cv_image)
        #print(test_light)
        return test_light

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #print("Inside process traffic light")
        #light = None
        closest_light = None
        line_wp_idx = None
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        if(self.pose):
            #car_wp_idx = self.get_closest_waypoint(self.pose)
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                #Stopline waypoint index
                line = stop_line_positions[i]
                #print("LIne:",line)
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                #print('Closest waypoint ', temp_wp_idx)
                #find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                #print('diff d ', d)
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
                    #print('closest light --> ', light, '\n line_wp_idx --> ', line_wp_idx)
                    
        if closest_light :
            #This is where you get the state, the function should call the classifier
            #print('inside closest light 172')
            state = self.get_light_state(closest_light)
            #print('state received ::', state)
            return line_wp_idx, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

