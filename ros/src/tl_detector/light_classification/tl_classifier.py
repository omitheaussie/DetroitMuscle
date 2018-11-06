from styx_msgs.msg import TrafficLight
import datetime
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import rospy
import yaml
from collections import defaultdict
from io import StringIO
#from matplotlib import pyplot as plt
#import matplotlib as pl
from PIL import Image
import cv2
import string_int_label_map_pb2
from google.protobuf import text_format
import glob
import collections
from cv_bridge import CvBridge
from timeit import default_timer as timer

from styx_msgs.msg import TrafficLight
RED_MIN1 = np.array([0, 100, 100],np.uint8)
RED_MAX1 = np.array([10, 255, 255],np.uint8)        

RED_MIN2 = np.array([160, 100, 100],np.uint8)
RED_MAX2 = np.array([179, 255, 255],np.uint8)

YELLOW_MIN = np.array([40.0/360*255, 100, 100],np.uint8)
YELLOW_MAX = np.array([66.0/360*255, 255, 255],np.uint8)

GREEN_MIN = np.array([90.0/360*255, 100, 100],np.uint8)
GREEN_MAX = np.array([140.0/360*255, 255, 255],np.uint8)
# Blue color HSV space limits: [ 84  14 211] [104  34 291]
BLUE_MIN = np.array([84,14,211],np.uint8)
BLUE_MAX = np.array([104,34,291],np.uint8)

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        print("Inside init")
        self.detection_graph = None
        self.session = None
        self.image_counter = 0
        self.bridge = CvBridge()
        self.classes = {1: TrafficLight.RED,
                        2: TrafficLight.YELLOW,
                        3: TrafficLight.GREEN,
                        4: TrafficLight.UNKNOWN}
        self.prevclass = TrafficLight.UNKNOWN
        
        self.label_map = self.load_labelmap('model/trafficsignal.pbtxt')
        NUM_CLASSES = 1
        self.categories = self.convert_label_map_to_categories(self.label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = self.create_category_index(self.categories)
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        self.load_model()
        #self.sess = tf.Session()
        #tf.saved_model.loader.load(self.sess, [], 'model/frozen_inference_graph.pb')
        #self.detection_graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        #cvimage = self.bridge.imgmsg_to_cv2(image, "rgb8")
        
        
        #print("Image classification")
        if (self.image_counter % 1) == 0:
            image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            image = cv2.resize(image,(400,300))
            image = Image.fromarray(image)
            lightcolor = "UNKNOWN"
            self.prevclass = TrafficLight.UNKNOWN
            current_time=datetime.datetime.now()
#            with self.detection_graph.as_default():
#                with tf.Session(graph=self.detection_graph) as sess:
            image_np = self.load_image_into_numpy_array(image)
            image_np_expanded = np.expand_dims(image_np, axis=0)
            image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

            # Actual detection.
            (boxes, scores, classes, num_detections) = self.session.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})
            boxes = np.squeeze(boxes)
            #classes = np.squeeze(classes).astype(np.int32)
            scores = np.squeeze(scores)
            #box_to_color_map = collections.defaultdict(str)
            min_score_thresh = 0.45
            #print(max(scores))
            if scores is None or max(scores) > min_score_thresh:
                #print("Score is more than thresh")
                box = tuple(boxes[scores.argmax(axis=0)].tolist())
                #print(box)
                im_width, im_height = image.size
                ymin, xmin, ymax, xmax = box
                (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                                              ymin * im_height, ymax * im_height)
                #print(left, right, top, bottom)
                light = image.crop((int(left), int(top), int(right), int(bottom)))
                light = light.resize((40,90))
                #light = cv2.inRange(light, RED_MIN1, RED_MAX1) 

                toplight = light.crop((0,0,40,30))
                middlelight = light.crop((0,30,40,60))
                bottomlight = light.crop((0,60,40,90))

                toplight = np.array(toplight)[:, :, ::-1].copy()
                middlelight = np.array(middlelight)[:, :, ::-1].copy()
                bottomlight = np.array(bottomlight)[:, :, ::-1].copy()

                toplight = cv2.cvtColor(toplight,cv2.COLOR_BGR2HSV)
                middlelight = cv2.cvtColor(middlelight,cv2.COLOR_BGR2HSV)
                bottomlight = cv2.cvtColor(bottomlight,cv2.COLOR_BGR2HSV)

                # red has hue 0 - 10 & 160 - 180 add another filter 
                # TODO  use Guassian mask
                frame_threshed1 = cv2.inRange(toplight, RED_MIN1, RED_MAX1) 
                frame_threshed2 = cv2.inRange(toplight, RED_MIN2, RED_MAX2)
                #print("Image:",imgno,":  ",end='')
                #print("Red count:", cv2.countNonZero(frame_threshed1) + cv2.countNonZero(frame_threshed2), ":", end='')
                if cv2.countNonZero(frame_threshed1) + cv2.countNonZero(frame_threshed2) > 50:
                    lightcolor="RED"
                    self.prevclass = TrafficLight.RED
                    print(lightcolor)

                frame_threshed3 = cv2.inRange(middlelight, YELLOW_MIN, YELLOW_MAX)
                #print("Yellow count:",cv2.countNonZero(frame_threshed3),":",end='')
                if cv2.countNonZero(frame_threshed3) > 50:
                    lightcolor="YELLOW"
                    self.prevclass = TrafficLight.YELLOW
                    print(lightcolor)

                frame_threshed4 = cv2.inRange(bottomlight, GREEN_MIN, GREEN_MAX)
                #print("Green count:",cv2.countNonZero(frame_threshed4), ":    ", end='')
                if cv2.countNonZero(frame_threshed4) > 50:
                    lightcolor="GREEN"
                    self.prevclass = TrafficLight.GREEN
                    print(lightcolor)
                #print("Time passed:", (datetime.datetime.now()-current_time).total_seconds())
        self.image_counter += 1      
        return self.prevclass
    
    def load_model(self):
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = 'model/frozen_inference_graph_FullSizeImg.pb'
        #src/tl_detector/light_classification/
        self.detection_graph = tf.Graph()
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        with tf.Session(graph=self.detection_graph, config=config) as sess:
          self.session = sess
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    
    def load_labelmap(self, path):
      with tf.gfile.GFile(path, 'r') as fid:
        label_map_string = fid.read()
        label_map = string_int_label_map_pb2.StringIntLabelMap()
        try:
          text_format.Merge(label_map_string, label_map)
        except text_format.ParseError:
          label_map.ParseFromString(label_map_string)
      self._validate_label_map(label_map)
      return label_map

    def _validate_label_map(self, label_map):
      for item in label_map.item:
        if item.id < 1:
          raise ValueError('Label map ids should be >= 1.')
    
    def convert_label_map_to_categories(self, label_map,
                                        max_num_classes,
                                        use_display_name=True):
      categories = []
      list_of_ids_already_added = []
      if not label_map:
        label_id_offset = 1
        for class_id in range(max_num_classes):
          categories.append({
              'id': class_id + label_id_offset,
              'name': 'category_{}'.format(class_id + label_id_offset)
          })
        return categories
      for item in label_map.item:
        if not 0 < item.id <= max_num_classes:
          logging.info('Ignore item %d since it falls outside of requested '
                       'label range.', item.id)
          continue
        if use_display_name and item.HasField('display_name'):
          name = item.display_name
        else:
          name = item.name
        if item.id not in list_of_ids_already_added:
          list_of_ids_already_added.append(item.id)
          categories.append({'id': item.id, 'name': name})
      return categories

    def create_category_index(self, categories):
      category_index = {}
      for cat in categories:
        category_index[cat['id']] = cat
      return category_index

    def load_image_into_numpy_array(self, image):
      
      #print("Image size:",image.size, image.shape)
      (im_width, im_height) = image.size
      return np.array(image.getdata()).reshape(
          (im_height, im_width, 3)).astype(np.uint8)


