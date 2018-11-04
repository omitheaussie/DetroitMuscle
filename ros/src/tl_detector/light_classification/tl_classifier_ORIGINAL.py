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
        
        self.detection_graph = None
        self.session = None
        self.image_counter = 0
        self.classes = {1: TrafficLight.RED,
                        2: TrafficLight.YELLOW,
                        3: TrafficLight.GREEN,
                        4: TrafficLight.UNKNOWN}
        self.label_map = load_labelmap('model/trafficsignal.pbtxt')
        NUM_CLASSES = 1
        self.categories = convert_label_map_to_categories(self.label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = create_category_index(self.categories)
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.load_model()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                image_np = load_image_into_numpy_array(image)
                image_np_expanded = np.expand_dims(image_np, axis=0)
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                # Actual detection.
                (boxes, scores, classes, num_detections) = sess.run(
                  [boxes, scores, classes, num_detections],
                  feed_dict={image_tensor: image_np_expanded})
                boxes = np.squeeze(boxes)
                #classes = np.squeeze(classes).astype(np.int32)
                scores = np.squeeze(scores)
                #box_to_color_map = collections.defaultdict(str)
                min_score_thresh = 0.45

                if scores is None or max(scores) > min_score_thresh:
                    lightcolor = "UNKNOWN"
                    box = tuple(boxes[scores.argmax(axis=0)].tolist())
                    #print(box)
                    im_width, im_height = image.size
                    ymin, xmin, ymax, xmax = box
                    (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                                      ymin * im_height, ymax * im_height)

                    light = image.crop((left, top, right, bottom))
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
                    print("Image:",imgno,":  ",end='')
                    print("Red count:", cv2.countNonZero(frame_threshed1) + cv2.countNonZero(frame_threshed2), ":", end='')
                    if cv2.countNonZero(frame_threshed1) + cv2.countNonZero(frame_threshed2) > 50:
                        lightcolor="RED"
                        print(lightcolor)
                        return TrafficLight.RED

                    frame_threshed3 = cv2.inRange(middlelight, YELLOW_MIN, YELLOW_MAX)
                    print("Yellow count:",cv2.countNonZero(frame_threshed3),":",end='')
                    if cv2.countNonZero(frame_threshed3) > 50:
                        lightcolor="YELLOW"
                        print(lightcolor)
                        return TrafficLight.YELLOW

                    frame_threshed4 = cv2.inRange(bottomlight, GREEN_MIN, GREEN_MAX)
                    print("Green count:",cv2.countNonZero(frame_threshed4), ":    ", end='')
                    if cv2.countNonZero(frame_threshed4) > 50:
                        lightcolor="GREEN"
                        print(lightcolor)
                        return TrafficLight.GREEN
                        
        return TrafficLight.UNKNOWN
    
    def load_model(self):
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = '/model/frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    
    def load_labelmap(path):
      with tf.gfile.GFile(path, 'r') as fid:
        label_map_string = fid.read()
        label_map = string_int_label_map_pb2.StringIntLabelMap()
        try:
          text_format.Merge(label_map_string, label_map)
        except text_format.ParseError:
          label_map.ParseFromString(label_map_string)
      _validate_label_map(label_map)
      return label_map

    def _validate_label_map(label_map):
      for item in label_map.item:
        if item.id < 1:
          raise ValueError('Label map ids should be >= 1.')
    
    def convert_label_map_to_categories(label_map,
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

    def create_category_index(categories):
      category_index = {}
      for cat in categories:
        category_index[cat['id']] = cat
      return category_index

    def load_image_into_numpy_array(image):
      (im_width, im_height) = image.size
      return np.array(image.getdata()).reshape(
          (im_height, im_width, 3)).astype(np.uint8)


