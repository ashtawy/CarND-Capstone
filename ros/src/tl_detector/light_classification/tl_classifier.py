import rospy
from styx_msgs.msg import TrafficLight
import rospkg
import os,sys
import tensorflow as tf
import numpy as np
import time
import os
import cv2


class TLClassifier(object):  
    def __init__(self, is_site=False):       
        self.is_model_loaded = False
        self.tf_session = None
        self.prediction = None
        detect_path = rospkg.RosPack().get_path('tl_detector')
        self.path_to_model = detect_path + '/light_classification/classifiers/'
        self.sample_image_path = detect_path + '/light_classification/cassifiers/classifier_warm_up_image.png'

        if is_site:
           self.path_to_model += 'site_classifier.pb'
        else:
           self.path_to_model += 'simulator_classifier.pb'
        rospy.logwarn('model going to be loaded from '+self.path_to_model)

        # Load the model
        self.tf_graph = tf.Graph()
        with self.tf_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.path_to_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='prefix')

        self.config = tf.ConfigProto(log_device_placement=False)

        self.config.gpu_options.per_process_gpu_memory_fraction = 0.8  
        #Setup timeout for any inactive option 
        self.config.operation_timeout_in_ms = 50000 
        self.image = self.tf_graph.get_tensor_by_name('prefix/image_tensor:0')
        self.class_scores = self.tf_graph.get_tensor_by_name('prefix/detection_scores:0')
        self.classes = self.tf_graph.get_tensor_by_name('prefix/detection_classes:0')

        # Create session
        with self.tf_graph.as_default():     
            self.tf_session = tf.Session(graph=self.tf_graph, config=self.config)        
            
        # warm up the classifier                
        img = cv2.imread(self.sample_image_path, cv2.IMREAD_COLOR)
        self.get_classification(img)        
        self.is_model_loaded = True  


    def get_classification(self, image, score_threshold=0.25):      
        # TrafficLight messages encodes the light classes as: red = 0, yellow = 1, green = 2. 
        # Classifier encodes the light classes as: red = 1, yellow = 2, green = 3
        if not self.is_model_loaded:
            return TrafficLight.UNKNOWN

        fd = {self.image: np.expand_dims(np.asarray(image, dtype=np.uint8), 0)}
        (predicted_scores, predicted_classes) = self.tf_session.run([self.class_scores, self.classes],feed_dict=fd)

        predicted_scores = predicted_scores.flatten()
        predicted_classes = predicted_classes.flatten().astype(np.int32)

        filtered_scores = predicted_scores[predicted_scores > score_threshold]
        filtered_classes = predicted_classes[predicted_scores > score_threshold]

        if len(filtered_classes) > 0:   
            predicted_class = filtered_classes[0] - 1 
        else:
            predicted_class = TrafficLight.UNKNOWN

        return predicted_class     
            