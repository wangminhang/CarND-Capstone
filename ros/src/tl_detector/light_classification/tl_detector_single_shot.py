import tensorflow as tf
import numpy as np
import rospy
import time
import os

class TrafficLightDetectorSingleShot(object):
  def __init__(self, thresh):
    self.thresh = thresh
    inference_graph_path = 'light_classification/model/frozen_inference_graph_single_shot.pb'

    # print(os.path.realpath(__file__))

    # set up tensorflow graph
    self.inference_graph = tf.Graph()

    config = tf.ConfigProto(
      allow_soft_placement=True
    )

    with self.inference_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(inference_graph_path, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

      self.sess = tf.Session(config=config, graph=self.inference_graph)

      self.sess.run(tf.initialize_all_variables())

      # get the detection tensors from the model
      self.image_tensor = self.inference_graph.get_tensor_by_name('image_tensor:0')
      self.boxes = self.inference_graph.get_tensor_by_name('detection_boxes:0')
      self.scores = self.inference_graph.get_tensor_by_name('detection_scores:0')
      self.classes = self.inference_graph.get_tensor_by_name('detection_classes:0')
      self.num_detections = self.inference_graph.get_tensor_by_name('num_detections:0')

  def get_tl_type(self, image):
    """
    :param image: 
    :return: List of boxes
    """

    expanded_image = np.array(np.expand_dims(image, 0))
    height, width = image.shape[:2]

    start = time.time()
    (boxes, scores, classes, num_detections) = self.sess.run([
        self.boxes, self.scores, self.classes, self.num_detections],
        feed_dict={self.image_tensor:expanded_image.astype(np.uint8)})
    end = time.time()

    # batch size is 1, so remove the single-dimensional entry from the shape
    boxes = np.squeeze(boxes)
    scores = np.squeeze(scores)
    classes = np.squeeze(classes)

    detections = []
    scores_final = []
    classes_final = []

    for index in range(len(scores)):
      if scores[index] > self.thresh:
        box = boxes[index]

        [y1, x1, y2, x2] = map(int, [box[0]*height, box[1]*width, box[2]*height, box[3]*width])
        det_w, det_h = abs(x2 - x1), abs(y2 - y1)

        # keep only large detections with aspect ratio > 1.5
        if det_w > 25 and det_h > 25 and 1.0/det_w*det_h > 1.5:
          detections.append([x1, y1, x2, y2])
          scores_final.append(scores[index])
          classes_final.append(classes[index])

    if not detections:
      return 4, [], []

    top = np.argmax(scores_final)
    detected_class = classes_final[top]



    return detected_class - 1, detections[top], scores_final[top]