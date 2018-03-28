import tensorflow as tf
import numpy as np

import time
import copy

import os
import glob

from lxml import etree

DATA_DIR = '/home/user2/Documents/tl_detector/snaps'
XMLS_DIR = '/home/user2/Documents/tl_detector/ssd_large/xmls'
DETS_DIR = '/home/user2/Documents/tl_detector/ssd_large/detections'

image_names = glob.glob(DATA_DIR + '/*.jpeg')

outfile = os.path.join(DETS_DIR, 'filelist.txt')

if not os.path.exists(XMLS_DIR):
    os.makedirs(XMLS_DIR)

if not os.path.exists(DETS_DIR):
    os.makedirs(DETS_DIR)

f = open(outfile, 'w')

def load_inference_graph(inference_graph_path):
    od_graph = tf.Graph()
    with od_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(inference_graph_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return od_graph

def visualize(sess, image, scres, bxes, clases, impath):

    tokens = impath.split('/')
    imname = tokens[-1]
    xmlpath = os.path.join(XMLS_DIR, imname.split('.')[0] + '.xml')

    root = etree.Element('annotation')

    filename_child = etree.SubElement(root, 'filename')
    filename_child.text = imname

    size_child = etree.SubElement(root, 'size')
    width_child = etree.SubElement(size_child, 'width')
    width_child.text = '800'
    height_child = etree.SubElement(size_child, 'height')
    height_child.text = '600'
    depth_child = etree.SubElement(size_child, 'depth')
    depth_child.text = '3'

    for batch_index in range(len(scres)):
        img = copy.copy(image)

        height, width = image.shape[:2]

        scores_b = scres[batch_index]
        boxes_b = bxes[batch_index]
        classes_b = clases[batch_index]

        for index in range(len(scores_b)):
            # if int(classes_b[index]) != 10: continue

            if scores_b[index] > 0.7:
                box = boxes_b[index]

                [y1, x1, y2, x2] = [box[0] * height, box[1] * width, box[2] * height, box[3] * width]
                [y1, x1, y2, x2] = [int(y1), int(x1), int(y2), int(x2)]

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                object_child = etree.SubElement(root, 'object')
                name_child = etree.SubElement(object_child, 'name')
                name_child.text = 'traffic light'

                bndbox_child = etree.SubElement(object_child, 'bndbox')
                xmin_child = etree.SubElement(bndbox_child, 'xmin')
                xmin_child.text = str(x1)
                ymin_child = etree.SubElement(bndbox_child, 'ymin')
                ymin_child.text = str(y1)
                xmax_child = etree.SubElement(bndbox_child, 'xmax')
                xmax_child.text = str(x2)
                ymax_child = etree.SubElement(bndbox_child, 'ymax')
                ymax_child.text = str(y2)

                f.write('{} {} {} {} {} {} {}\n'.format(impath, str(index),
                    str(int(classes_b[index])), str(x1), str(y1), str(x2), str(y2)))

        et = etree.ElementTree(root)
        et.write(xmlpath, pretty_print=True)
        cv2.imwrite(os.path.join(DETS_DIR, imname), img)

def placeholder(vis=False):
    inference_graph = load_inference_graph(inference_graph_path)
    config = tf.ConfigProto(allow_soft_placement=True)

    times = []

    with tf.Session(config=config, graph=inference_graph) as sess:
        sess.run(tf.initialize_all_variables())

        image_tensor = inference_graph.get_tensor_by_name('image_tensor:0')
        boxes = inference_graph.get_tensor_by_name('detection_boxes:0')
        scores = inference_graph.get_tensor_by_name('detection_scores:0')
        classes = inference_graph.get_tensor_by_name('detection_classes:0')
        num_detections = inference_graph.get_tensor_by_name('num_detections:0')

        for index, image_path in enumerate(image_names):
            print('Started {} {}'.format(index, image_path))

            image=cv2.imread(image_path)

            image1 = np.array(np.expand_dims(image,0))
            start = time.time()
            (bxes, scres, cls, num_det) = sess.run(
                [boxes, scores, classes, num_detections],feed_dict={image_tensor:image1.astype(np.uint8)})

            stop = time.time()

            if vis:
                visualize(sess, image, scres, bxes, cls, image_path)

            if index > 10:
                times.append(stop - start)

            print('Done {} {}'.format(index, image_path))

    avg_time = np.sum(times) / len(times)
    print('Feeding input placeholder avg time %f s' % avg_time)

import cv2
inference_graph_path = '/home/user2/dev/tensorflow/tensorflow/models/research/object_detection/graphs/traffic_light_ssd_mobilenet_v1_large/frozen_inference_graph.pb'
print('Given pb : %s' % inference_graph_path)

placeholder(vis=True)

f.close()