import os
import glob
import cv2
from lxml import etree
import numpy as np
from enum import Enum

DATASET_DIR = "/home/vlad/Documents/ros_site"
ANNOS_DIR = "xmls_coco"
IMGS_DIR = "snaps"
CROPS_DIR = "crops_gama"
LABELS_DIR = "labels_gama"

label_map_dict = { 'traffic light': 1}
labels = ['traffic light']

MASK_SIZE_THRESH = 0.1


class Color(Enum):
    UNKNOWN = 0
    RED = 1
    GREEN = 2

color_labels = {

  Color.UNKNOWN : "UNK",
  Color.RED: "RED",
  Color.GREEN: "GREEN"

  }


# gamma correction function used to reduce high sun exposure
def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)


def adjust_saturation(image, sat):
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  h, s, v = cv2.split(hsv)
  hsv[:, :, 1] = hsv[:, :, 1] * sat
  hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
  return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def preprocess_pipeline(img):
  return adjust_saturation(adjust_gamma(img, gamma=0.4), sat=0.6)


def recursive_parse_xml_to_dict(xml):
  """Recursively parses XML contents to python dict.

  We assume that `object` tags are the only ones that can appear
  multiple times at the same level of a tree.

  Args:
    xml: xml tree obtained by parsing XML file contents using lxml.etree

  Returns:
    Python dictionary holding XML contents.
  """
  if not xml:
    return {xml.tag: xml.text}
  result = {}
  for child in xml:
    child_result = recursive_parse_xml_to_dict(child)
    if child.tag != 'object':
      result[child.tag] = child_result[child.tag]
    else:
      if child.tag not in result:
        result[child.tag] = []
      result[child.tag].append(child_result[child.tag])
  return {xml.tag: result}


def read_xml(xml_anno):

  with open(xml_anno, "r") as f:
      xml_str = f.read()
      xml = etree.fromstring(xml_str)
      data = recursive_parse_xml_to_dict(xml)['annotation']


  width = int(data['size']['width'])
  height = int(data['size']['height'])

  xmin = []
  ymin = []
  xmax = []
  ymax = []
  classes = []
  classes_text = []

  if 'object' in data:
    for obj in data['object']:
      class_name = obj['name']

      xmin.append(int(obj['bndbox']['xmin']))
      ymin.append(int(obj['bndbox']['ymin']))
      xmax.append(int(obj['bndbox']['xmax']))
      ymax.append(int(obj['bndbox']['ymax']))

      classes_text.append(class_name.encode('utf8'))
      classes.append(label_map_dict[class_name])

  return xmin, ymin, xmax, ymax, classes_text, classes, height, width


def mask_image(image, lower_range, upper_range):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only selected range colors
    mask = cv2.inRange(hsv, lower_range, upper_range)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image, image, mask=mask)
    return mask


def detect_red(bgr_img):

  lower_red_1 = np.array([0, 100, 100])
  upper_red_1 = np.array([10, 255, 255])

  lower_red_2 = np.array([160, 100, 100])
  upper_red_2 = np.array([179, 255, 255])

  mask_1 = mask_image(bgr_img, lower_red_1, upper_red_1)
  mask_2 = mask_image(bgr_img, lower_red_2, upper_red_2)

  mask = cv2.bitwise_or(mask_1, mask_2)
  return  mask


def detect_green(bgr_img):
  lower_green = np.array([37, 38, 100])
  upper_green = np.array([85, 255, 255])
  mask = mask_image(bgr_img, lower_green, upper_green)
  return  mask

def check_consintency(red_count, green_count, total, total_pix, ar):
  if total < 3:
    return False
  if ar < 1.7:
    return False

  return True


def get_label(red_mask, green_mask):
  green_count = np.count_nonzero(green_mask)
  red_count = np.count_nonzero(red_mask)

  total = green_count + red_count
  h, w = red_mask.shape[:2]
  total_pix = h * w
  ar = float(h) / w

  # a/r_test should be done here

  if total > 0:
    p_red = float(red_count) / total
    p_green = float(green_count) / total

  print("Red count: {}. Green count {}".format(red_count, green_count))
  print("Total pix {}. Total r+g {}".format(total_pix, total))
  print("A/R {}".format(ar))

  if not check_consintency(red_count, green_count, total, total_pix, ar):
    return Color.UNKNOWN

  if green_count > red_count:
    return Color.GREEN

  return Color.RED


def detect_traffic_light(img_arg, img_in_memory=False):

  if not img_in_memory:
    bgr_img = cv2.imread(img_arg)
  else:
    bgr_img = img_arg

  # bgr_img = cv2.medianBlur(bgr_img, 5)
  # cv2.imwrite("blur.jpg", bgr_img)

  green_mask = detect_green(bgr_img)
  red_mask = detect_red(bgr_img)


  label = get_label(red_mask, green_mask)
  return label


def extract_crops(img_path, xml_file, crops_dir, label_dir):
  img = cv2.imread(img_path)
  xmin, ymin, xmax, ymax, classes_text, classes, height, width = read_xml(xml_file)

  img = preprocess_pipeline(img)

  img_split = os.path.split(img_path)[-1]
  img_no_ext = os.path.splitext(img_split)[0]

  num_obj = len(xmin)

  for i in range(num_obj):
    bbox = [ymin[i], ymax[i], xmin[i], xmax[i]]
    img_crop = img[bbox[0]:bbox[1], bbox[2]:bbox[3], :]
    crop_fname = "{}_{}.png".format(img_no_ext, i)
    crop_absname = os.path.join(crops_dir, crop_fname)

    if not os.path.exists(crop_absname):
      cv2.imwrite(crop_absname, img_crop)

    label = detect_traffic_light(img_crop, img_in_memory=True)
    print(label)
    print(color_labels[label])
    font = cv2.FONT_HERSHEY_SIMPLEX

    color = (255, 255, 255)
    if label == Color.RED:
      color = (0, 0, 255)
    elif label == Color.GREEN:
      color = (0, 255, 0)


    label_absname = os.path.join(label_dir, crop_fname)

    placholder = np.zeros((height, width, 3))
    h, w = img_crop.shape[:2]
    h_offset = height/2 - h / 2
    w_offset = width/2 - w / 2

    placholder[h_offset: h_offset + h, w_offset: w_offset + w, :] = img_crop
    cv2.putText(placholder, color_labels[label], (30,30), font, 1, color, 2, cv2.LINE_AA)

    cv2.imwrite(label_absname, placholder)


def ensure_dir_created(dir_path):
  if not os.path.exists(dir_path):
    os.makedirs(dir_path)

def process_dataset():
  imgs_path = os.path.join(DATASET_DIR, IMGS_DIR)
  annos_path = os.path.join(DATASET_DIR, ANNOS_DIR)
  crops_path = os.path.join(DATASET_DIR, CROPS_DIR)
  labels_path = os.path.join(DATASET_DIR, LABELS_DIR)
  ensure_dir_created(crops_path)
  ensure_dir_created(labels_path)

  imgs = sorted(glob.glob("{}/*.png".format(imgs_path)))

  for img_file in imgs:
    img_split = os.path.split(img_file)[-1]
    img_no_ext = os.path.splitext(img_split)[0]
    xml_file = os.path.join(annos_path, "{}.xml".format(img_no_ext))

    print(img_file)
    print(xml_file)

    if not os.path.exists(xml_file):
      continue

    extract_crops(img_file, xml_file, crops_path, labels_path)


if __name__ == "__main__":
  process_dataset()


  # img_path = os.path.join(DATASET_DIR, CROPS_DIR, "725_0.jpeg")
  # detect_traffic_light(img_path)


