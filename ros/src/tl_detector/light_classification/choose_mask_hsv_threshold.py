import os
import glob
import cv2
from lxml import etree
import numpy as np

DATASET_DIR = "./dataset/tl_detector"
ANNOS_DIR = "xmls"
IMGS_DIR = "snaps"
CROPS_DIR = "crops"

label_map_dict = { 'traffic light': 1}
labels = ['traffic light']

def nothing(x):
  pass




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
  upper_red_1 = np.array([100, 255, 255])

  lower_red_2 = np.array([160, 100, 100])
  upper_red_2 = np.array([179, 255, 255])

  mask_1 = mask_image(bgr_img, lower_red_1, upper_red_1)
  mask_2 = mask_image(bgr_img, lower_red_2, upper_red_2)

  mask = cv2.bitwise_or(mask_1, mask_2)
  return  mask


def filter_mask(bgr_img, low, up):
  mask = mask_image(bgr_img, low, up)
  return  mask


def detect_traffic_light(img_path):
  bgr_img = cv2.imread(img_path)
  # bgr_img = cv2.medianBlur(bgr_img, 5)
  # cv2.imwrite("blur.jpg", bgr_img)

  # green_mask = detect_green(bgr_img)
  # red_mask = detect_red(bgr_img)

  # cv2.imwrite("green_mask_2.jpg", green_mask)
  # cv2.imwrite("red_mask_2.jpg", red_mask)

  # circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT, 1, 100, 1)

  cv2.namedWindow('image', cv2.WINDOW_NORMAL)
  cv2.resizeWindow('image', 600, 800)

  # create trackbars for HSV
  cv2.createTrackbar('H_low', 'image', 0, 179, nothing)
  cv2.createTrackbar('S_low', 'image', 0, 255, nothing)
  cv2.createTrackbar('V_low', 'image', 0, 255, nothing)

  cv2.createTrackbar('H_up', 'image', 0, 255, nothing)
  cv2.createTrackbar('S_up', 'image', 0, 255, nothing)
  cv2.createTrackbar('V_up', 'image', 0, 255, nothing)

  while (1):
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
      break

    # get current positions of four trackbars
    h_low = cv2.getTrackbarPos('H_low', 'image')
    s_low = cv2.getTrackbarPos('S_low', 'image')
    v_low = cv2.getTrackbarPos('V_low', 'image')

    h_up = cv2.getTrackbarPos('H_up', 'image')
    s_up = cv2.getTrackbarPos('S_up', 'image')
    v_up = cv2.getTrackbarPos('V_up', 'image')

    lower_thresh = np.array([h_low, s_low, v_low])
    upper_thresh = np.array([h_up, s_up, v_up])

    mask = filter_mask(bgr_img, lower_thresh, upper_thresh)
    cv2.imshow('image', cv2.resize(bgr_img, (600, 800)))
    cv2.imshow('mask', cv2.resize(mask, (600, 800)))
    
  cv2.destroyAllWindows()





if __name__ == "__main__":
  img_path = os.path.join(DATASET_DIR, CROPS_DIR, "667_0.jpeg")
  detect_traffic_light(img_path)




