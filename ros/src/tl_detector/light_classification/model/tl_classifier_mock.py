import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def mask_image(self, image, lower_range, upper_range):
        # Convert RGB to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only selected range colors
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # # Bitwise-AND mask and original image
        # res = cv2.bitwise_and(image, image, mask=mask)

        return mask

    def detect_yellow(self, img):
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = self.mask_image(img, lower_yellow, upper_yellow)

        return mask

    def detect_red(self, img):
        lower_red_1 = np.array([0, 100, 100])
        upper_red_1 = np.array([10, 255, 255])

        lower_red_2 = np.array([160, 100, 100])
        upper_red_2 = np.array([179, 255, 255])

        mask_1 = self.mask_image(img, lower_red_1, upper_red_1)
        mask_2 = self.mask_image(img, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask_1, mask_2)

        return mask

    def detect_green(self, img):
        lower_green = np.array([37, 38, 100])
        upper_green = np.array([85, 255, 255])
        mask = self.mask_image(img, lower_green, upper_green)

        return mask

    def get_tl_label(self, red_mask, green_mask, yellow_mask):
        green_count = np.count_nonzero(green_mask)
        red_count = np.count_nonzero(red_mask)
        yellow_count = np.count_nonzero(yellow_mask)

        total = green_count + red_count + yellow_count

        color = [red_count, yellow_count, green_count]

        if total < 3:
            return 4

        index = color.index(max(color))

        # if green_count > red_count:
        #     return TrafficLight.GREEN
        #
        # return TrafficLight.RED
        return index

    def get_classification(self, image, detections):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light
            detections: list containing tl detections

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        tl_labels = [0] * 5
        index = 4

        for detection in detections:
            x1, y1, x2, y2 = detection
            crop = image[y1:y2, x1:x2]

            green_mask = self.detect_green(crop)
            red_mask = self.detect_red(crop)
            yellow_mask = self.detect_yellow(crop)

            tl_label = self.get_tl_label(red_mask, green_mask, yellow_mask)
            tl_labels[tl_label] += 1

        if detections:
            # get max vote
            index = tl_labels.index(max(tl_labels))

        # return TrafficLight.UNKNOWN
        return index
