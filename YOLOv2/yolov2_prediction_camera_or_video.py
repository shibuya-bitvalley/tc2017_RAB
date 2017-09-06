#python libraries
import cv2
import numpy as np
import argparse

#chainer
from chainer import serializers, Variable
import chainer.functions as F

#python scripts
from yolov2 import *
from CocoPredictor import *


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='yolov2_darknet_predict for video')
    parser.add_argument('--video_file', '-v', type=str, default=False,help='path to video')
    parser.add_argument('--camera_ID', '-c', type=int, default=0,help='camera ID')
    args = parser.parse_args()

    if not args.video_file == False:
        cap = cv2.VideoCapture(args.video_file)
    else:
        cap = cv2.VideoCapture(args.camera_ID)

    coco_predictor = CocoPredictor()

    cv2.namedWindow("video", cv2.WINDOW_NORMAL)


    while(True):

        ret, orig_img = cap.read()

        if ret is not True:
            break

        nms_results = coco_predictor(orig_img)

        # draw result
        for result in nms_results:
            left, top = result["box"].int_left_top()
            right, bottom = result["box"].int_right_bottom()
            cv2.rectangle(orig_img, (left, top), (right, bottom), (255, 0, 255), 3)
            text = '%s(%2d%%)' % (result["label"], result["probs"].max()*result["conf"]*100)
            cv2.putText(orig_img, text, (left, top-7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            print(text)

        cv2.imshow("video", orig_img)

        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break
