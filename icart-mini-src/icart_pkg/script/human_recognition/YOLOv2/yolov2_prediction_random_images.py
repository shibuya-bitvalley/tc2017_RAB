# Python libraries
import time
import numpy as np
import argparse
import random
import os

# chainer
from chainer import serializers, Variable, cuda
import chainer.functions as F

# OpenCV
import cv2

# Python scripts
from yolov2 import *
from CocoPredictor import *


if __name__ == "__main__":

    dataset_path = '../../dataset/images/overlaped/2017_08_18/for_training/increased/'
    save_path = 'results/'

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    save_cnt = 0

    # load YOLOv2 trained model
    start = time.time()
    predictor = CocoPredictor()
    elapsed_time = time.time() - start
    print ("elapsed time to load model:{0:.5f}".format(elapsed_time) + "[sec]")

    while (True):

        end_flag = False

        # load image
        dir_n  = random.randint(1,10)
        image_n = random.randint(0,1000)
        image_file = dataset_path + str(dir_n) + '/' + str(image_n) + '.jpg'
        print("\n"+"loading image...")
        orig_img = cv2.imread(image_file)

        # predict
        start = time.time()
        nms_results = predictor(orig_img)
        elapsed_time = time.time() - start

        print ("elapsed time to predict:{0:.5f}".format(elapsed_time) + "[sec]")

        # draw result
        for result in nms_results:

            left, top = result["box"].int_left_top()

            # search "person"
            if result["class_id"] == 0:
                color = (0,255,0)
            else:
                color = (221,184,90)

            cv2.rectangle(orig_img, result["box"].int_left_top(),
                          result["box"].int_right_bottom(), color, 3)

            text = '%s(%2d%%)' % (result["label"], result["probs"].max()*result["conf"]*100)
            cv2.putText(orig_img, text, (left, top-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            print(text)

        cv2.imshow("detection result", orig_img)

        while (True):

            key = cv2.waitKey(1)

            # load next image
            if key & 0xFF == ord("z"):
                break

            # save detection result
            if key & 0xFF == ord("s"):
                print("save the detection result")
                cv2.imwrite(save_path+"yolov2_"+str(save_cnt)+".jpg", orig_img)
                save_cnt += 1

            # finish this program
            if key & 0xFF == 27:
                end_flag = True
                break

        if end_flag == True:
            break
