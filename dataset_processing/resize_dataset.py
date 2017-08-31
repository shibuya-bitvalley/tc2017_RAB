# -*- coding: utf-8 -*-

# Python libraries
import os
import numpy as np
from numpy.random import *
from matplotlib import pylab as plt
import random

# OpenCV
import cv2


# resize dataset
def resize(img, rec, size):

    height, width = img.shape[:2]

    x_ratio = size/float(width)
    y_ratio = size/float(height)

    img = cv2.resize(img, (size, size), interpolation = cv2.INTER_AREA)
    resized_rec = [int(rec[0]*x_ratio), int(rec[1]*y_ratio), \
                   int(rec[2]*x_ratio), int(rec[3] * y_ratio)]

    return img, resized_rec


# draw rectangle
def draw_rectangle(img, rec, rec_color, t):

    cv2.rectangle(img, (rec[0] - rec[2]/2, rec[1] - rec[3]/2), \
    (rec[0] + rec[2]/2, rec[1] + rec[3]/2), rec_color, t)

    return img


# load the dataset and display
def display_resized_dataset(dataset_path, save_path):

    size = 224

    rec_file = open(dataset_path+'rectangles.txt','r')
    label_file = open(dataset_path+'labels.txt','r')

    rec_data = []
    label_data = []

    for rec_f in rec_file:
        rec_str = (rec_f.split(','))
        rec_data.append([int(rec_str[0]),int(rec_str[1]),int(rec_str[2]),int(rec_str[3])])

    for label_f in label_file:
        label_str = label_f
        label_data.append(int(label_str))

    index_list = random.sample(xrange(len(rec_data)), 2)

    for index in index_list:

        img = cv2.imread(dataset_path+str(index)+'.jpg')
        rec_img = img

        if label_data[index] == 1: rec_color = (0,165,255)
        elif label_data[index] == 2: rec_color = (255,125,86)
        else: rec_color = (0,0,0)

        resized_img, resized_rec = resize(img, rec_data[index], size)

        rec_img = draw_rectangle(rec_img, rec_data[index], rec_color, 10)
        resized_rec_img = draw_rectangle(resized_img, resized_rec, rec_color, 2)

        cv2.namedWindow('original data '+str(index), cv2.WINDOW_NORMAL)
        cv2.imshow('original data '+str(index),rec_img)
        cv2.imwrite(save_path+'original_'+str(index)+'.jpg', rec_img)

        cv2.namedWindow('resized data '+str(index), cv2.WINDOW_NORMAL)
        cv2.imshow('resized data '+str(index),resized_rec_img)
        cv2.imwrite(save_path+'resized_'+str(index)+'.jpg', resized_rec_img)


# main
if __name__ == '__main__':

    #dataset_path = '../../dataset/images/overlaped/cource_2017/for_training/sample/'
    #dataset_path = '../../dataset/images/overlaped/cource_2017/for_training/'

    #dataset_path = '../../dataset/images/overlaped/2017_08_18/for_training/sample/'
    dataset_path = '../../dataset/images/overlaped/2017_08_18/for_training/original/'

    #d = 1
    # d = 2
    # d = 3
    # d = 4
    # d = 5
    # d = 6
    # d = 7
    d = 10

    dataset_path = dataset_path + str(d) + '/'

    save_path = '../../dataset/images/overlaped/2017_08_18/for_training/resized/'

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    display_resized_dataset(dataset_path, save_path)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
