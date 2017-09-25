# -*- coding: utf-8 -*-

# Python libraries
import os
import numpy as np
from numpy.random import *
from matplotlib import pylab as plt
import random

# OpenCV
import cv2


# save label list
def save_label_list(label_list,file_address_name):

    f = open(file_address_name,'w')
    for label in label_list:
        f.write(str(label) + '\n')


# load the dataset and crop the picture to person rectangle
def crop_dataset(dataset_path, save_path):

    rec_file = open(dataset_path+'rectangles.txt','r')
    label_file = open(dataset_path+'labels.txt','r')

    rec_data = []
    label_data = []

    person_label = []
    person_cnt = 0
    orange_cnt = 0
    blue_cnt = 0

    for rec_f in rec_file:
        rec_str = (rec_f.split(','))
        rec_data.append([int(rec_str[0]),int(rec_str[1]),int(rec_str[2]),int(rec_str[3])])

    for label_f in label_file:
        label_str = label_f
        label_data.append(int(label_str))

    data_N = len(rec_data)
    print data_N

    for i in range(data_N):

        img = cv2.imread(dataset_path+str(i)+'.jpg')

        if not label_data[i] == 0:

            if label_data[i] == 1:
                person_label.append(1)
                orange_cnt += 1

            elif label_data[i] == 2:
                person_label.append(2)
                blue_cnt += 1

            x = rec_data[i][0] - rec_data[i][2]/2
            y = rec_data[i][1] - rec_data[i][3]/2
            w = rec_data[i][2]
            h = rec_data[i][3]

            cv2.imwrite(save_path+str(person_cnt)+'.jpg', img[y:y+h, x:x+w])
            person_cnt += 1

        if i % 500 == 0 :
            print str(i)+'/'+str(data_N)

    save_label_list(person_label, save_path+'labels.txt')

    print 'orange person: '+str(orange_cnt)
    print 'blue person: '+str(blue_cnt)


# main
if __name__ == '__main__':

    dataset_path = '../../dataset/images/overlaped/2017_08_18/for_training/increased/'
    export_path = '../../dataset/images/overlaped/2017_08_18/for_training/humans/'

    for d in range(10):

        print
        print 'directory: '+str(d+1)

        image_path = dataset_path + str(d+1) + '/'
        save_path = export_path + str(d+1) + '/'

        if not os.path.isdir(save_path):
            os.makedirs(save_path)

        crop_dataset(image_path, save_path)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
