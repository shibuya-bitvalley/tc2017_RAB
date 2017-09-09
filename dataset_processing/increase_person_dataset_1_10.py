# python library
import os
import getpass
import matplotlib.pyplot as plt
import numpy as np
import imghdr # image file type checker
import random

# OpenCV
import cv2


# save label list
def save_label_list(label_list,file_address_name):

    f = open(file_address_name,'w')
    for label in label_list:
        f.write(str(label) + '\n')


# increase dataset
def increase(dataset_path, save_path):

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    label_file = open(dataset_path+'labels.txt','r')
    im_cnt = 0

    label_data = []

    increased_label_data = []

    for label_f in label_file:
        label_str = label_f
        label_data.append(int(label_str))

    data_N = len(label_data)

    for i in range(data_N):

        # original image
        origin_img = cv2.imread(dataset_path+str(i)+'.jpg')
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', origin_img)
        increased_label_data.append(label_data[i])
        im_cnt += 1

        # resize
        h, w = origin_img.shape[:2]
        r1 = random.uniform(0.6,1.0)
        r2 = random.uniform(0.6,1.0)
        resized_img = cv2.resize(origin_img, (int(w*r1),int(h*r2)))
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', resized_img)
        increased_label_data.append(label_data[i])
        im_cnt += 1

        # resize
        r3 = random.uniform(0.6,1.0)
        r4 = random.uniform(0.6,1.0)
        resized_img = cv2.resize(origin_img, (int(w*r3),int(h*r4)))
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', resized_img)
        increased_label_data.append(label_data[i])
        im_cnt += 1

        if i % 100 == 0:
            print str(i) + '/' + str(data_N)

    save_label_list(increased_label_data, save_path+'labels.txt')


# main
if __name__ == '__main__':

    load_path = '../../dataset/images/overlaped/2017_08_18/for_training/humans/original/'
    export_path = '../../dataset/images/overlaped/2017_08_18/for_training/humans/'

    dir_1 = 1
    dir_2 = 10

    for d in range(dir_1,dir_2+1):

        dataset_path = load_path + str(d) + '/'
        save_path = export_path + str(d) + '/'

        print 'directory: ' + str(d)
        increase(dataset_path, save_path)
