#!/usr/bin/env python
# coding: UTF-8
import numpy as np
import matplotlib.pyplot as plt
import shutil
import os

import cv2


# count images
def how_many(image_path):

    files = sorted(os.listdir(image_path))
    print files
    print len(files)


# move images
def move(image_path, save_path):

    dirs = sorted(os.listdir(image_path))

    i = 0
    im_cnt_1 = 0
    dr_cnt = 0

    for d in dirs:

        files = sorted(os.listdir(image_path+d))
        print len(files)

        im_cnt_2 = 0
        for f in files:

            if im_cnt_1 < 2000:
                d_cnt = 1

            elif 2000 <= im_cnt_1 and im_cnt_1 < 4000:
                d_cnt = 2

            elif 4000 <= im_cnt_1 and im_cnt_1 < 6000:
                d_cnt = 3

            elif 6000 <= im_cnt_1 and im_cnt_1 < 8000:
                d_cnt = 4

            elif 8000 <= im_cnt_1 and im_cnt_1 < 10000:
                d_cnt = 5

            elif 10000 <= im_cnt_1 and im_cnt_1 < 12000:
                d_cnt = 6

            else:
                d_cnt = 7

            moved_path = save_path +'/'+ str(d_cnt)

            if not os.path.isdir(moved_path):
                os.makedirs(moved_path)

            shutil.copyfile(image_path+d+'/'+f, moved_path+'/'+f)
            im_cnt_1 += 1


# rename image files
def rename(image_path):

    d_list = [str(x + 1) for x in range(13)]
    print d_list

    for d in d_list:

        files = sorted(os.listdir(image_path+d))
        cnt = 0

        for f in files:
            os.rename(image_path+d+'/'+f, image_path+d+'/'+'G'+d+'I'+str(cnt)+'.jpg')
            cnt += 1

    print files[0]




if __name__ == '__main__':

    image_path = '../../dataset/images/backgrounds/2017_08_18/camera_1/'

    #how_many(image_path)
    #move(image_path, save_path)
    rename(image_path)
