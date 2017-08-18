#!/usr/bin/env python
# coding: UTF-8
import numpy as np
import matplotlib.pyplot as plt
import os
import re

import cv2


def how_many(image_path):

    files = os.listdir(image_path)
    print len(files)


def rename(image_path):

    files = os.listdir(image_path)

    i = 0
    for f in files:
        jpg = re.compile('jpg')
        if jpg.search(f):
            os.rename(image_path+f,image_path+'im'+str(i)+'.jpg')
            i += 1
        else:
            pass


if __name__ == '__main__':

    image_path = '../../../dataset/images/from_video/'

    how_many(image_path)
    #rename(image_path)
