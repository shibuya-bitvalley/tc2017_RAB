# -*- coding: utf-8 -*-
import os
import numpy as np
from numpy.random import *
from PIL import Image
from PIL import ImageOps
import math
import random

import cv2


# overlay on part
def overlay_on_part(src_image, overlay_image, posX, posY):

    ol_height, ol_width = overlay_image.shape[:2]

    src_image_RGBA = cv2.cvtColor(src_image, cv2.COLOR_BGR2RGB)
    overlay_image_RGBA = cv2.cvtColor(overlay_image, cv2.COLOR_BGRA2RGBA)

    src_image_PIL = Image.fromarray(src_image_RGBA)
    overlay_image_PIL = Image.fromarray(overlay_image_RGBA)

    src_image_PIL = src_image_PIL.convert('RGBA')
    overlay_image_PIL = overlay_image_PIL.convert('RGBA')

    tmp = Image.new('RGBA', src_image_PIL.size, (255, 255, 255, 0))
    tmp.paste(overlay_image_PIL, (posX, posY), overlay_image_PIL)
    result = Image.alpha_composite(src_image_PIL, tmp)

    return  cv2.cvtColor(np.asarray(result), cv2.COLOR_RGBA2BGRA)


# rotate
def rotate(img_src,angle):

    img_src = cv2.cvtColor(img_src, cv2.COLOR_BGRA2RGBA)
    img_PIL = Image.fromarray(img_src)
    rot = img_PIL.rotate(angle, expand=True)

    return cv2.cvtColor(np.asarray(rot), cv2.COLOR_RGBA2BGRA)


# mouse event detection
def mouse_event(event, x, y, flags, param):
    global background_img, human_img, overlapped_img, m_x, m_y

    #height, width = human_img.shape[:2]

    if event == cv2.EVENT_LBUTTONUP:
        m_x = x
        m_y = y


# overlap the human image to the background image
def overlap(background_image_path, human_image_path, save_image_path):
    global background_img, human_img, overlapped_img, m_x, m_y

    # read background image
    background_files = os.listdir(background_image_path)
    background_file_index = randint(len(background_files))
    background_img = cv2.imread(background_image_path+background_files[background_file_index])

    # read human image
    human_files = os.listdir(human_image_path)
    human_file_index = randint(len(human_files))
    human_img = cv2.imread(human_image_path+human_files[human_file_index], cv2.IMREAD_UNCHANGED)

    overlapped_img = background_img
    modified_human_img = human_img
    height, width = human_img.shape[:2]

    cnt = 0
    m_x = 0
    m_y = 0

    rate = 1.05

    cv2.namedWindow('overlapping', cv2.WINDOW_NORMAL)
    cv2.imshow('overlapping', background_img)

    cv2.setMouseCallback('overlapping', mouse_event)

    while (True):

        key = cv2.waitKey(1)

        # expansion human image
        if key & 0xFF == ord("p"):
            height = int(height*rate)
            width = int(width*rate)
            modified_human_img = cv2.resize(modified_human_img,(width,height),interpolation = cv2.INTER_LINEAR)

        # shrink human image
        if key & 0xFF == ord("n"):
            height = int(height*1./rate)
            width = int(width*1./rate)
            modified_human_img = cv2.resize(modified_human_img,(width,height),interpolation = cv2.INTER_AREA)

        # expansion human image
        if key & 0xFF == ord("w"):
            width += 5
            modified_human_img = cv2.resize(modified_human_img,(width,height),interpolation = cv2.INTER_LINEAR)

        # expansion human image
        if key & 0xFF == ord("h"):
            height += 5
            modified_human_img = cv2.resize(modified_human_img,(width,height),interpolation = cv2.INTER_LINEAR)

        # flip human image
        if key & 0xFF == ord("f"):
            modified_human_img = cv2.flip(modified_human_img,1)

        # rotate human image left
        if key & 0xFF == ord("l"):
            modified_human_img = rotate(modified_human_img, -5)

        # rotate human image right
        if key & 0xFF == ord("r"):
            modified_human_img = rotate(modified_human_img, 5)

        # reset human image
        if key & 0xFF == ord("o"):
            #height, width = human_img.shape[:2]
            modified_human_img = human_img

        # save overlaped image
        if key & 0xFF == ord("s"):
            cv2.imwrite(save_image_path+str(cnt)+'.png', overlapped_img)
            cnt += 1

        if key & 0xFF == ord("q"):
            break

        #print height, width
        height, width = modified_human_img.shape[:2]
        overlapped_img = overlay_on_part(background_img, modified_human_img, m_x-width/2, m_y-height/2)
        cv2.imshow('overlapping', overlapped_img)

    cv2.destroyAllWindows()


#main
if __name__ == '__main__':

    background_image_path = '../../../dataset/images/overlaped/test_images/backgrounds/'
    human_image_path = '../../../dataset/images/overlaped/test_images/humans/'
    save_image_path = '../../dataset/images/overlaped/test_images/generated/'

    overlap(background_image_path, human_image_path, save_image_path)
