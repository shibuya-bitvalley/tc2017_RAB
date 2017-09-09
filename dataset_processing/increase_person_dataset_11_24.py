# python library
import os
import getpass
import matplotlib.pyplot as plt
import numpy as np
import imghdr # image file type checker
import random

# OpenCV
import cv2


# high contrast and low contrast table
def cont():

    min_table = 5
    max_table = 210
    diff_table = max_table - min_table

    LUT_HC = np.arange(256, dtype = 'uint8' )
    LUT_LC = np.arange(256, dtype = 'uint8' )

    for i in range(0, min_table):
        LUT_HC[i] = 0
    for i in range(min_table, max_table):
        LUT_HC[i] = 255 * (i - min_table) / diff_table
    for i in range(max_table, 255):
        LUT_HC[i] = 255

    for i in range(256):
        LUT_LC[i] = min_table + i * (diff_table) / 255

    return LUT_HC, LUT_LC


# add gaussian noise
def noise(img):
    row,col,ch= img.shape
    mean = 2
    sigma = 8
    gauss = np.random.normal(mean,sigma,(row,col,ch))
    gauss = gauss.reshape(row,col,ch)
    gauss_img = img + gauss
    return gauss_img


# save label list
def save_label_list(label_list,file_address_name):

    f = open(file_address_name,'w')
    for label in label_list:
        f.write(str(label) + '\n')


# increase dataset
def increase(dataset_path, save_path, label):

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    LUT_HC, LUT_LC = cont()

    im_cnt = 0

    increased_rec_data = []
    increased_label_data = []

    files = sorted(os.listdir(dataset_path))
    data_N = len(files)

    #print files

    for i in range(data_N):

        # original image
        origin_img = cv2.imread(dataset_path+files[i])
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', origin_img)
        increased_label_data.append(label)
        im_cnt += 1

        # high contrast
        high_cont_img = cv2.LUT(origin_img, LUT_HC)
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', high_cont_img)
        increased_label_data.append(label)
        im_cnt += 1

        # low contrast
        low_cont_img = cv2.LUT(origin_img, LUT_LC)
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', low_cont_img)
        increased_label_data.append(label)
        im_cnt += 1

        # noise
        noised_img = noise(origin_img)
        #noised_img = noise(high_cont_img)
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', noised_img)
        increased_label_data.append(label)
        im_cnt += 1

        # smoothing
        average_size = 2
        blur_img = cv2.blur(origin_img, (average_size,average_size))
        #blur_img = cv2.blur(high_cont_img, (average_size,average_size))
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', blur_img)
        increased_label_data.append(label)
        im_cnt += 1

        # flip
        # flipped_img = cv2.flip(origin_img, 1)
        # cv2.imwrite(save_path+str(im_cnt)+'.jpg', flipped_img)
        # increased_label_data.append(label)
        # im_cnt += 1

        # resize
        h, w = origin_img.shape[:2]
        r1 = random.uniform(0.6,1.0)
        r2 = random.uniform(0.6,1.0)
        resized_img = cv2.resize(origin_img, (int(w*r1),int(h*r2)))
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', resized_img)
        increased_label_data.append(label)
        im_cnt += 1

        # resize
        r3 = random.uniform(0.6,1.0)
        r4 = random.uniform(0.6,1.0)
        resized_img = cv2.resize(origin_img, (int(w*r3),int(h*r4)))
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', resized_img)
        increased_label_data.append(label)
        im_cnt += 1

        if i % 100 == 0:
            print str(i) + '/' + str(data_N)

    save_label_list(increased_label_data, save_path+'labels.txt')


# main
if __name__ == '__main__':

    load_path = '../../dataset/images/overlaped/2017_08_18/for_training/humans/original/'
    export_path = '../../dataset/images/overlaped/2017_08_18/for_training/humans/'

    dir_1 = 20
    dir_2 = 24

    # others:0, orange:1, blue:2
    label = 0

    for d in range(dir_1,dir_2+1):

        dataset_path = load_path + str(d) + '/'
        save_path = export_path + str(d) + '/'

        print 'directory: ' + str(d)
        increase(dataset_path, save_path, label)
