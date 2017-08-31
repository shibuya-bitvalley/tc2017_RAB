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


# save rec list
def save_rec_list(rec_list,file_address_name):

    f = open(file_address_name,'w')
    for rec in rec_list:
        f.write(str(rec[0]) + ',' + str(rec[1]) + ',' + \
                str(rec[2]) + ',' + str(rec[3]) + '\n')


# save label list
def save_label_list(label_list,file_address_name):

    f = open(file_address_name,'w')
    for label in label_list:
        f.write(str(label) + '\n')


# increase dataset
def increase(dataset_path, save_path):

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    LUT_HC, LUT_LC = cont()

    rec_file = open(dataset_path+'rectangles.txt','r')
    label_file = open(dataset_path+'labels.txt','r')
    im_cnt = 0

    rec_data = []
    label_data = []

    increased_rec_data = []
    increased_label_data = []

    for rec_f in rec_file:
        rec_str = (rec_f.split(','))
        rec_data.append([int(rec_str[0]),int(rec_str[1]),int(rec_str[2]),int(rec_str[3])])

    for label_f in label_file:
        label_str = label_f
        label_data.append(int(label_str))

    data_N = len(rec_data)

    for i in range(data_N):
    #for i in range(5):

        # original image
        origin_img = cv2.imread(dataset_path+str(i)+'.jpg')
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', origin_img)
        increased_rec_data.append(rec_data[i])
        increased_label_data.append(label_data[i])
        im_cnt += 1

        # high contrast
        high_cont_img = cv2.LUT(origin_img, LUT_HC)
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', high_cont_img)
        increased_rec_data.append(rec_data[i])
        increased_label_data.append(label_data[i])
        im_cnt += 1

        # low contrast
        low_cont_img = cv2.LUT(origin_img, LUT_LC)
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', low_cont_img)
        increased_rec_data.append(rec_data[i])
        increased_label_data.append(label_data[i])
        im_cnt += 1

        # noise
        noised_img = noise(origin_img)
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', noised_img)
        increased_rec_data.append(rec_data[i])
        increased_label_data.append(label_data[i])
        im_cnt += 1

        # smoothing
        average_size = 2
        blur_img = cv2.blur(origin_img, (average_size,average_size))
        cv2.imwrite(save_path+str(im_cnt)+'.jpg', blur_img)
        increased_rec_data.append(rec_data[i])
        increased_label_data.append(label_data[i])
        im_cnt += 1

        if i % 100 == 0:
            print str(i) + '/' + str(data_N)

    save_rec_list(increased_rec_data, save_path+'rectangles.txt')
    save_label_list(increased_label_data, save_path+'labels.txt')


# main
if __name__ == '__main__':

    load_path = '../../dataset/images/overlaped/2017_08_18/for_training/original/'
    export_path = '../../dataset/images/overlaped/2017_08_18/for_training/increased/'

    for d in range(11):

        dataset_path = load_path + str(d+1) + '/'
        save_path = export_path + str(d+1) + '/'

        print 'directory: ' + str(d+1)
        increase(dataset_path, save_path)
