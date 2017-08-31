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
    min_table = 10
    max_table = 230
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


# add gauss noise
def noise(img):
    row,col,ch= img.shape
    mean = 0
    sigma = 18
    gauss = np.random.normal(mean,sigma,(row,col,ch))
    gauss = gauss.reshape(row,col,ch)
    gauss_img = img + gauss
    return gauss_img


# main
if __name__ == '__main__':

    dataset_path = '../../dataset/images/overlaped/2017_08_18/for_training/original/'
    save_path = '../../dataset/images/overlaped/2017_08_18/for_training/increased/'

    #d = 1
    # d = 2
    # d = 3
    # d = 4
    # d = 5
    # d = 6
    # d = 7
    d = 10

    dataset_path = dataset_path + str(d) + '/'

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    rec_file = open(dataset_path+'rectangles.txt','r')
    label_file = open(dataset_path+'labels.txt','r')

    rec_data = []
    label_data = []

    # contrast tuning
    #UT_HC, LUT_LC = cont()

    # load RGB image and save GRAY image
    # for i in range(len(name_list)-1):
    #for i in range(1):

        # image_path = RGB_path + name_list[i]

        # print '\n'+image_path+'\n'

        # files = os.listdir(image_path)
        # n_image = len(files)

        # for j in range(n_image):
        # #for j in range(10):

        #     file_name = files[j]
        #     path = image_path +'/'+ file_name

        #     # skip broken data
        #     if imghdr.what(path) != 'jpeg':
        #         continue

        #     rgb_image = cv2.imread(path)

        #     # flip
        #     #fliped_image = cv2.flip(rgb_image, 1)
        #     #cv2.imwrite(image_path+'/fliped_'+file_name, fliped_image)

        #     # contrast tuning
        #     # high_cont_image = cv2.LUT(rgb_image, LUT_HC)
        #     # low_cont_image = cv2.LUT(rgb_image, LUT_LC)
        #     # cv2.imwrite(image_path+'/high_c_'+file_name, high_cont_image)
        #     # cv2.imwrite(image_path+'/low_c_'+file_name, low_cont_image)

        #     # add gauss noise
        #     # noised_image = noise(rgb_image)
        #     # cv2.imwrite(image_path+'/gns_'+file_name, noised_image)

        #     # rotate
        #     #rot_image = rotate(rgb_image)
        #     #cv2.imwrite(image_path+'/rot_'+file_name, rot_image)

        #     # rotate
        #     #rot_image = rotate(rgb_image)
        #     #cv2.imwrite(image_path+'/rot_'+file_name, rot_image)

        #     # resize and crop
        #     r_c_image = resize_and_crop(rgb_image)
        #     cv2.imwrite(image_path+'/res_cr'+file_name, r_c_image)

        #     if j % 10 == 0:
        #         print str(j) + '/' + str(n_image)
