# python library
import os
import getpass
import matplotlib.pyplot as plt
import numpy as np
import imghdr # image file type checker
import random


# save list as .txt data
def save_list(label_list,file_address_name):

    f = open(file_address_name,'w')
    for label in label_list:
        f.write(str(label) + '\n')


# dataset
def load_dataset(dataset_path):

    im_cnt = 0

    labels = []
    images = []

    label_file = open(dataset_path+'labels.txt','r')

    for label_f in label_file:
        labels.append(int(label_f))

    data_N = len(labels)

    for i in range(data_N):
        images.append(dataset_path+str(i)+'.jpg')

    return images, labels


# main
if __name__ == '__main__':

    load_path = '../../dataset/images/overlaped/2017_08_18/for_training/humans/'
    #save_path = '../person_classifier/data/'
    save_path = '../person_classifier/data1/'

    dir_1 = 1
    dir_2 = 24

    d_range_1 = range(1,15)
    d_range_2 = [17,20,21,22,23,24]
    d_range_1.extend(d_range_2)

    all_images = []
    all_labels = []

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    #for d in range(dir_1,dir_2+1):
    for d in d_range_1:

        dataset_path = load_path + str(d) + '/'

        print 'directory: ' + str(d)

        images, labels = load_dataset(dataset_path)

        all_images.extend(images)
        all_labels.extend(labels)

    print len(all_images)
    print len(all_labels)

    save_list(all_images, save_path+'images.txt')
    save_list(all_labels, save_path+'labels.txt')
