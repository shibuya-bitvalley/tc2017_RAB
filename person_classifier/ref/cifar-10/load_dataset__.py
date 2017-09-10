#python library
import numpy as np
from numpy.random import *
from matplotlib import pyplot as plt
import cPickle

# OpenCV
import cv2

#python script
import path as p


# load label names
def load_names():

    path = p.data_path()+'batches.meta'
    bm = load_dict(path)
    label_names = bm['label_names']

    return label_names


# load the dictionary-type data
def load_dict(file_path):

    with open(file_path, 'rb') as fo:
        dict = cPickle.load(fo)

    return dict


# separate the dict data into images and labels
def separate_dataset(dict_data):

    img_data = dict_data['data']
    label_data = dict_data['labels']

    return img_data,label_data


# generated permuted array for img and label
def generate_random_array(img_list,label_list):

    random_img_list = []
    random_label_list = []

    data_amount = len(img_list)
    random_index = np.random.permutation(data_amount)

    for n in range(data_amount):

        random_img_list.append(img_list[random_index[n]])
        random_label_list.append(label_list[random_index[n]])

    img_array = (np.array(random_img_list).astype(np.float32))/255
    label_array = np.array(random_label_list).astype(np.int32)

    return img_array,label_array


# load dataset for training CNN
def load_dataset(train_n,validation_n):

    min_batch = 1
    max_batch = 5

    # 1. load all dataset in the range of batch
    img_list = []
    label_list = []

    for batch_number in range(min_batch,max_batch+1):

        path = p.data_path()+'data_batch_'+str(batch_number)
        data = load_dict(path)
        img_data,label_data = separate_dataset(data)

        for i in range(len(img_data)):
            img = img_data[i].reshape((3,32,32))
            img_list.append(img)
            label_list.append(label_data[i])

        print "loaded from data_batch_"+str(batch_number)

    # 2. permutation
    img_array,label_array = generate_random_array(img_list,label_list)

    # 3. pick up dataset for training and validation
    train_img = img_array[0:train_n]
    train_label = label_array[0:train_n]

    validation_img = img_array[train_n+1:train_n+validation_n+1]
    validation_label = label_array[train_n+1:train_n+validation_n+1]

    return train_img,train_label,validation_img,validation_label


# load validation dataset
def load_validation_dataset(path, validation_n):

    img_list = []
    label_list = []

    data = load_dict(path)
    img_data,label_data = separate_dataset(data)

    for i in range(len(img_data)):
        img = img_data[i].reshape((3,32,32))
        img_list.append(img)
        label_list.append(label_data[i])

    # 2. permutation
    img_array,label_array = generate_random_array(img_list,label_list)

    # 3. pick up dataset for test
    validation_img = img_array[0:validation_n]
    validation_label = label_array[0:validation_n]

    return validation_img, validation_label


# display 10 images and labels
def img_show_10(img_data,label_data,name):

    fig,axes = plt.subplots(nrows=2,ncols=5,figsize=(10,5))

    for i in range(10):

        if i < 5:
            graph_r = 0
            graph_c = i
        else:
            graph_r = 1
            graph_c = i-5

        img = (img_data[i]*255).astype(np.uint8)
        img = np.rollaxis(img.reshape((3,32,32)),0,3)

        axes[graph_r,graph_c].imshow(img)
        axes[graph_r,graph_c].set_title(name[label_data[i]])
        axes[graph_r,graph_c].axis('off')


#main
if __name__ == '__main__':

    train_N = 10000
    validation_N = 1000
    test_N = 10000

    name = load_names()

    print "\n"+"loading the dataset.."+"\n"

    tr_img,tr_label,vl_img,vl_label = load_dataset(train_N,validation_N)
    ts_img,ts_label = load_test_dataset(test_N)

    print "\n"+"train_N: " + str(len(tr_img))
    print "validation_N: " + str(len(vl_img)) + "\n"
    print "test_N: " + str(len(ts_img)) + "\n"

    img_show_10(tr_img,tr_label,name)

    plt.show()
