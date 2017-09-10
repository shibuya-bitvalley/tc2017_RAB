#python library
import numpy as np
from matplotlib import pyplot as plt
import cPickle
import csv
import getpass

# OpenCV
import cv2

#python script
#import path as p


# load csv file and return image file name
def load_csv():

    gray_file_name = []
    rgb_file_name = []

    f_gray = open('data/gray_file_name.csv', 'r')
    reader_gray = csv.reader(f_gray)

    f_rgb = open('data/rgb_file_name.csv', 'r')
    reader_rgb = csv.reader(f_rgb)

    for row in reader_gray:
        gray_file_name.append(row)
    f_gray.close()

    for row in reader_rgb:
        rgb_file_name.append(row)
    f_rgb.close()

    gray_file_name = np.array(sorted(gray_file_name)).transpose()
    rgb_file_name = np.array(sorted(rgb_file_name)).transpose()

    return gray_file_name, rgb_file_name


# generate permuted array for img and label
def generate_random_array(gray_file_name, rgb_file_name):

    random_gray_list = []
    random_rgb_list = []

    data_amount = len(gray_file_name)
    random_index = np.random.permutation(data_amount)

    print gray_file_name.shape
    print gray_file_name[8801][0]
    print rgb_file_name[8801][0]
    print random_index[0:10]

    for n in range(data_amount):

        random_gray_list.append(gray_file_name[random_index[n]])
        random_rgb_list.append(rgb_file_name[random_index[n]])

    random_gray_array = np.array(random_gray_list)
    random_rgb_array = np.array(random_rgb_list)

    #return gray_file_name, rgb_file_name
    return random_gray_array, random_rgb_array


# load dataset for training CNN
def load_dataset(train_n,validation_n):

    # 1. load csv file (image file name)
    gray_file_name, rgb_file_name = load_csv()

    print gray_file_name[0:10]
    print rgb_file_name[0:10]

    print
    print 'total image N: ' + str(len(gray_file_name))
    print

    # 2. permutation
    gray_array, rgb_array = generate_random_array(gray_file_name, rgb_file_name)

    print
    print gray_array[0:10]
    print rgb_array[0:10]

    # 3. pick up dataset for training and validation
    train_gray = gray_array[0:train_n]
    train_rgb = rgb_array[0:train_n]

    validation_gray = gray_array[train_n:train_n+validation_n]
    validation_rgb = rgb_array[train_n:train_n+validation_n]

    return train_gray, train_rgb, validation_gray, validation_rgb


# load image data from file name
# def myConverter(gray_array, rgb_array, size):

#     user_name = getpass.getuser()
#     HDD_path = '/media/'+ user_name + '/ELECOM USBHDD/SUN397'
#     RGB_image_path = HDD_path +'/cluster0/RGB/'
#     GRAY_image_path = HDD_path +'/cluster0/GRAY/'

#     newBatch = []

#     for i in range(100):

#         img = cv2.imread(GRAY_image_path + gray_array[i][0])

#         x = cv2.cvtColor(cv2.imread(GRAY_image_path + gray_array[i][0]),cv2.COLOR_BGR2GRAY)
#         y = cv2.cvtColor(cv2.imread(RGB_image_path + rgb_array[i][0]),cv2.COLOR_BGR2RGB)

#         x_img = cv2.resize(x, (size,size), interpolation = cv2.INTER_LINEAR)
#         y_img = cv2.resize(y, (size,size), interpolation = cv2.INTER_LINEAR)

#         x_array = x_img.reshape((1,size,size)).astype(np.float32) / 255.
#         #y_array = np.rollaxis(y_img.reshape((3,size,size)),0,3)
#         #y_array = y_array.reshape((3,size,size)).astype(np.float32) / 255.
#         y_array = y_img.reshape((3,size,size)).astype(np.float32) / 255.

#         newBatch.append((x_array, y_array))

#     return newBatch


def myConverter(gray_array, rgb_array, size):

    user_name = getpass.getuser()
    HDD_path = '/media/'+ user_name + '/ELECOM USBHDD/SUN397'
    RGB_image_path = HDD_path +'/cluster0/RGB/'
    GRAY_image_path = HDD_path +'/cluster0/GRAY/'

    newBatch = []

    for i in range(100):

        img = cv2.imread(GRAY_image_path + gray_array[i][0])

        x = cv2.cvtColor(cv2.imread(GRAY_image_path + gray_array[i][0]),cv2.COLOR_BGR2GRAY)
        y = cv2.cvtColor(cv2.imread(RGB_image_path + rgb_array[i][0]),cv2.COLOR_BGR2RGB)

        x_img = cv2.resize(x, (size,size), interpolation = cv2.INTER_LINEAR)
        y_img = cv2.resize(y, (size,size), interpolation = cv2.INTER_LINEAR)

        x_array = x_img.reshape((1,size,size)).astype(np.float32) / 255.
        #y_array = np.rollaxis(y_img.reshape((3,size,size)),0,3)
        #y_array = y_array.reshape((3,size,size)).astype(np.float32) / 255.
        y_array = y_img.reshape((3,size,size)).astype(np.float32) / 255.

        newBatch.append((x_array, y_array))

    return newBatch



#main
if __name__ == '__main__':

    size = 32

    train_N = 8000
    validation_N = 800

    train_gray, train_rgb, validation_gray, validation_rgb = load_dataset(train_N, validation_N)

    print
    print "train_N: " + str(len(train_gray))
    print "validation_N: " + str(len(validation_gray)) + "\n"
    print

    batch = myConverter(train_gray, train_rgb, size)

    print train_gray[0]
    print train_rgb[0]

    print
    print batch[0][0].shape
    print batch[0][1].shape
    print len(batch)

    gray = (batch[0][0].reshape(size,size)*255).astype(np.uint8)
    rgb = (batch[0][1].reshape(size,size,3)*255).astype(np.uint8)

    fig1 = plt.figure(1)
    plt.imshow(cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB))

    fig2 = plt.figure(2)
    plt.imshow(rgb)

    plt.show()
