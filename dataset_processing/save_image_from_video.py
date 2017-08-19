import numpy as np
import matplotlib.pyplot as plt
import os

import cv2


def video_to_images(video_path, image_path):

    cap = cv2.VideoCapture(video_path)

    fps = cap.get(5)
    print fps

    cnt = 0
    im_cnt = 0
    d_cnt = 0

    cv2.namedWindow('video_frame', cv2.WINDOW_NORMAL)
    cv2.namedWindow('saved_frame', cv2.WINDOW_NORMAL)

    while(True):
        ret, frame = cap.read()

        if ret is not True:
            break

        cv2.imshow('video_frame', frame)
        cnt += 1

        key = cv2.waitKey(1) & 0xFF

        if key == ord('a'):
            print cnt

        if key == ord('q'):
            break

        #if cnt > skip and cnt %2 == 0:
        if cnt %2 == 0:

            if im_cnt < 2000:
                d_cnt = 1

            elif 2000 <= im_cnt and im_cnt < 4000:
                d_cnt = 2

            elif 4000 <= im_cnt and im_cnt < 6000:
                d_cnt = 3

            elif 6000 <= im_cnt and im_cnt < 8000:
                d_cnt = 4

            elif 8000 <= im_cnt and im_cnt < 10000:
                d_cnt = 5

            elif 10000 <= im_cnt and im_cnt < 12000:
                d_cnt = 6

            elif 12000 <= im_cnt and im_cnt < 14000:
                d_cnt = 7

            elif 14000 <= im_cnt and im_cnt < 16000:
                d_cnt = 8

            elif 16000 <= im_cnt and im_cnt < 18000:
                d_cnt = 9

            elif 18000 <= im_cnt and im_cnt < 20000:
                d_cnt = 10

            elif 20000 <= im_cnt and im_cnt < 22000:
                d_cnt = 11

            elif 22000 <= im_cnt and im_cnt < 24000:
                d_cnt = 12

            elif 24000 <= im_cnt and im_cnt < 26000:
                d_cnt = 13

            elif 26000 <= im_cnt and im_cnt < 28000:
                d_cnt = 14

            else:
                d_cnt = 15

            save_path = image_path +'/'+ str(d_cnt)
            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            cv2.imshow('saved_frame', frame)
            cv2.imwrite(save_path+'/im'+str(im_cnt)+'.jpg', frame)
            im_cnt += 1

    print 'total frame: ' + str(cnt)
    print 'total image: ' + str(im_cnt)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':

    #video_path = '../../dataset/videos/cource_2017.mp4'
    #image_path = '../../dataset/images/from_video/origin'

    # video_path = '../../dataset/videos/2017_08_18/camera_1.avi'
    # image_path = '../../dataset/images/from_video/2017_08_18/camera_1'

    video_path = '../../dataset/videos/2017_08_18/camera_2.avi'
    image_path = '../../dataset/images/from_video/2017_08_18/camera_2'

    print video_path

    video_to_images(video_path, image_path)
