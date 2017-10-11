import numpy as np
import matplotlib.pyplot as plt
import os

import cv2


def video_to_images(video_path, image_path):

    cap = cv2.VideoCapture(video_path)

    cnt = 0

    cv2.namedWindow('video_frame', cv2.WINDOW_NORMAL)
    cv2.namedWindow('saved_frame', cv2.WINDOW_NORMAL)

    while(True):
        ret, frame = cap.read()
        key = cv2.waitKey(25) & 0xFF

        if ret is not True:
            break

        cv2.imshow('video_frame', frame)
        cnt += 1

        if key == ord('a'):
            print cnt

        if key == ord('s'):
            cv2.imshow('saved_frame', frame)
            cv2.imwrite(image_path+'im'+str(cnt)+'.jpg', frame)

        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':

    video_path = 'videos/1/camera_0.avi'
    image_path = 'images/'

    if not os.path.isdir(image_path):
        os.makedirs(image_path)

    print video_path

    video_to_images(video_path, image_path)
