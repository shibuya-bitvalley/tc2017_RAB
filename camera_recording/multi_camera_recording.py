#python library
import numpy as np
import argparse
import os

#OpenCV
import cv2


# preparing to save the video
def initWriter(camera_ID, w, h, fps, save_path):
    #fourcc = cv2.cv.CV_FOURCC('D','I','B',' ')
    #fourcc = cv2.cv.CV_FOURCC('D','I','V','X')
    fourcc = cv2.cv.CV_FOURCC('F','L','V','1')
    rec = cv2.VideoWriter(save_path+'camera_'+str(camera_ID)+'.avi', \
                          fourcc, fps, (w, h))
    return rec


# capture the video
def capture(camera_ID, video_number, width, height, brightness, contrast, saturation, fps):

    cap = cv2.VideoCapture(camera_ID)

    print '\n'+'camera: '+ str(camera_ID)
    print 'dafault value'
    print 'width: '+str(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
    print 'height: '+str(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
    #print 'frame rate: '+str(cap.get(cv2.cv.CV_CAP_PROP_FPS))
    print 'brightness: '+str(cap.get(cv2.cv.CV_CAP_PROP_BRIGHTNESS))
    print 'contrast: '+str(cap.get(cv2.cv.CV_CAP_PROP_CONTRAST))
    print 'saturation: '+str(cap.get(cv2.cv.CV_CAP_PROP_SATURATION))+'\n'

    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.cv.CV_CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.cv.CV_CAP_PROP_SATURATION, saturation)
    cap.set(cv2.cv.CV_CAP_PROP_FPS, fps)

    save_path = 'videos/' + str(video_number) + '/'

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    rec = initWriter(camera_ID, width, height, fps, save_path)
    cv2.namedWindow('camera:'+str(camera_ID), cv2.WINDOW_NORMAL)

    return cap, rec


#main
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Camera streaming')
    parser.add_argument('--video_number', '-n', type=int, \
                        default=1,help='Number of trial')
    args = parser.parse_args()

    w = 480
    h = 270
    b = 0.498039215803
    c = 0.6
    s = 0.3
    f = 30

    # camera 0
    #i0= 0

    # camera 1
    i1= 1

    # camera 2
    i2= 2

    #cap_0, rec_0 = capture(0, args.video_number, w, h, b, c, s, f)
    cap_1, rec_1 = capture(1, args.video_number, w, h, b, c, s, f)
    cap_2, rec_2 = capture(2, args.video_number, w, h, b, c, s, f)

    save_flag = False

    while(True):

        #ret_0, frame_0 = cap_0.read()
        ret_1, frame_1 = cap_1.read()
        ret_2, frame_2 = cap_2.read()


        # if ret_0 == False:
        #     print 'error: camera ' + str(i0)
        #     break

        if ret_1 == False:
            print 'error: camera ' + str(i1)
            break

        if ret_2 == False:
            print 'error: camera ' + str(i2)
            break

        #cv2.imshow('camera:'+str(i0), frame_0)
        cv2.imshow('camera:'+str(i1), frame_1)
        cv2.imshow('camera:'+str(i2), frame_2)

        k = cv2.waitKey(1)

        if k & 0xFF == ord('q'):
            break

        if k & 0xFF == ord('s'):
            print 'start recording'
            save_flag = True

        if save_flag:
            #rec_0.write(frame_0)
            rec_1.write(frame_1)
            rec_2.write(frame_2)

    #cap_0.release()
    #rec_0.release()

    cap_1.release()
    rec_1.release()

    cap_2.release()
    rec_2.release()
