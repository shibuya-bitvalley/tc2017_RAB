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
    #fourcc = cv2.cv.CV_FOURCC('F','L','V','1')
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    rec_ = cv2.VideoWriter(save_path+'camera_'+str(camera_ID)+'.avi', \
                          fourcc, fps, (w, h))
    return rec_


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
    parser.add_argument('--video_name', '-n', type=str, \
                        default=1,help='video file name')
    parser.add_argument('--camera_ID', '-i', nargs='*', type=int,\
                        default=0,help='camera id')
    args = parser.parse_args()

    ID_list = args.camera_ID

    w = 480
    h = 270
    b = 0.498039215803
    c = 0.6
    s = 0.3
    f = 30

    camera_N = len(ID_list)

    cap = [0]*camera_N
    rec = [0]*camera_N

    for i in range(camera_N):
        cap[i], rec[i] = capture(ID_list[i], args.video_name, w, h, b, c, s, f)

    save_flag = False

    ret = [0]*camera_N
    frame = [0]*camera_N

    while(True):

        for i in range(camera_N):
            ret[i], frame[i] = cap[i].read()

            if ret[i] == False:
                print 'error: camera ' + str(ID_list[i])
                break

            cv2.imshow('camera:'+str(ID_list[i]), frame[i])

        k = cv2.waitKey(1)

        if k & 0xFF == ord('q'):
            break

        if k & 0xFF == ord('s'):
            print 'start recording'
            save_flag = True

        if save_flag:
            for i in range(camera_N):
                rec[i].write(frame[i])

    print 'finished'
    for i in range(camera_N):
        cap[i].release()
        rec[i].release()
