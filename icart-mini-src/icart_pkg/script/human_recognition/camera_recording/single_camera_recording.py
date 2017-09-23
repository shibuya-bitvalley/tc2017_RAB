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
    rec = cv2.VideoWriter(save_path+'camera_'+str(camera_ID)+'.avi', \
                          fourcc, fps, (w, h))
    return rec


# capture the video
def capture(camera_ID, video_number):

    width = 320
    height = 240
    # brightness = 0.498039215803
    # contrast = 0.6
    # saturation = 0.3
    # codec = 0x47504A4D
    fps = 30

    cap = cv2.VideoCapture(camera_ID)

    print '\n'+'dafault value'
    print 'width: '+str(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
    print 'height: '+str(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
    #print 'frame rate: '+str(cap.get(cv2.cv.CV_CAP_PROP_FPS))
    print 'brightness: '+str(cap.get(cv2.cv.CV_CAP_PROP_BRIGHTNESS))
    print 'contrast: '+str(cap.get(cv2.cv.CV_CAP_PROP_CONTRAST))
    print 'saturation: '+str(cap.get(cv2.cv.CV_CAP_PROP_SATURATION))+'\n'

    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
    # cap.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, brightness)
    # cap.set(cv2.cv.CV_CAP_PROP_CONTRAST, contrast)
    # cap.set(cv2.cv.CV_CAP_PROP_SATURATION, saturation)
    #cap.set(cv2.cv.CV_CAP_PROP_FOURCC('M','J','P','E','G'))
    cap.set(cv2.cv.CV_CAP_PROP_FPS, fps)

    save_path = 'videos/' + str(video_number) + '/'

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    rec = initWriter(camera_ID, width, height, fps, save_path)
    cv2.namedWindow('camera:'+str(camera_ID), cv2.WINDOW_NORMAL)
    save_flag = False

    while(True):

        ret, frame = cap.read()
        #frame = cv2.flip(frame, 1)

        if ret == False:
            print 'error'
            break

        cv2.imshow('camera:'+str(camera_ID), frame)

        k = cv2.waitKey(1)

        if k & 0xFF == ord('q'):
            break

        if k & 0xFF == ord('s'):
            print 'start recording'
            save_flag = True

        if save_flag:
            rec.write(frame)

    cap.release()
    rec.release()


#main
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Camera streaming')
    parser.add_argument('--ID', '-i', type=int, default=0,help='Camera ID')
    parser.add_argument('--video_name', '-n', type=str, default=1,help='video file name')
    args = parser.parse_args()

    capture(args.ID, args.video_name)
    #cv2.dstroyAllWindows()
