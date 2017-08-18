# -*- coding: utf-8 -*-
import cv2

def mouse_event(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONUP:
        cv2.circle(img, (x, y), 50, (0, 0, 255), -1)

    elif event == cv2.EVENT_RBUTTONUP and flags & cv2.EVENT_FLAG_SHIFTKEY:
        cv2.putText(img, "CLICK!!", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 255, 0), 3, cv2.CV_AA)

    elif event == cv2.EVENT_RBUTTONUP:
        cv2.rectangle(img, (x-100, y-100), (x+100, y+100), (255, 0, 0), -1)


img = cv2.imread("test.jpg", 1)
cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("img", mouse_event)


while (True):
    cv2.imshow("img", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cv2.destroyAllWindows()
