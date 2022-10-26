import sys
import cv2

def read_cam():
    cap = cv2.VideoCapture("-v udpsrc port=1234 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 !  rtph264depay ! decodebin ! videoconvert ! appsink")
    if cap.isOpened():
        cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
        while True:
            ret_val, img = cap.read()
            cv2.imshow('demo',img)
            cv2.waitKey(10)
    else:
     print("camera open failed")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    read_cam()