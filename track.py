import cv2
import numpy as np
import math
import serial
import time

timing = time.time()

if __name__ == '__main__':
    def nothing(*arg):
        pass

ser = serial.Serial('COM6', 9600)
#time.sleep(2)
ser.reset_input_buffer()

cv2.namedWindow("result")  # создаем главное окно

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)
hsv_min = np.array((141, 111, 180), np.uint8)
hsv_max = np.array((175, 203, 255), np.uint8)

color_yellow = (0, 255, 255)
movingX = False
movingY = False
movingR = False

while True:
    success, img = cap.read()
    #img = cv2.flip(img, 0)  # переворот кадра
    img = cv2.flip(img, 1)  # отражение кадра вдоль оси Y
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, hsv_min, hsv_max)

    moments = cv2.moments(thresh, 1)
    dM01 = moments['m01']
    dM10 = moments['m10']
    S = moments['m00'] #area in pixels
    w = math.sqrt(S*4/3.1415) + 0.0000001 #width in pixels or diameter
    W = 6
    f = 884 #focal lenght = 844
    d = f*W/w

    if S > 500:
        x = int(dM10 / S)
        y = int(dM01 / S)
        cv2.circle(img, (x, y), 5, color_yellow, 2)
        cv2.putText(img, "%d" % d, (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)



        if (x<600 and movingR == False):
            ser.write(b"R")
            ser.write(b"40")
            print("R")
            print("40")
            movingR = True
            movingX = False
            print(movingR)


        if (x>680 and movingR == False):
            ser.write(b"R")
            ser.write(b"-40")
            print("R")
            print("-40")
            movingR = True
            movingX = False


        if (x<=680 and x>=600 and movingX == False):
            movingX = True
            if (movingR == True):
                ser.write(b"S")
                ser.write(b"0")
                print("S")
                print("0")
                movingR = False
            ser.write(b"X")
            ser.write(b"60")
            print("X")
            print("60")


        if (d < 50):
            ser.write(b"S")
            ser.write(b"0")
            print("S")
            print("0")




    cv2.line(img, (640, 320), (640, 400), (0, 0, 255), 5)
    cv2.line(img, (600, 360), (680, 360), (0, 0, 255), 5)

    cv2.imshow('result', img)

    ch = cv2.waitKey(5)
    if ch == 27:
        break

cap.release()
cv2.destroyAllWindows()