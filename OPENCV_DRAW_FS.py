# Capture Video from Camera
import numpy as np
import cv2

drawing = False
click = 0
X1 = 0
Y1 = 0
X2 = 0
Y2 = 0
X3 = 0
point1 = ()
point2 = ()

#USE NUMBER OF CLICKS 

def mouse_drawing(event, x, y, flags, params):
    global point1, point2, drawing
    if event == cv2.EVENT_LBUTTONDOWN:
        if drawing is False:
            drawing = True
            X1 = x
            Y1 = y
        else:
            drawing = False
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing is True:
            X2 = x
            Y2 = y
    point1 = (X1, Y1)
    point2 = (X2, Y2)

window = "CamFeed"  # Naming the window where the camera feed will be displayed

cap = cv2.VideoCapture(0)  # Getting the camera feed from the Webcam
cv2.namedWindow(window, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.setMouseCallback(window, mouse_drawing)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(width)
print(height)

HSVFrame = []

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    cv2.imshow(window, frame)
    if cv2.waitkey
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
