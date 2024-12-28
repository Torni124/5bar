import cv2
import numpy as np

#contour_points contains wanted data. 
edge = False
#sensitivity thresholds
threshLow = 50
threshHigh = 150

cap = cv2.VideoCapture(0)
success, image = cap.read()
def processImg(img):
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray, (3, 3), 0)
    edges = cv2.Canny(image = img_blur, threshold1=threshLow, threshold2=threshHigh)
    return edges
if success:
    edges = processImg(image)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour_points = []
for contour in contours:
    # Simplify the contour slightly to reduce noise
    points = contour.reshape(-1, 2)
    contour_points.append(points)
height, width, _ = image.shape
black = np.zeros((height, width), dtype = np.uint8)
points = [point for contour in contour_points for point in contour]
for point in points:
    x, y = point
    black[y, x] = 255
print(len(contour_points[0][0]))
#return contour_points
cv2.imshow('Canny Edge Detection', black)
cv2.waitKey(0)
cap.release() 
cv2.destroyAllWindows()