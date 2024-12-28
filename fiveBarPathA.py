
import math
import numpy as np
#Resolutoion set to 100
res = 10

def circle(r, x, y):
    x_left= x - r
    x_right = x + r
    global res
    print(f"Resolution set to {res} points")
    xRange = np.linspace(x_left, x_right, res)
    pointsTop = []
    pointsBottom = []
    for a in xRange:
        pointTop = []
        pointBottom = []
        pointy = math.sqrt(abs(pow(r, 2)- pow((a - x), 2)))+y
        top = pointy
        bottom = pointy - (2*(pointy - y))
        pointTop = [a, top]
        pointBottom = [a, bottom]
    
        #print(f"Top: {pointTop} Bottom: {pointBottom}")
        pointsTop.append(pointTop)
        pointsBottom.append(pointBottom)
    pointsBottom.reverse()
    finalOut = pointsTop + pointsBottom
    return finalOut


