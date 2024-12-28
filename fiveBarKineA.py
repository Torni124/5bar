'''
TODO: Implement Limit Switches, calculate concavity
'''


from time import sleep
import math
import numpy as np
#import RPi.GPIO as gpio
#Units in Meters
base_distance = .05
#Left Base
arm1Length = .105
arm2Length = .105
arm3Length = .105
#Right Base
arm4Length = .105
direction_pin = 20
pulse_pin = 21
cw_direction = 0
ccw_direction = 1

bdp = 20
bsp = 21

adpl = 15
aspl = 14
#Implement Limit Switches
lStart = 90
rStart = 90
'''
gpio.setmode(gpio.BCM)
gpio.setup(bdp, gpio.OUT)

gpio.setup(bsp, gpio.OUT)
gpio.setup(adpl, gpio.OUT)
gpio.setup(aspl, gpio.OUT)
'''
#Formulas from 
# https://orenanderson.com/five-bar-robotics-kinematics-and-dynamics/
#Calculates the first of 4 solutions, should probably add other 3 and 
#Determine the best one. 
def inverse_kinematics(x1, y1):
    c = base_distance
    a1 = arm1Length
    a2 = arm2Length
    a3 = arm3Length
    a4 = arm4Length
    d12 = np.sqrt(((x1*x1) + (y1*y1)))
    d24 = np.sqrt(pow((x1-c), 2) + pow(y1, 2))
    alpha1 = math.acos((pow(d12, 2) + pow(a1, 2) - pow(a3,2))/(2*a1*d12))
    alpha2 = math.acos((pow(d24,2)+pow(a2, 2) - pow(a4, 2))/(2*a2*d24))
    q1 = math.atan2(y1, x1) - alpha1
    q2 = math.atan2(y1, x1-c) - alpha2
    q12 = math.atan2(y1, x1) - alpha1
    q22 = math.atan2(y1, x1-c) + alpha2
    q13 = math.atan2(y1, x1) + alpha1
    q23 = math.atan2(y1, x1-c) - alpha2
    q14 = math.atan2(y1, x1) + alpha2
    q24 = math.atan2(y1, x1-c) + alpha2
    qArr = np.array([[q1, q2], [q12, q22], [q13, q23], [q14, q24], [0,0]])
    aArr = np.degrees(qArr)
    fin1 = 90
    fin2 = 90
    for i in aArr:
         fin1 = i[0]
         fin2 = i[1]
         if( fin1 > 90 and fin2 < 90):
                print(1)
                break
                
           #elif(abs(q1-q2) < 100):
            #    break
         elif((fin1 - fin2) > 0):
                print(2)
                break
    resArr = [fin1, fin2]
    return resArr
'''
def moveBoth(bs, bd, sa, ad):
    high = 0
    low = 1000000000
    
    if(bs < sa):
        high = int(sa)
        low = bs
    else :
        high = int(bs)
        low = sa
    
    for x in range(high+1):
        if(bd == True):
            gpio.output(bdp, gpio.LOW)
        else :
            gpio.output(bdp, gpio.HIGH)
            
        if(ad == True):
            gpio.output(adpl, gpio.LOW)
        else:
            gpio.output(adpl, gpio.HIGH)
            
        if(bs > x):
            gpio.output(bsp, gpio.HIGH)
            sleep(.001)
            gpio.output(bsp, gpio.LOW)
            sleep(.001)
            
        if(sa > x):
            gpio.output(aspl, gpio.HIGH)
            sleep(.001)
            gpio.output(aspl, gpio.LOW)
            sleep(.001)
last_l = lStart
last_r = rStart
try:
     while True:

          accurate = True
          x_desired = float(input("X-Coord"))
          y_desired = float(input("Y-Coord"))
          sArr = inverse_kinematics(x_desired, y_desired)
          aArr = np.degrees(sArr)
          #for i in aArr:
          #print("Q1: " + str(i[0]) + " Q2: " + str(i[1]))
          #print("Q12: " + str(q12) + " Q22: " + str(q22))
          #print("Q13: " + str(q13) + " Q23: " + str(q23))
          #print("Q14: " + str(q14) + " Q24: " + str(q24))
          for i in aArr:
               q1 = i[0]
               q2 = i[1]
               if( q1 > 90 and q2 < 90):
                      print(1)
                      break
                      
                 #elif(abs(q1-q2) < 100):
                  #    break
               elif((q1 - q2) > 0):
                      print(2)
                      break
               if(q1 == 0 and q2 ==0):
                      print(3)
                      break

          if q1 > 200 or q1 < 15:
                 print(5)
                 accurate = False 
          if q2 > 165 or q2 < -10:
                 print(6)
                 accurate = False
            #Calculations to confirm concavity
          leftx = arm1Length*math.cos(180-q1)
          lefty = arm1Length*math.sin(180-q1)
          righty = arm4Length*math.sin(q2)
          rightx1 = arm4Length*math.cos(q2)
          rightx = rightx1 + .05
          slope = (righty - lefty)/(abs(rightx - leftx))
          line = slope * (x_desired - leftx) + lefty
         
          #xSample = np.linspace(1,10, 100)
          #ySample = slope * (xSample- leftx) + lefty
          #plt.figure(figsize= (10,6))
          #plt.plot(xSample, ySample, label=f'y = {slope}x + {lefty}')
          #plt.grid(True)
          #plt.show()
          print(leftx)
          print(lefty)
          print(rightx)
          print(righty)
          print(slope)

          print(line)
          print(y_desired)
          if(y_desired < line):
               print(7)
               accurate = False
          if(accurate):
               print("Final Coords:  Q1: " + str(q1) + " Q2: " + str(q2))
               move_l = last_l - q1
               move_r = last_r - q2
               stepsL = (move_l * 1600)/90
               stepsR = (move_r * 1600)/90
               ld = True
               rd = True
               if(move_l < 0):
                    ld = False
               if(move_r > 0):
                    rd = False
               print(move_l)
               print(move_r)
               #moveBoth(abs(stepsL), ld, abs(stepsR), rd)
               #x, y = dynamics.forward_kinematics(solution[0], solution[1])
               #print(f"End-effector position for q1 = {math.degrees(solution[0]):.2f}, q2 = {math.degrees(solution[1]):.2f}:")
               #print(f"x = {x:.4f}, y = {y:.4f}") 
               last_l = q1
               last_r = q2
               #print("Last_L" + str(last_l))
               #print("Last_R" + str(last_r))
          else:
               print("Unreachable Position")
except KeyboardInterrupt:
    print("exiting")
#finally:
    #gpio.cleanup()
'''
