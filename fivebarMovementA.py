

import RPi.GPIO as gpio #type: ignore
from gpiozero import Button #type: ignore
from time import sleep
arm4Length = .105
direction_pin = 20
pulse_pin = 21
cw_direction = 0
ccw_direction = 1

ldp = 20
lsp = 21

rdp = 15
rsp = 14
#leftLimit = Button(16, bounce_time = .01)
#rightLimit = Button(17, bounce_time = .01)
rightPos = 90
leftPos = 90
rlimSet = False
llimSet = False

gpio.setmode(gpio.BCM)
gpio.setup(ldp, gpio.OUT)
gpio.setup(lsp, gpio.OUT)
gpio.setup(rdp, gpio.OUT)
gpio.setup(rsp, gpio.OUT)
def left_pressed():
    leftPos = 45
    llimSet = True
def right_pressed():
    rightPos = 135
    rlimSet = True
#leftLimit.when_pressed = left_pressed
#rightLimit.when_pressed = right_pressed

def checkLimits():
    global rlimSet, llimSet
    while not(rlimSet and llimSet):
        moveBoth(1, False, 1, False)
        #Left
        moveBoth(1, True, 1, True)

def moveBoth(ls, ld, rs, rd):
    high = 0
    low = 1000000000
    
    if(ls < rs):
        high = int(rs)
        low = ls
    else :
        high = int(ls)
        low = rs
    
    for x in range(high+1):
        if(ld == True):
            gpio.output(ldp, gpio.LOW)
        else :
            gpio.output(ldp, gpio.HIGH)
            
        if(rd == True):
            gpio.output(rdp, gpio.LOW)
        else:
            gpio.output(rdp, gpio.HIGH)
            
        if(ls > x):
            gpio.output(lsp, gpio.HIGH)
            sleep(.001)
            gpio.output(lsp, gpio.LOW)
            sleep(.001)
            
        if(rs > x):
            gpio.output(rsp, gpio.HIGH)
            sleep(.001)
            gpio.output(rsp, gpio.LOW)
            sleep(.001)
def run(path):
    #checkLimits()
    lastl = leftPos
    lastr = rightPos
    for x in path:
        #print(x)
        movel = lastl- x[0]
        #print(movel)
        mover = lastr - x[1]
        print(movel)
        print(mover)
        
        stepsL = (movel * 1600)/90
        stepsR = (mover * 1600)/90
        ld = True
        rd = True
        if movel < 0:
            ld = False
        if mover > 0:
            rd = False
        moveBoth(abs(stepsL), ld, abs(stepsR), rd)
        sleep(.1)
        lastl = x[0]
        lastr = x[1]

