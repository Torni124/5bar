import numpy as np
import fiveBarKineA
import fivebarMovementA
import fiveBarPathA
import RPi.GPIO as gpio #type: ignore

angles = []
try:
	points = fiveBarPathA.circle(.05, 0, .15)
	for x in points:
		aAng = fiveBarKineA.inverse_kinematics(x[0], x[1])
		angles.append(aAng)
	#print(angles)
	fivebarMovementA.run(angles)
except: KeyboardInterrupt
gpio.cleanup()
