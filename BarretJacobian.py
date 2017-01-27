#!/usr/bin/env python

#import IPython
#IPython.embed()

import numpy as np
import time

#from transforms3d import *
from transforms3d.axangles import *
from transforms3d.quaternions import *


from openravepy import *
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/viki/openrave/src/robots/barrettwam.robot.xml') # load a simple scene
robotInit = env.GetRobots()[0] # get the first robot

from openravepy.misc import *
InitOpenRAVELogging()


PI = np.pi

#for link in robotInit.GetLinks():
#   for geom in link.GetGeometries():
#            geom.SetTransparency(0.1)
robotTarget = RaveCreateRobot(env,robotInit.GetXMLId())
robotTarget.Clone(robotInit,0) 
env.AddRobot(robotTarget,True)
manip=robotTarget.GetManipulators()[0]
CoordTarget = None
CoordInit=DrawAxes(env,robotInit.GetManipulators()[0].GetTransformPose(),0.1)
viewer = env. GetViewer()
CamTrafo = np.array([[ 0.96150224, -0.13236341,  0.24081814, -0.61274928],
    [-0.27203665, -0.33457765,  0.90224933, -2.18945813],
    [-0.03885243, -0.93302611, -0.35770485,  1.22028399],
    [ 0.        ,  0.        ,  0.        ,  1.        ]])
viewer.SetCamera(CamTrafo)
viewer.SendCommand('ShowWorldAxes 1')
for link in robotTarget.GetLinks():
    for geom in link.GetGeometries():
        geom.SetTransparency(0.001)

	  
def setTarget(StepYAW=0.5, StepPITCH=0.5, StepROLL=0.5, joint0=0, joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, joint6=0, NumSteps=10, TimeSleep=0.1): 
    # all arguements in Degrees

    for step in range(NumSteps):
        global manip
        angJ = manip.CalculateAngularVelocityJacobian()
        wManip = [PI*StepROLL/180.0,  PI*StepPITCH/180.0, PI*StepYAW/180.0]
        try:
            wSol=np.linalg.lstsq(angJ, wManip)
            print "Full Sollution for np.linalg.lstsq(angJ, wManip), Step= " + str(step)
            print wSol
            #print "Solution:"
            #print wSol[0]
            deltaDOFValues = wSol[0]
            #print "deltaDOFValues + robotTarget.GetDOFValues() : "
            global robotTarget
            print  deltaDOFValues + manip.GetArmDOFValues()
            newDOFValues = deltaDOFValues + manip.GetArmDOFValues()
	    robotTarget.SetDOFValues(np.array(newDOFValues), manip.GetArmIndices())
	    global CoordTarget
            CoordTarget = DrawAxes(env, manip.GetTransformPose(),0.1)
            time.sleep(TimeSleep)
	except  np.linalg.linalg.LinAlgError:
	    raise ValueError("Inversion for angular Jacobian does not converge.")  
	  
def reset(joint0=0, joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, joint6=0): # Manipulator's joints
    global robotInit,robotTarget, manip, CoordInit, CoordTarget
    robotInit.SetDOFValues([joint0, joint1, joint2, joint3, joint4, joint5, joint6], manip.GetArmIndices())
    CoordInit=DrawAxes(env,robotInit.GetManipulators()[0].GetTransformPose(),0.1)
    robotTarget.SetDOFValues([joint0, joint1, joint2, joint3, joint4, joint5, joint6], manip.GetArmIndices())
    CoordTarget=DrawAxes(env, robotTarget.GetManipulators()[0].GetTransformPose(),0.1)
    

def printRobot(robot):
    raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))

