#!/usr/bin/env python

#import IPython
#IPython.embed()

import numpy as np
import time

#from transforms3d import *
#from transforms3d.axangles import *
#from transforms3d.quaternions import *
from transforms3d.euler import *
import random
from os import *

from openravepy import *
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
#env.Load('/home/viki/openrave/src/robots/barrettwam.robot.xml') # load a simple scene
env.Load(getenv("HOME") + '/openrave/src/robots/barrettwam.robot.xml')
robot = env.GetRobots()[0] # get the first robot

from openravepy.misc import *
InitOpenRAVELogging()


PI = np.pi

manip=robot.GetManipulators()[0]

Coords = []

viewer = env. GetViewer()
CamTrafo = np.array([[ 0.96150224, -0.13236341,  0.24081814, -0.61274928],
    [-0.27203665, -0.33457765,  0.90224933, -2.18945813],
    [-0.03885243, -0.93302611, -0.35770485,  1.22028399],
    [ 0.        ,  0.        ,  0.        ,  1.        ]])
viewer.SetCamera(CamTrafo)
viewer.SendCommand('ShowWorldAxes 1')

	  
def setTarget( NumSteps=10, TimeSleep=0.1):
    """
    recorder = RaveCreateModule(env,'viewerrecorder')
    env.AddModule(recorder,'')
    codecs = recorder.SendCommand('GetCodecs') # linux only
    filename = 'openrave.mpg'
    codec = 13 # mpeg4
    recorder.SendCommand('Start 640 480 10 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))
    """
    j0L=robot.GetJoints()[0].GetLimits()[1]
    j1L=robot.GetJoints()[1].GetLimits()[1]
    for step in range(NumSteps):
        global manip
        #robotInit.SetDOFValues(np.deg2rad(joint0, joint1, joint2, joint3, joint4, joint5, joint6));
        #robotTarget.SetDOFValues([joint0, joint1, joint2, joint3, joint4, joint5, joint6]);
        quatJ = manip.CalculateRotationJacobian()
        #wManip = euler2quat(np.deg2rad(StepROLL), np.deg2rad(StepPITCH), np.deg2rad(StepYAW), 'sxyz')
        #quatManip0 = manip.GetTransformPose()[0:4]
	#quatManip1 = qmult(quatManip0, wManip)
	#j0W = np.sin(step/(NumSteps+1)*j0L)/1000.0
	#j1W = np.cos(step/(NumSteps+1)*j1L)/1000.0
	j0W = random.uniform(-j0L, j0L)/20.0
	j1W = random.uniform(-j1L, j1L)/20.0
	err = quatJ[0:4,0:2].dot([j0W, j1W]) # for erratic movement of the first two joints
        main5 = quatJ[0:4,2:7] # the remaining 5 joints compensating the erratic movement
        try:
            wSol=np.linalg.lstsq(main5, -err)
            #print "Full Sollution for np.linalg.lstsq(angJ, wManip), Step= " + str(step)
            #print wSol
            #print "Solution:"
            #print wSol[0]
            sol = wSol[0]
            #print "deltaDOFValues + robotTarget.GetDOFValues() : "
            deltaDOFValues = np.concatenate([j0W, j1W, sol[0], sol[1], sol[2], sol[3], sol[4]])
            #print  deltaDOFValues + manip.GetArmDOFValues()
            YPR = np.rad2deg(quat2euler(manip.GetTransformPose()[0:4], 'rzyx')) # EE orientation as Yaw Pitch Roll
            print("Yaw {} , Pitch {} , Roll {}".format(YPR[0],YPR[1],YPR[2]) )
            newDOFValues = deltaDOFValues + manip.GetArmDOFValues()
	    robot.SetDOFValues(np.array(newDOFValues), manip.GetArmIndices())
	    #global CoordInit
            coord = DrawAxes(env, manip.GetTransformPose(), 0.20)
            Coords.append(coord)
            time.sleep(TimeSleep)
	except  np.linalg.linalg.LinAlgError:
	    raise ValueError("Inversion for quaternion Jacobian does not converge.")  
	  
def reset(joint0=0.0, joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=0.0, joint6=0.0): # Manipulator's joints
    global robot, manip, Coords
    robot.SetDOFValues(np.deg2rad([joint0, joint1, joint2, joint3, joint4, joint5, joint6]), manip.GetArmIndices())
    del Coords[:]

    

def printRobot(robot):
    raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))

