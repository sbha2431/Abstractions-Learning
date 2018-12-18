"""Follow Ball."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from task import Task
import memory
import core
import pose
import commands
import cfgstiff
from memory import localization_mem
import time
from task import Task
from state_machine import Node, C, T, StateMachine
import math
import numpy as np
x_move = 0
y_move = 0

class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()
    class HeadLeft(Node):
        def run(self):
            if core.joint_values[core.HeadYaw] <=1.2:

                commands.setHeadPan(core.joint_values[core.HeadYaw]+0.06,0.03)

            
    class Diagnostic(Node):
        #goalhold = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
        file = open('sensorout.txt','w')
        def run(self):
            commands.setHeadTilt(0.3)
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
            if ball.seen:
                print("Ball at ",ball.visionDistance," bearing ",ball.visionBearing,"  Camera: ",ball.fromTopCamera)
            goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
            WO_BEACON_BLUE_YELLOW = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW)
            WO_BEACON_YELLOW_BLUE = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE)
            WO_BEACON_BLUE_PINK = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
            WO_BEACON_PINK_BLUE = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE)
            WO_BEACON_PINK_YELLOW = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
            WO_BEACON_YELLOW_PINK = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)
            # print(WO_BEACON_PINK_BLUE.seen)
            # if WO_BEACON_BLUE_YELLOW.seen:
            #     print("WO_BEACON_BLUE_YELLOW seen ","Bearing: ",WO_BEACON_BLUE_YELLOW.visionBearing," Range: ",WO_BEACON_BLUE_YELLOW.visionDistance)
            #     self.file.write('{},{}\n'.format(WO_BEACON_BLUE_YELLOW.visionBearing,WO_BEACON_BLUE_YELLOW.visionDistance))
            # if WO_BEACON_YELLOW_BLUE.seen:
            #     print("WO_BEACON_YELLOW_BLUE seen ","Bearing: ",WO_BEACON_YELLOW_BLUE.visionBearing," Range: ",WO_BEACON_YELLOW_BLUE.visionDistance)
            #     self.file.write('{},{}\n'.format(WO_BEACON_YELLOW_BLUE.visionBearing,WO_BEACON_YELLOW_BLUE.visionDistance))
            # if WO_BEACON_BLUE_PINK.seen:
            #     print("WO_BEACON_BLUE_PINK seen ","Bearing: ",WO_BEACON_BLUE_PINK.visionBearing," Range: ",WO_BEACON_BLUE_PINK.visionDistance)
            #     self.file.write('{},{}\n'.format(WO_BEACON_BLUE_PINK.visionBearing,WO_BEACON_BLUE_PINK.visionDistance))
            # if WO_BEACON_PINK_BLUE.seen:
            #     print("WO_BEACON_PINK_BLUE seen ","Bearing: ",WO_BEACON_PINK_BLUE.visionBearing," Range: ",WO_BEACON_PINK_BLUE.visionDistance)
            #     self.file.write('{},{}\n'.format(WO_BEACON_PINK_BLUE.visionBearing,WO_BEACON_PINK_BLUE.visionDistance))
            # if WO_BEACON_PINK_YELLOW.seen:
            #     print("WO_BEACON_PINK_YELLOW seen ","Bearing: ",WO_BEACON_PINK_YELLOW.visionBearing," Range: ",WO_BEACON_PINK_YELLOW.visionDistance)
            #     self.file.write('{},{}\n'.format(WO_BEACON_PINK_YELLOW.visionBearing,WO_BEACON_PINK_YELLOW.visionDistance))
            # if WO_BEACON_YELLOW_PINK.seen:
            #     print("WO_BEACON_YELLOW_PINK seen ","Bearing: ",WO_BEACON_YELLOW_PINK.visionBearing," Range: ",WO_BEACON_YELLOW_PINK.visionDistance)
            #     self.file.write('{},{}\n'.format(WO_BEACON_YELLOW_PINK.visionBearing,WO_BEACON_YELLOW_PINK.visionDistance))
            if self.getTime() > 120.0:
                self.file.close()
                self.finish()

    class localization_diag(Node):
        robot= memory.world_objects.getObjPtr(5)
        def run(self):
            # print("localization player ",self.local)
            # print("localalization state ",self.localstate)
            commands.setHeadTilt(-0.0)
            # if core.joint_values[core.HeadYaw] <=1.0:
            #     commands.setHeadPan(core.joint_values[core.HeadYaw]+0.06,0.03)
            print("robot ", self.robot.loc)
            print("orientation ",self.robot.orientation)
            print("covariance ",self.robot.sd.getMagnitude())
            # print(localization_mem)
            if self.getTime() > 120.0:
                self.finish()



    def setup(self):
        stand = self.Stand()
        sit = pose.Sit()
        diag = self.Diagnostic()
        locdiag = self.localization_diag()
        panleft = self.HeadLeft()
        #self.trans(stand,C)

            #print("a")
        self.trans(stand,C,locdiag,T(120.0),sit)



            
            

            
                
