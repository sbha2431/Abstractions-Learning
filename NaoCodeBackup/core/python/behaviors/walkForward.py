"""Blank behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from task import Task
import memory
import core
import pose
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, StateMachine



class Playing(StateMachine):
    def HeadTilter(self):
        
        commands.setHeadTilt(0.3)

    class Stand(Node):
        def run(self):
            robot= memory.world_objects.getObjPtr(5)

            print("robot ", robot.loc)
            print("orientation ",robot.orientation)
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()

    class Walk(Node):
        def run(self):
            # commands.setHeadTilt(0.3)
            commands.setWalkVelocity(0.0,0.0,-0.2)


    class WalkP(Node):
        def run(self):
            commands.setWalkPedantic(0.3, 0, 0)

    class TurnR(Node):
        def run(self):
            robot= memory.world_objects.getObjPtr(5)

            print("robot ", robot.loc)
            print("orientation ",robot.orientation)
            commands.setHeadTilt(0.3)

            commands.setWalkPedantic(0, 0, 0.01)

    class TurnL(Node):
        def run(self):
            robot= memory.world_objects.getObjPtr(5)

            print("robot ", robot.loc)
            print("orientation ",robot.orientation)
            commands.setHeadTilt(0.3)

            commands.setWalkPedantic(0, 0, -0.01)


    class ForwardTurn(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0.3)

    class WalkTarget(Node):
        def run(self):
            commands.setWalktoTarget(1000, 0, 0)
    class HeadTilt(Node):
        def run(self):
            commands.setHeadTilt(0.3)
    class HeadPanL(Node):
        def run(self):
            #print("We looking now")  
            self,HeadTilter()
            commands.setHeadPan(0.65,5)

    class HeadPanR(Node):
        def run(self):
            #print("We looking now")  
            self,HeadTilter()
            commands.setHeadPan(-0.65,5)

    class HeadPanZ(Node):
        def run(self):
            #print("We looking now") 
            commands.setHeadTilt(0.3)
   
            commands.setHeadPan(0,5)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()


    def setup(self):
        stand = self.Stand()
        walk = self.Walk()
        turnL = self.TurnL()
        turnR = self.TurnR()

        sit = pose.Sit()	
        fwdturn = self.ForwardTurn()
        targ = self.WalkTarget()
        ped = self.WalkP()
        panL = self.HeadPanL()
        panR = self.HeadPanR()
        panZ = self.HeadPanZ()

        off = self.Off()
        tilt= self.HeadTilt()
        # self.trans(stand, C, tilt, T(5.0),turnL,T(10.0),turnR,T(10.0) ,sit, C, off)

        self.trans(stand, C, walk, T(5.0),sit, C, off)
