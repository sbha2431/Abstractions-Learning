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
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()

    class Walk(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0)

    class Turn(Node):
        def run(self):
            commands.setWalkVelocity(0, 0, 0.5)

    class ForwardTurn(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0.3)

    class HeadPanL(Node):
	def run(self):
	    commands.setHeadPan(0.65,1)

    class HeadPanR(Node):
	def run(self):
	    commands.setHeadPan(-0.65,1)

    class HeadPanZ(Node):
	def run(self):
	    commands.setHeadPan(0,1)
	    self.finish()
    class TiltHeadUp(Node):
	def run(self):
	    commands.setHeadTilt(0.4)

    class TiltHeadDown(Node):
	def run(self):
	    commands.setHeadTilt(-0.1)


    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            self.finish()
            #if self.getTime() > 2.0:
            #    memory.speech.say("turned off stiffness")
            #    self.finish()

    class Seen(Node):
        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
	    #print(ball.seen)
	    if ball.seen:
		print("I see ballz")

    def setup(self):
        stand = self.Stand()
        walk = self.Walk()
	turn = self.Turn()
        sit = pose.Sit()
	seen = self.Seen()
        panleft = self.HeadPanL()
	panright = self.HeadPanR()
	panstraight = self.HeadPanZ()
	tiltup = self.TiltHeadUp()
	tiltdown = self.TiltHeadDown()	
	fwdturn = self.ForwardTurn()
        off = self.Off()
        self.trans(stand, C, panleft,T(2.0),panright,T(2.0),panstraight,T(2.0),tiltup,T(2.0),tiltdown,T(2.0),sit,C, off)

