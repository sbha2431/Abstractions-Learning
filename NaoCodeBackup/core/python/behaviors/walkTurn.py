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
            commands.setWalkVelocity(0, 0, 0.1)

    class ForwardTurn(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0.3)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

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
	    fwdturn = self.ForwardTurn()
        off = self.Off()
        self.trans(stand, C, turn, T(5.0), sit, C, off)

