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
from task import Task
from state_machine import Node, C, T, StateMachine

x_move = 0
y_move = 0

class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()
    
    class On(Node):
        def run(self):
            commands.setStiffness()
	    self.finish()

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()
    class TiltHeadUp(Node):
	def run(self):
	    commands.setHeadTilt(core.joint_values[core.HeadPitch]+0.5)

    class TiltHeadDown(Node):
	def run(self):
	    commands.setHeadTilt(core.joint_values[core.HeadPitch]-0.5)


    class BallSeen(Node):
        def run(self):
            ball = memory.world_objects.getObjPtr(core.WO_BALL)
    	    if ball.seen:
                print("I see ball")
                #print("Elevation ",ball.visionElevation,", Bearing ",ball.visionBearing,", Distance ",ball.visionDistance)
                x_ball = ball.imageCenterX
                y_ball = ball.imageCenterY
                print(x_ball)
                print(core.joint_values[core.HeadYaw])
                if x_ball > 150:
                    commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05,0.05)
                elif x_ball < 90:
                    commands.setHeadPan(core.joint_values[core.HeadYaw]+0.05,0.05)
                else:
                    commands.setHeadPan(core.joint_values[core.HeadYaw],0.01)

    ##		if y_ball < 360:
    ##		    commands.setHeadTilt(core.joint_values[core.HeadPitch]-0.02)
    ##		elif y_ball > 600:
    ##		    commands.setHeadPanTilt(core.joint_values[core.HeadYaw],core.joint_values[core.HeadPitch]+0.05,0.01)
    ##		elif y_ball>=360 and y_ball<=600:
    ##	            commands.setHeadPanTilt(core.joint_values[core.HeadYaw],core.joint_values[core.HeadPitch]-0.05,0.01)
    		    
    	    else:
                print("No ball seen")


    class GoalSeen(Node):
        def run(self):
            goal = memory.world_objects.getObjPtr(core.WO_OWN_GOAL)
            if goal.seen:
                #print("Elevation ",ball.visionElevation,", Bearing ",ball.visionBearing,", Distance ",ball.visionDistance)
                x_goal = goal.imageCenterX
                y_goal = goal.imageCenterY
                if x_goal > 780:
                    commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05,0.01)
                elif x_goal < 400:
                    commands.setHeadPan(core.joint_values[core.HeadYaw]+0.05,0.01)
    ##      if y_ball < 360:
    ##          commands.setHeadTilt(core.joint_values[core.HeadPitch]-0.02)
    ##      elif y_ball > 600:
    ##          commands.setHeadPanTilt(core.joint_values[core.HeadYaw],core.joint_values[core.HeadPitch]+0.05,0.01)
    ##      elif y_ball>=360 and y_ball<=600:
    ##              commands.setHeadPanTilt(core.joint_values[core.HeadYaw],core.joint_values[core.HeadPitch]-0.05,0.01)
                
            else:
                print("No goal seen")


    def setup(self):
        stand = self.Stand()
        sit = pose.Sit()
        Goalseen = self.GoalSeen()
        Ballseen = self.BallSeen()
        off = self.Off()
        self.trans(stand, C, Ballseen, T(20.0), sit, C, off)

