"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
import cfgpose
import util
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles
from state_machine import Node, S,C, T, LoopingStateMachine
from task import Task, MultiTask
import task
import UTdebug
import head
import cfgstiff
from pose import PoseSequence

# class BlockLeft(Node):
#     def run(self):
#         print("Block left")
#         commands.setLShoulderRoll(1.25, 0.15)
#         # pose.BlockLeft()
#         UTdebug.log(15, "Blocking left")



# class BlockRight(Node):
#     def run(self):
#         commands.setRShoulderRoll(1.25, 0.15)
#         # pose.BlockRight()
#         UTdebug.log(15, "Blocking right")


# class BlockCenter(Node):
#     def run(self):
#         commands.setBShoulderPitch(-0.05, 0.25)
#         # pose.BlockCenter()
#         UTdebug.log(15, "Blocking center")

class reinit(Node):
    def run(self):
        pose.sit()
        commands.setStiffness(cfgstiff.Zero)
        # pose.BlockCenter()
        UTdebug.log(15, "Blocking center")





# class ToPose(Task):
#   def __init__(self, pose, time = 0.50, reverse=False):
#     Task.__init__(self)
#     self.pose = pose
#     self.time = time
#     self.reverse = reverse
  
#   def reset(self):
#     super(ToPose, self).reset()
#     self.first = True

#   def run(self):
#     if self.first:
#       for i in range(2, core.NUM_JOINTS):
#         val = util.getPoseJoint(i, self.pose, self.reverse)
#         if val != None:
#           joint_commands.setJointCommand(i, val * core.DEG_T_RAD)

#       joint_commands.send_body_angles_ = True
#       joint_commands.body_angle_time_ = self.time * 1000.0
#       walk_request.noWalk()
#       kick_request.setNoKick()
#       self.first = False

#     if self.getTime() > self.time:
#       self.finish()

#   @staticmethod
#   def ToPoseTimes(poses, times):
#     posetimes = []
#     for i in range(len(poses)):
#       posetimes.append(poses[i])
#       posetimes.append(times[i])
#     return posetimes

# class PoseSequence(Task):
#   def __init__(self, *args):
#     super(PoseSequence, self).__init__()
#     if len(args) % 2 != 0:
#       raise Exception("Pose sequence arguments must be (pose, time) pairs.")
#     pi, ti = 0, 1
#     chain = []
#     while ti < len(args):
#       pose = args[pi]
#       time = args[ti]
#       chain += [ToPose(pose = pose, time = time)]
#       pi += 2
#       ti += 2
#     self.setChain(chain)
#     ToPose.reset()

#   def start(self):
#     commands.setStiffness()
#     super(PoseSequence, self).start()

#   @staticmethod
#   def ToPoseTimes(poses, times):
#     posetimes = []
#     for i in range(len(poses)):
#       posetimes.append(poses[i])
#       posetimes.append(times[i])
#     return posetimes


# class BlockRight(Task):
#   def __init__(self, time = 1.0):
#     super(BlockRight, self).__init__(time=time)
#     self.setSubtask(PoseSequence(
#       cfgpose.blockright, 0.5,
#       cfgpose.blockright, self.time,
#       cfgpose.sittingPoseV3, 0.5
#     ))

class BlockLeft(Task):
  def __init__(self, time = 1.0):
    super(BlockLeft, self).__init__(time=time)
    self.setSubtask(PoseSequence(
      cfgpose.blockleft, 0.5,
      cfgpose.blockleft, self.time,
      cfgpose.sittingPoseV3, 0.5
    ))
  def run(self):
  # super(BlockLeft, self).__init__(time=0.1)
  self.setSubtask(PoseSequence(
    cfgpose.blockleft, 0.5,
    cfgpose.sittingPoseV3, 0.5
  ))


# class BlockCenter(Task):
#   def __init__(self, time = 1.0):
#     super(BlockCenter, self).__init__(time=time)
#     self.setSubtask(PoseSequence(
#       cfgpose.blockcenter, 0.5,
#       cfgpose.blockright, self.time,
#       cfgpose.sittingPoseV3, 0.5
#     ))
# class BlockRight(Task):
#   def run(self):
#     # super(BlockRight, self).__init__(time=0.1)
#     self.setSubtask(PoseSequence(
#       cfgpose.blockright, 0.5,
#       # cfgpose.sittingPoseV3, 0.5
#     ))

class BlockLeft(Task):

# class BlockCenter(Task):
#   def run(self):
#     # super(BlockCenter, self).__init__(time=0.1)
#     self.setSubtask(PoseSequence(
#       cfgpose.blockcenter, 0.5,
#       # cfgpose.sittingPoseV3, 0.5
#     ))

# class reinit(Task):
#   def run(self):
#     # super(BlockCenter, self).__init__(time=0.1)
#     self.setSubtask(PoseSequence(
#       cfgpose.sittingPoseV3, 0.5
#     ))



class BlockRight(Task):
  def __init__(self, time = 1.0):
    super(BlockRight, self).__init__(time=time)
    self.setSubtask(PoseSequence(
      cfgpose.keeperSave3,1.0,
      cfgpose.standingPose, 0.5
    ))

class BlockLeft(Task):
  def __init__(self, time = 1.0):
    super(BlockLeft, self).__init__(time=time)
    self.setSubtask(PoseSequence(
      cfgpose.keeperSave1, 1.0,
      cfgpose.standingPose, 0.5
    ))

class BlockCenter(Task):
  def __init__(self, time = 1.0):
    super(BlockCenter, self).__init__(time=time)
    self.setSubtask(PoseSequence(
      cfgpose.keeperSave2, 1.0,
      cfgpose.standingPose, 0.5
    ))


class Blocker(Node):


    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        # commands.setStiffness()
        # print('x ', ball.px,' y ', ball.py)
        if ball.seen:
            # print("updated position estimate: ",ball.px, " ", ball.py)
            # print("updated velocity estimate: ",ball.relVel.x, " ", ball.relVel.y)
            self.xhat = [ball.loc.x,ball.loc.y,ball.relVel.x,ball.relVel.y]
            commands.setHeadPan(ball.bearing, 0.15)
            commands.setStiffness()
            if ball.dirn < 3:
                UTdebug.log(15, "Ball is close, blocking!")
                if ball.dirn == 0:
                    print('Moving left arm')
                    choice = "left"
                elif ball.dirn == 2:
                    choice = "center"
                    print('Moving both arms')
                else:
                   choice = "right"
                   print('Moving right arm')

                self.postSignal(choice)

        # if ball.seen==False:
            #print("Ball not seen")
            # commands.setHeadPan(0, 1)
            # commands.setBShoulderPitch(-1.57, 0.5)
            # commands.setLShoulderPitch(-1.57, 1.0)
            # commands.setRShoulderRoll(0.02, 0.5)
            # commands.setLShoulderRoll(0.02, 0.5)

class Playing(LoopingStateMachine):
    
    def setup(self):
        blocker = Blocker()
        blocks = {"left": BlockLeft(),
                  "right": BlockRight(),
                  "center": BlockCenter(),
                  "sit": reinit()
                  }
        # sit = reinit()
        # self.trans(self.Stand(),C)
        for name in blocks:
            b = blocks[name]
            print("Block module: ",b)
            self.add_transition(blocker, S(name), b, T(2), blocker)