
from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import math
import core
import memory
import commands
import mem_objects
import cfgpose
import util
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles
from state_machine import Node, S,C, T, LoopingStateMachine, StateMachine
from task import Task, MultiTask
import task
import UTdebug
import head
import cfgstiff
from pose import PoseSequence
global tilt_angle
tilt_angle = -0.1
global disp
disp = [[0,0],0]

global robotlocx,robotlocy
robotlocx = 0
robotlocy = 0


class reinit(Node):
    def run(self):
        print("dafuq")
        pose.sit()
        commands.setStiffness()
        # pose.BlockCenter()
        UTdebug.log(15, "Blocking center")

class ToPose(Task):
  def __init__(self, pose, time = 0.50, reverse=False):
    Task.__init__(self)
    self.pose = pose
    self.time = time
    self.reverse = reverse
    walk_request.noWalk()
    kick_request.setNoKick()
  
  def reset(self):
    super(ToPose, self).reset()
    self.first = True

  def run(self):
    if self.first:
      for i in range(2, core.NUM_JOINTS):
        val = util.getPoseJoint(i, self.pose, self.reverse)
        if val != None:
          joint_commands.setJointCommand(i, val * core.DEG_T_RAD)

      joint_commands.send_body_angles_ = True
      joint_commands.body_angle_time_ = self.time * 1000.0
      walk_request.noWalk()
      kick_request.setNoKick()
      self.first = False

    if self.getTime() > self.time:
      print("done")
      self.finish()

  @staticmethod
  def ToPoseTimes(poses, times):
    posetimes = []
    for i in range(len(poses)):
      posetimes.append(poses[i])
      posetimes.append(times[i])
    return posetimes

class PoseSequence(Task):
  def __init__(self, *args):
    super(PoseSequence, self).__init__()
    if len(args) % 2 != 0:
      raise Exception("Pose sequence arguments must be (pose, time) pairs.")
    pi, ti = 0, 1
    chain = []
    while ti < len(args):
      pose = args[pi]
      time = args[ti]
      chain += [ToPose(pose = pose, time = time)]
      pi += 2
      ti += 2
    self.setChain(chain)
    ToPose(pose = pose, time = time).reset()

  def start(self):
    commands.setStiffness()
    super(PoseSequence, self).start()

  @staticmethod
  def ToPoseTimes(poses, times):
    posetimes = []
    for i in range(len(poses)):
      posetimes.append(poses[i])
      posetimes.append(times[i])
    return posetimes

class Kick(Node):
    def run(self):
        if self.getFrames() <= 3:
            memory.walk_request.noWalk()
            memory.kick_request.setFwdKick()
        if self.getFrames() > 10 and not memory.kick_request.kick_running_:
            commands.stand()
            self.finish()

class Defense(Node):
  def run(self):
    commands.stand()
    self.postSignal("kick")

class Playing(LoopingStateMachine):
  def setup(self):
      defense = Defense()
      blocks = {"kick":Kick()
                }
      savelist = ["left","right","center"]
      # sit = reinit()
      # self.trans(self.Stand(),C)
      for name in blocks:
          b = blocks[name]
          print("Block module: ",b)
          if name in savelist:
            self.add_transition(defense, S(name), b, T(2.5), defense)            
          else:
            self.add_transition(defense, S(name), b, T(10.5), defense)