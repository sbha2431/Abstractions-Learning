"""Simple keeper behavior."""

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
        pose.sit()
        commands.setStiffness(cfgstiff.Zero)
        # pose.BlockCenter()
        UTdebug.log(15, "Blocking center")

class ToPose(Task):
  def __init__(self, pose, time = 0.50, reverse=False):
    Task.__init__(self)
    self.pose = pose
    self.time = time
    self.reverse = reverse
  
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
    # ToPose(pose = pose, time = time).reset()

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


class BlockLeft(Task):
  def run(self):
  # super(BlockLeft, self).__init__(time=0.1)
    print(self.getTime())
    self.setSubtask(PoseSequence(
      cfgpose.blockleft, 0.5,
      cfgpose.sittingPoseNoArms, 1.0
  ))
    print(self.getTime())
    # while self.getTime()<5.0:
    #   print(self.getTime())



class BlockRight(Task):
  def run(self):
    # super(BlockRight, self).__init__(time=0.1)
    print(self.getTime())
    self.setSubtask(PoseSequence(
      cfgpose.blockright, 0.5,
      cfgpose.sittingPoseNoArms, 1.0
    ))
    print(self.getTime())
    # while self.getTime()<5.0:
    #   print(self.getTime())
    #   pass

class BlockCenter(Task):
  def run(self):
    # super(BlockCenter, self).__init__(time=0.1)
    print(self.getTime())
    self.setSubtask(PoseSequence(
      cfgpose.blockcenter, 1.5,
      cfgpose.sittingPoseNoArms, 1.0
    ))
    print(self.getTime())
    # while self.getTime()<5.0:
    #   print(self.getTime())
    #   pass

class BallSearch(Node):
    yawFlag = True
    turnFlag = False

    def run(self):
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      maxYaw = 1.0
      minYaw = -1.0

      # print(core.joint_values[core.HeadYaw])
      
      if ball.seen:
          print("Elevation ",ball.visionElevation,", Bearing ",ball.visionBearing,", Distance ",ball.visionDistance)
          # commands.setHeadPan(0,0.2)
          # return ball.visionElevation, ball.visionBearing,ball.visionDistance
      elif not ball.seen and core.joint_values[core.HeadYaw] <=maxYaw and self.yawFlag and not self.turnFlag:
          #print("turn+")
          commands.setHeadPan(core.joint_values[core.HeadYaw]+0.07,0.01)
          if core.joint_values[core.HeadYaw]+0.05 >= maxYaw:
              self.yawFlag=False
      elif not ball.seen and core.joint_values[core.HeadYaw] >=minYaw and not self.yawFlag and not self.turnFlag:
          #print("turn-")
          commands.setHeadPan(core.joint_values[core.HeadYaw]-0.07,0.01)
          if core.joint_values[core.HeadYaw]-0.05 <= minYaw:
              self.yawFlag=True
              self.turnFlag=True
      elif not ball.seen and self.turnFlag:
          commands.setHeadPan(0,0.1)
          commands.setWalkVelocity(0, 0, 0.4)
          self.turnFlag = False



class ApproachToLine(Node):
  # Right kick
  x_pos = 184
  y_pos = 190
  theta_pos = 0
  # target = np.array([x_pos,y_pos,-theta_pos])
  err_x_run = 0
  err_y_run = 0
  err_t_run = 0
  err_x = 0
  err_y = 0
  err_t = 0
  # err_run = np.zeros((3,))
  # err = np.zeros((3,))

  def run(self):
      # Note x is pixel (left-right) and y is (up-down)
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      line = memory.world_objects.getObjPtr(core.WO_OWN_PENALTY)
      dt = 0.01
      # Gain values
      # K_P = np.diag([1e-2,1e-3,1e-3])
      # K_I = np.diag([1e-3,1e-3,1e-3])
      # K_D = np.eye(3)*1e-6
      K_Px = 2e-3
      K_Py = 2e-3
      K_Pt = 1.0
      K_Ix = 5e-3
      K_Iy = 1e-4
      K_It = 1e-2
      K_Dx = 0.0
      K_Dy = 0.0
      K_Dt = 0.0

      # Saturation block - low since we are close
      max_speed = 0.5
      # pos = np.array([ball.imageCenterX,ball.imageCenterY,-goal.visionBearing])
      # print("Target shape", self.target.shape," pos shape ", pos.shape)
      # err2 = self.target-pos
      # print(err2.shape)
      err_x2 = self.x_pos-ball.imageCenterX
      err_y2 = self.y_pos-ball.imageCenterY
      err_t2 = ball.visionBearing-self.theta_pos
      dist_error = (float(err_x2)**2+float(err_y2)**2)**0.5
      # dist_error2 = np.linalg.norm(err[0:2])
      self.err_x_run += err_x2*dt
      self.err_y_run += err_y2*dt
      self.err_t_run += err_t2*dt
      # print(self.err_run.shape)
      # self.err_run += err2*dt
      err_x_delta = (err_x2-self.err_x)/dt
      err_y_delta = (err_y2-self.err_y)/dt
      err_t_delta = (err_t2-self.err_t)/dt
      # err_delta = (err2-self.err)/dt
      # self.err = err2
      self.err_x = err_x2
      self.err_y = err_y2
      self.err_t = err_t2
      # out_com = np.matmul(K_P,self.err.reshape((3,1)))+np.matmul(K_I,self.err_run.reshape(3,1)) + np.matmul(K_D,err_delta.reshape(3,1))
      w_dot = K_Px*self.err_x+K_Ix*self.err_x_run+K_Dx*err_x_delta
      h_dot = K_Py*self.err_y+K_Iy*self.err_y_run+K_Dy*err_y_delta
      t_dot = K_Pt*self.err_t+K_It*self.err_t_run+K_Dt*err_t_delta
      command_out = (w_dot**2+h_dot**2)**0.5
      # command_out2 = np.linalg.norm(out_com)
      # print("Output h: ",h_dot,", Output w: ",w_dot,"Output t",t_dot)
      # print("Output2 h: ",out_com[1],", Output2 w: ",out_com[0],"Output2 t",out_com[2])
      
      commands.setHeadPan(0,0.1)
      
      # commands.setWalkVelocity(0,0,max(min(t_dot,max_speed),-max_speed))
      # print("Distance error : ", dist_error,"ball angle error: ",err_t2,"   Command out: ",command_out)
      # print("Distance error : ", dist_error)
      # print('line distance ', line.visionDistance)

      if line.visionDistance < 120 and not (dist_error<= 6 and err_t2<=0.15 ):
        commands.setWalkVelocity(0,0,0)
        print("Stopping because at line!")
      
      elif (dist_error<= 6 and err_t2<=0.15 ):
          # print("Ready to save. Ball at: ",ball.imageCenterX,", ",ball.imageCenterY)
          commands.setWalkVelocity(0,0,0)
          self.finish()
      else:
        # print('x vel', w_dot)
        commands.setWalkVelocity(max(min(w_dot,max_speed),-max_speed),0,0)



class Rotate(Node):

  # Right kick
  theta_pos = 0
  # target = np.array([x_pos,y_pos,-theta_pos])
  err_t_run = 0
  err_t = 0
  target_dist = 80
  # err_run = np.zeros((3,))
  # err = np.zeros((3,))
  currt = 0
  err_x_run = 0
  err_x = 0

  def run(self):
    line = memory.world_objects.getObjPtr(core.WO_OWN_PENALTY)
    robot= memory.world_objects.getObjPtr(5)
    err_x2 = line.visionDistance - self.target_dist
    # Note x is pixel (left-right) and y is (up-down)
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    dt = 1e-2
    # Gain values
    # K_P = np.diag([1e-2,1e-3,1e-3])
    # K_I = np.diag([1e-3,1e-3,1e-3])
    # K_D = np.eye(3)*1e-6
    K_Px = 2e-3
    K_Py = 2e-3
    K_Pt = 0.0
    K_Ix = 2e-3
    K_Iy = 1e-3
    K_It = 0.0
    K_Dx = 0.0
    K_Dy = 1e-6
    K_Dt = 0.0
        # Saturation block - low since we are close

    self.err_x_run += err_x2*dt
    # print(self.err_run.shape)
    # self.err_run += err2*dt
    err_x_delta = (err_x2-self.err_x)/dt
    # err_delta = (err2-self.err)/dt
    # self.err = err2
    self.err_x = err_x2
    h_dot = K_Px*self.err_x+K_Ix*self.err_x_run+K_Dx*err_x_delta
    max_speed = 0.4
    self.currt = self.getTime()

    err_t2 = ball.bearing-self.theta_pos
    self.err_t_run += err_t2*dt

    err_t_delta = (err_t2-self.err_t)/dt

    self.err_t = err_t2

    w_dot = K_Py*self.err_t+K_Iy*self.err_t_run
    truew_dot = math.copysign(0.4,err_t2)
    truea_dot = math.copysign(0.09,err_t2)
    command_out = (truew_dot**2)**0.5
    # command_out2 = np.linalg.norm(out_com)
    # print(Output w: ",w_dot")
    # print("Output2 h: ",out_com[1],", Output2 w: ",out_com[0],"Output2 t",out_com[2])
    
    commands.setHeadPan(0,0.1)
    commands.setWalkVelocity(0,truew_dot,truea_dot)
    # commands.setWalkVelocity(0,0,max(min(t_dot,max_speed),-max_speed))
    # print("Ball angle error: ",err_t2,"   Command out: ",command_out)
    # print("Distance error : ", dist_error,"   &  ",dist_error2)
    if abs(err_t2)<=0.04:
        # print("Ready to save at: ball at angle ",ball.bearing)
        commands.setWalkVelocity(0,0,0)
        # self.finish()

class StopStill(Task):
  def run(self):
    commands.setWalkVelocity(0,0,0)

class Defense(Node):
  t = 0
  robot = memory.world_objects.getObjPtr(5)
  line = memory.world_objects.getObjPtr(core.WO_OWN_PENALTY)
  lineflag = False
  rotateflag = False
  hld_bdrin = 0
  def run(self):
    global robotlocx, robotlocy
    robotlocx=0.5*robotlocx+0.5*self.robot.loc.x
    robotlocy=0.5*robotlocy+0.5*self.robot.loc.y
    # commands.setHeadTilt(tilt_angle)
    commands.stand()  
    ball = mem_objects.world_objects[core.WO_BALL]
    commands.stand()
    # print('x ', ball.px,' y ', ball.py)
    if self.hld_bdrin == 6:
      self.hld_bdrin = 6
    else:
      self.hld_bdrin = ball.dirn
    print("Balldir ",self.hld_bdrin)
    if not self.lineflag:
      # commands.setHeadPan(ball.bearing, 0.1)
      print('Moving to line')
      # print('Line distance: ', line.visionDistance )
      self.hld_bdrin = 5
      self.postSignal("LineUp")
      if self.line.visionDistance < 120:
        self.lineflag = True
    elif self.lineflag and not ball.seen and self.hld_bdrin >= 5:
      self.postSignal("ball search")
    elif self.lineflag and not self.rotateflag and ball.seen and self.hld_bdrin >= 5:
      self.hld_bdrin = 5
      self.postSignal("rotate")
      if self.line.visionDistance > 200:
        self.lineflag = False
      if abs(ball.visionBearing) > 0.04:
        self.rotateflag = False
    elif self.hld_bdrin == 0 and self.lineflag:
      self.hld_bdrin = 6
      print('Blocking left')
      self.postSignal("left")
    elif self.hld_bdrin == 2 and self.lineflag:
      print('Blocking center')
      self.postSignal("center")
      self.hld_bdrin = 6
    elif self.hld_bdrin == 1 and self.lineflag:
      print('Blocking right')
      self.postSignal("right")
      self.hld_bdrin = 6
                

class Set(StateMachine):
  def setup(self):
    global disp
    disp = [[0,0],0]
    print("resetting odom")
  

class Playing(LoopingStateMachine):
  def setup(self):
      defense = Defense()
      blocks = {"LineUp":ApproachToLine(),
                "stop":StopStill(),
                "rotate": Rotate(),
                "ball search": BallSearch(),
                "left":BlockLeft(),
                "right":BlockRight(),
                "center":BlockCenter()
                }
      savelist = ["left","right","center"]
      # sit = reinit()
      # self.trans(self.Stand(),C)
      for name in blocks:
          b = blocks[name]
          print("Block module: ",b)
          if name in savelist:
            self.add_transition(defense, S(name), b, T(1.7), defense)            
          else:
            self.add_transition(defense, S(name), b, T(0.5), defense)
