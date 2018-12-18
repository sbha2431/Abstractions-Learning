"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import math
import commands
import mem_objects
import cfgpose
import util
import memory
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles
from state_machine import Node, S,C, T, LoopingStateMachine
from task import Task, MultiTask
import task
import UTdebug
import head
import cfgstiff
from pose import PoseSequence

global xy_threshold
global center_threshold
global tilt_angle
point_x = 0
point_y = 0
point_t = 0
tilt_angle = -0.05
xy_threshold = 250
center_threshold = 100
global t_threshold
t_threshold = 0.5

class Stand(Node):
    def run(self):
        commands.stand()
        if self.getTime() > 5.0:
            memory.speech.say("Starting localization")
            self.finish()

        

class HeadLeft(Node):
    def run(self):
        commands.setHeadPan(core.joint_values[core.HeadYaw]+0.06,0.03)
        # commands.setHeadTilt(tilt_angle)
        # print("Block left")
        # commands.setLShoulderRoll(1.25, 0.15)
        # pose.BlockLeft()
        # UTdebug.log(15, "Blocking left")



class HeadRight(Node):
    def run(self):
        commands.setHeadPan(core.joint_values[core.HeadYaw]-0.06,0.03)
        # commands.setHeadTilt(tilt_angle)
        # print("Block left")
        # commands.setLShoulderRoll(1.25, 0.15)
        # pose.BlockLeft()
        # UTdebug.log(15, "Blocking left")

class HeadStill(Node):
    def run(self):
        # commands.setHeadTilt(tilt_angle)
        # commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05,0.01)
        pass


class MoveCenter(Node):
    goal_x = 0
    goal_y = 0
    goal_theta = [-2.0,-1.0,1.0,2.0] # Ignore for now
    err_x_run = 0
    err_y_run = 0
    err_t_run = 0
    err_x = 0
    err_y = 0
    err_t = 0
    yawFlag = True

    def run(self):
        print("moving to center ")
        robot= memory.world_objects.getObjPtr(5)
        # commands.setHeadTilt(tilt_angle)
        dt = 1e-2

        K_Px = 5e-4
        K_Py = 5e-4
        K_Pt= 1
        K_Ix = 5e-5
        K_Iy = 5e-5
        K_It= 1e-3
        K_Dx = 1e-6
        K_Dy = 1e-6
        K_Dt= 0.0
        max_speed = 0.5

        err_x2 = self.goal_x - robot.loc.x
        err_y2 = self.goal_y - robot.loc.y
        err_list = [i-robot.orientation for i in self.goal_theta]
        abs_list = [abs(sk) for sk in err_list]
        err_t2 = err_list[abs_list.index(min(abs_list))]
        
        dist_error = (float(err_x2)**2+float(err_y2)**2)**0.5
        self.err_x_run += err_x2*dt
        self.err_y_run += err_y2*dt
        self.err_t_run += err_t2*dt
        err_x_delta = (err_x2-self.err_x)/dt
        err_y_delta = (err_y2-self.err_y)/dt
        err_t_delta = (err_t2-self.err_t)/dt
        self.err_x = err_x2
        self.err_y = err_y2
        self.err_t = err_t2
        # print("x_err: ",  self.err_x, " y_err: ",self.err_y)

        f_dot = (K_Px*self.err_x+K_Ix*self.err_x_run+K_Dx*err_x_delta)*math.cos(robot.orientation) + (K_Py*self.err_y+K_Iy*self.err_y_run+K_Dy*err_y_delta)*math.sin(robot.orientation) # Forward
        h_dot = (K_Px*self.err_x+K_Ix*self.err_x_run+K_Dx*err_x_delta)*math.sin(-robot.orientation) + (K_Py*self.err_y+K_Iy*self.err_y_run+K_Dy*err_y_delta)*math.cos(robot.orientation) # Horiztontal
        # print("h_vel: ", h_dot, " f_vel: ",f_dot)
        a_dot =  K_Pt*self.err_t+K_It*self.err_t_run+K_Dt*err_t_delta# Angle rate
        commands.setWalkVelocity(max(min(f_dot,max_speed),-max_speed),max(min(h_dot,max_speed),-max_speed),max(min(a_dot,max_speed),-max_speed))
        # if core.joint_values[core.HeadYaw] <=0.75 and self.yawFlag:
        #     commands.setHeadPan(core.joint_values[core.HeadYaw]+0.05,0.08)
        # elif core.joint_values[core.HeadYaw] >= -0.75:
        #     self.yawFlag = False
        #     commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05,0.08)
        


class MoveToPoint(Node):
    global point_x,point_y,point_t
    goal_x = point_x
    goal_y = point_y
    goal_theta = [point_t] # Ignore for now
    err_x_run = 0
    err_y_run = 0
    err_t_run = 0
    err_x = 0
    err_y = 0
    err_t = 0
    yawFlag = True

    def run(self):
        robot= memory.world_objects.getObjPtr(5)
        # commands.setHeadTilt(tilt_angle)
        dt = 1e-2

        K_Px = 5e-4
        K_Py = 5e-4
        K_Pt= 1
        K_Ix = 5e-5
        K_Iy = 5e-5
        K_It= 1e-3
        K_Dx = 1e-6
        K_Dy = 1e-6
        K_Dt= 0.0
        max_speed = 0.5

        err_x2 = self.goal_x - robot.loc.x
        err_y2 = self.goal_y - robot.loc.y
        err_list = [i-robot.orientation for i in self.goal_theta]
        abs_list = [abs(sk) for sk in err_list]
        err_t2 = err_list[abs_list.index(min(abs_list))]
        
        dist_error = (float(err_x2)**2+float(err_y2)**2)**0.5
        self.err_x_run += err_x2*dt
        self.err_y_run += err_y2*dt
        self.err_t_run += err_t2*dt
        err_x_delta = (err_x2-self.err_x)/dt
        err_y_delta = (err_y2-self.err_y)/dt
        err_t_delta = (err_t2-self.err_t)/dt
        self.err_x = err_x2
        self.err_y = err_y2
        self.err_t = err_t2
        # print("x_err: ",  self.err_x, " y_err: ",self.err_y)

        f_dot = (K_Px*self.err_x+K_Ix*self.err_x_run+K_Dx*err_x_delta)*math.cos(robot.orientation) + (K_Py*self.err_y+K_Iy*self.err_y_run+K_Dy*err_y_delta)*math.sin(robot.orientation) # Forward
        h_dot = (K_Px*self.err_x+K_Ix*self.err_x_run+K_Dx*err_x_delta)*math.sin(-robot.orientation) + (K_Py*self.err_y+K_Iy*self.err_y_run+K_Dy*err_y_delta)*math.cos(robot.orientation) # Horiztontal
        # print("h_vel: ", h_dot, " f_vel: ",f_dot)
        a_dot =  K_Pt*self.err_t+K_It*self.err_t_run+K_Dt*err_t_delta# Angle rate
        commands.setWalkVelocity(max(min(f_dot,max_speed),-max_speed),max(min(h_dot,max_speed),-max_speed),max(min(a_dot,max_speed),-max_speed))




class ReachCenter(Node):
    def run(self):
        commands.setHeadPan(0,0.5)
        # commands.setHeadTilt(tilt_angle)
        robot= memory.world_objects.getObjPtr(5)
        memory.speech.say("Reached center")
        print("Reached center")
        commands.setWalkVelocity(0,0,0)
        print("Final robot ", robot.loc," orientation: ",robot.orientation)
        print("Final Position covariance ",robot.sd.getMagnitude(),"cov_or: ",robot.sdOrientation)


class TurnInPlace(Node):
    def run(self):
        commands.setHeadPan(0,0.1)
        commands.setHeadTilt(tilt_angle)
        robot= memory.world_objects.getObjPtr(5)
        # memory.speech.say("I'm still turning")
        print("Looking for another beacon")

        sgn = math.copysign(1,robot.loc.x)*math.copysign(1,robot.orientation)
        commands.setWalkVelocity(0,0,sgn*0.1)
        # print("Final robot ", robot.loc," orientation: ",robot.orientation)
        # print("Final Position covariance ",robot.sd.getMagnitude(),"cov_or: ",robot.sdOrientation)

class reinit(Node):
    def run(self):
        pose.sit()
        commands.setStiffness(cfgstiff.Zero)
        # pose.BlockCenter()
        UTdebug.log(15, "Blocking center")


class Finder(Node):
    
    robot= memory.world_objects.getObjPtr(5)
    goal_x = 0
    goal_y = 0
    yawFlag = True
    centerFlag = False
    def run(self):
        # commands.setHeadTilt(tilt_angle)
        commands.stand()  
        beacons = {"WO_BEACON_BLUE_YELLOW" : memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW),
        "WO_BEACON_YELLOW_BLUE" : memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE),
        "WO_BEACON_BLUE_PINK" : memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK),
        "WO_BEACON_PINK_BLUE" : memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE),
        "WO_BEACON_PINK_YELLOW" : memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW),
        "WO_BEACON_YELLOW_PINK" : memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)}
        beacon_seen = [beacons[beacon].seen for beacon in beacons]
        no_seen_beacons = sum([int(bs) for bs in beacon_seen])
        sd_pos = self.robot.sd.getMagnitude()
        sd_or = self.robot.sdOrientation
        print("robot ", self.robot.loc," orientation: ",self.robot.orientation)
        print("Position covariance ",self.robot.sd.getMagnitude(),"cov_or: ",self.robot.sdOrientation)
        err_x = self.goal_x - self.robot.loc.x
        err_y = self.goal_y - self.robot.loc.y
        if (sd_pos > xy_threshold or sd_or > t_threshold) and not self.centerFlag:
            if no_seen_beacons < 2 and core.joint_values[core.HeadYaw] <=0.75 and self.yawFlag:
                self.postSignal("lookleft")
            elif no_seen_beacons < 2 and core.joint_values[core.HeadYaw] >= -0.75:
                self.yawFlag = False
                self.postSignal("lookright")
            elif no_seen_beacons>=2:
                self.postSignal("holdlook")
                self.yawFlag = False
            else:
                self.yawFlag = True
        else:
            if (err_x**2+err_y**2)**0.5 < center_threshold and (err_x**2+err_y**2)**0.5 > 0.0 and sd_pos<75:
                self.postSignal("reachedcenter")
            elif (err_x**2+err_y**2)**0.5 < center_threshold and (err_x**2+err_y**2)**0.5 > 0.0 and sd_pos>=75:
                self.postSignal("turn")
            elif self.robot.loc.getMagnitude()>=500 and not self.centerFlag:
                global point_x,point_y,point_t
                point_x = -250*math.copysign(1,self.robot.loc.x)
                point_y = -250*math.copysign(1,self.robot.loc.x)
                point_t = math.atan2(point_y,point_x)
                self.postSignal("movepoint")
            elif no_seen_beacons>=2 or 1 == 1:
                self.centerFlag = True
                self.postSignal("movecenter")
            elif no_seen_beacons<2:
                self.centerFlag = True
                self.postSignal("turn")


          




class Diagnostic(Node):

    robot= memory.world_objects.getObjPtr(5)
    def run(self):
        print("robot ", self.robot.loc," orientation: ",self.robot.orientation)
        print("Position covariance ",self.robot.sd.getMagnitude(),"cov_or: ",self.robot.sdOrientation)
        if core.joint_values[core.HeadYaw] <=0.75 :
            self.postSignal("lookleft")
        else:
            self.postSignal("holdlook")


class Playing(LoopingStateMachine):
    
    def setup(self):
        finder = Finder()
        diag = Diagnostic()
        blocks = {"lookleft": HeadLeft(),
                  "lookright": HeadRight(),
                  "holdlook": HeadStill(),
                  "movecenter": MoveCenter(),
                  "movepoint": MoveToPoint(),
                  "reachedcenter":ReachCenter(),
                  "turn":TurnInPlace(),
                  "sit": reinit()
                  }
        # sit = reinit()
        # self.trans(self.Stand(),C)
        for name in blocks:
            b = blocks[name]
            print("Block module: ",b)
            self.add_transition(finder, S(name), b, T(0.5), finder)