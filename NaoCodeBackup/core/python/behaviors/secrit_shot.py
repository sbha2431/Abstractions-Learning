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
global robotlocx,robotlocy
robotlocx=-750
robotlocy=0
point_x = 0
point_y = 0
point_t = 0
tilt_angle = -0.1
xy_threshold = 100
center_threshold = 100
global t_threshold
t_threshold = 0.5

class Stand(Node):
    def run(self):
        commands.stand()
        commands.setWalkVelocity(0,0,0.05)
        if self.getTime() > 5.0:
            commands.setWalkVelocity(0,0,0)
            memory.speech.say("Starting localization")
            self.finish()

        

class HeadLeft(Node):
    def run(self):
        commands.setHeadPan(core.joint_values[core.HeadYaw]+0.12,0.06)
        commands.setHeadTilt(tilt_angle)
        # print("Block left")
        # commands.setLShoulderRoll(1.25, 0.15)
        # pose.BlockLeft()
        # UTdebug.log(15, "Blocking left")



class HeadRight(Node):
    def run(self):
        commands.setHeadPan(core.joint_values[core.HeadYaw]-0.12,0.06)
        commands.setHeadTilt(tilt_angle)
        # print("Block left")
        # commands.setLShoulderRoll(1.25, 0.15)
        # pose.BlockLeft()
        # UTdebug.log(15, "Blocking left")

class HeadStill(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        # commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05,0.01)
        pass


class BallSearch(Node):
    yawFlag = True
    turnFlag = False

    def run(self):

        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        maxYaw = 1.0
        minYaw = -1.0
        print(core.joint_values[core.HeadYaw])
        


        if ball.seen:
            print("Elevation ",ball.visionElevation,", Bearing ",ball.visionBearing,", Distance ",ball.visionDistance)
            # commands.setHeadPan(0,0.2)
            # return ball.visionElevation, ball.visionBearing,ball.visionDistance
        elif not ball.seen and core.joint_values[core.HeadYaw] <=maxYaw and self.yawFlag and not self.turnFlag:
            #print("turn+")
            commands.setHeadPan(core.joint_values[core.HeadYaw]+0.05,0.01)
            if core.joint_values[core.HeadYaw]+0.05 >= maxYaw:
                self.yawFlag=False
        elif not ball.seen and core.joint_values[core.HeadYaw] >=minYaw and not self.yawFlag and not self.turnFlag:
            #print("turn-")
            commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05,0.01)
            if core.joint_values[core.HeadYaw]-0.05 <= minYaw:
                self.yawFlag=True
                self.turnFlag=True
        elif not ball.seen and self.turnFlag:
            commands.setHeadPan(0,0.1)
            commands.setWalkVelocity(0, 0, 0.2)


class Turn2Ball(Node):

    def run(self):

        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        print("Bearing ",ball.visionBearing,", Distance ",ball.visionDistance)
        if abs(ball.visionBearing) > 0.05:
            print('turning')
            sgn = 1
            sgn = math.copysign(sgn,ball.visionBearing)
            commands.setWalkVelocity(0, 0, 0.2*sgn)
            commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05*sgn,0.2)
        else:
            commands.setHeadPan(0,0.1)
            if abs(ball.visionBearing) < 0.05:
                print('done')
                self.finish()
            commands.setWalkVelocity(0, 0, 0)

class Turn2BallAnti(Node):

    def run(self):

        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        print("Bearing ",ball.visionBearing,", Distance ",ball.visionDistance)
        if abs(ball.visionBearing) > 0.05:
            print('turning')
            sgn = 1
            sgn = math.copysign(sgn,ball.visionBearing)
            commands.setWalkVelocity(0, 0, 0.2*sgn)
            commands.setHeadPan(core.joint_values[core.HeadYaw]-0.05*sgn,0.2)
        else:
            commands.setHeadPan(0,0.1)
            if abs(ball.visionBearing) < 0.05:
                print('done')
                self.finish()
            commands.setWalkVelocity(0, 0, 0)

class Walk2Ball(Node):
    ballseentimeflag=0
    ballseentime=0
    ballseenleave=0
    Flag = False
    err_d_run = 0
    err_t_run = 0
    err_dist = 0
    err_theta = 0

    def run(self):
        # print("Start walk")
        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        goal_dist = 200
        goal_theta = 0
        max_speed = 1.0
        dt = 1e-2
        # print("Elevation ",ball.visionElevation,", Bearing ",ball.visionBearing,", Distance ",ball.visionDistance)
        err_dist2 = ball.visionDistance-goal_dist
        err_theta2 = ball.visionBearing-goal_theta
        K_P = 7e-4
        K_Pt= 1
        K_I = 5e-4
        K_It= 1e-3
        K_D = 0.0
        K_Dt= 0.0
        self.err_d_run += err_dist2*dt
        self.err_t_run += err_theta2*dt
        err_d_delta = (err_dist2-self.err_dist)/dt
        err_t_delta = (err_theta2-self.err_theta)/dt
        self.err_dist = err_dist2
        self.err_theta = err_theta2
        x_dot = K_P*self.err_dist+K_I*self.err_d_run+K_D*err_d_delta
        t_dot = K_Pt*self.err_theta+K_It*self.err_t_run+K_Dt*err_t_delta
        # print("Output velocity ",x_dot)
        # print("Output angular velocity ",t_dot)
        commands.setWalkVelocity(min(x_dot,max_speed), 0, max(min(t_dot,max_speed),-max_speed))




class CircleWalk(Node):

    def run(self):
        print("circling clock")
        commands.setWalkVelocity(0,0.25,-0.075)
        commands.setHeadPan(0,0.1)
        commands.setHeadTilt(-0.1)


class CircleWalkAnti(Node):

    def run(self):
        print("circling anti")
        commands.setWalkVelocity(0,-0.25,+0.075)
        commands.setHeadPan(0,0.1)
        commands.setHeadTilt(-0.1)



class Approach(Node):
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
        goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
        dt = 1e-2
        # Gain values
        # K_P = np.diag([1e-2,1e-3,1e-3])
        # K_I = np.diag([1e-3,1e-3,1e-3])
        # K_D = np.eye(3)*1e-6
        K_Px = 2e-2
        K_Py = 2e-3
        K_Pt = 1.0
        K_Ix = 8e-3
        K_Iy = 1e-3
        K_It = 1e-2
        K_Dx = 1e-6
        K_Dy = 1e-6
        K_Dt = 1e-5

        # Saturation block - low since we are close
        max_speed = 0.3
        # pos = np.array([ball.imageCenterX,ball.imageCenterY,-goal.visionBearing])
        # print("Target shape", self.target.shape," pos shape ", pos.shape)
        # err2 = self.target-pos
        # print(err2.shape)
        err_x2 = self.x_pos-ball.imageCenterX
        err_y2 = self.y_pos-ball.imageCenterY
        err_t2 = goal.visionBearing-self.theta_pos
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
        print("Output h: ",h_dot,", Output w: ",w_dot,"Output t",t_dot)
        # print("Output2 h: ",out_com[1],", Output2 w: ",out_com[0],"Output2 t",out_com[2])
        
        commands.setHeadPan(0,0.1)
        commands.setWalkVelocity(min(h_dot,max_speed),max(min(w_dot,max_speed),-max_speed),max(min(t_dot,max_speed),-max_speed))
        # commands.setWalkVelocity(0,0,max(min(t_dot,max_speed),-max_speed))
        print("Distance error : ", dist_error,"Goal angle error: ",err_t2,"   Command out: ",command_out)
        # print("Distance error : ", dist_error,"   &  ",dist_error2)
        # if dist_error<= 6 and err_t2<=0.15:
        #     print("Ready to kick. Ball at: ",ball.imageCenterX,", ",ball.imageCenterY)
        #     commands.setWalkVelocity(0,0,0)
        #     self.finish()



class DribbleRight(Node):
    # Right kick
    x_pos = 204
    y_pos = 360
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
        goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
        line = memory.world_objects.getObjPtr(core.WO_OWN_PENALTY)
        dt = 1e-2
        # Gain values
        # K_P = np.diag([1e-2,1e-3,1e-3])
        # K_I = np.diag([1e-3,1e-3,1e-3])
        # K_D = np.eye(3)*1e-6
        K_Px = 1e-2
        K_Py = 1e-3
        K_Pt = 1e-1
        K_Ix = 1e-3
        K_Iy = 1e-3
        K_It = 1e-2
        K_Dx = 1e-6
        K_Dy = 1e-6
        K_Dt = 1e-5

        # Saturation block - low since we are close
        max_speed = 0.4
        # pos = np.array([ball.imageCenterX,ball.imageCenterY,-goal.visionBearing])
        # print("Target shape", self.target.shape," pos shape ", pos.shape)
        # err2 = self.target-pos
        # print(err2.shape)
        err_x2 = self.x_pos-ball.imageCenterX
        err_y2 = self.y_pos-ball.imageCenterY
        err_t2 = goal.visionBearing-self.theta_pos
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
        print("Output h: ",h_dot,", Output w: ",w_dot,"Output t",t_dot)
        # print("Output2 h: ",out_com[1],", Output2 w: ",out_com[0],"Output2 t",out_com[2])
        
        commands.setHeadPan(0,0.1)
        commands.setWalkVelocity(min(h_dot,max_speed),max(min(w_dot,max_speed),-max_speed),max(min(t_dot,max_speed),-max_speed))
        # commands.setWalkVelocity(0,0,max(min(t_dot,max_speed),-max_speed))
        print("Distance error : ", dist_error, "Goal angle error: ",err_t2,"   Command out: ",command_out)
        # print("Distance error : ", dist_error,"   &  ",dist_error2)
        print("Goal distance: ", goal.visionDistance)
        if line.seen:
            print(line.visionDistance)
            if line.visionDistance<500:
                commands.setWalkVelocity(0,0,0)


class DribbleLeft(Node):
    # Right kick
    x_pos = 164
    y_pos = 360
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
        goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
        line = memory.world_objects.getObjPtr(core.WO_OWN_PENALTY)
        dt = 1e-2
        # Gain values
        # K_P = np.diag([1e-2,1e-3,1e-3])
        # K_I = np.diag([1e-3,1e-3,1e-3])
        # K_D = np.eye(3)*1e-6
        K_Px = 1e-2
        K_Py = 1e-3
        K_Pt = 1e-1
        K_Ix = 1e-3
        K_Iy = 1e-3
        K_It = 1e-2
        K_Dx = 1e-6
        K_Dy = 1e-6
        K_Dt = 1e-5

        # Saturation block - low since we are close
        max_speed = 0.4
        # pos = np.array([ball.imageCenterX,ball.imageCenterY,-goal.visionBearing])
        # print("Target shape", self.target.shape," pos shape ", pos.shape)
        # err2 = self.target-pos
        # print(err2.shape)
        err_x2 = self.x_pos-ball.imageCenterX
        err_y2 = self.y_pos-ball.imageCenterY
        err_t2 = goal.visionBearing-self.theta_pos
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
        print("Output h: ",h_dot,", Output w: ",w_dot,"Output t",t_dot)
        # print("Output2 h: ",out_com[1],", Output2 w: ",out_com[0],"Output2 t",out_com[2])
        
        commands.setHeadPan(0,0.1)
        commands.setWalkVelocity(min(h_dot,max_speed),max(min(w_dot,max_speed),-max_speed),max(min(t_dot,max_speed),-max_speed))
        # commands.setWalkVelocity(0,0,max(min(t_dot,max_speed),-max_speed))
        print("Distance error : ", dist_error, "Goal angle error: ",err_t2,"   Command out: ",command_out)
        # print("Distance error : ", dist_error,"   &  ",dist_error2)
        print("Goal distance: ", goal.visionDistance)
        if line.seen:
            print(line.visionDistance)
            if line.visionDistance<500:
                commands.setWalkVelocity(0,0,0)



        
class Diagnostic(Node):
    #goalhold = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
    file = open('sensorout.txt','w')
    def run(self):
        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

        commands.setHeadPan(0,0.1)
        #print("Goal hold angle ",self.goalhold.visionBearing)
        #print("Head tilt ",core.joint_values[core.HeadPitch])

        if ball.seen:
            print("Ball at ",ball.px,", ",ball.py,"Bearing: ",ball.visionBearing," Range: ",ball.visionDistance)
            self.file.write('{},{},{},{}\n'.format(ball.px,ball.py,ball.visionBearing,ball.visionDistance))
        if goal.seen:
            print("Goal at ",goal.imageCenterX,", ",goal.imageCenterY, "Bearing ", goal.visionBearing)
        if self.getTime() > 120.0:
            self.file.close()
            self.finish()

class NoneClass(Node):
    def run(self):
        print("None class")

            
class Kick(Node):
    def run(self):
        if self.getFrames() <= 3:
            memory.walk_request.noWalk()
            memory.kick_request.setFwdKick()
        if self.getFrames() > 10 and not memory.kick_request.kick_running_:
            commands.stand()
            self.finish()

class reinit(Node):
    def run(self):
        pose.sit()
        commands.setStiffness(cfgstiff.Zero)
        # pose.BlockCenter()
        UTdebug.log(15, "Blocking center")


class Offense(Node):

    
    robot= memory.world_objects.getObjPtr(5)
    x_pos = 184
    y_pos = 190
    k_time = 0
    b_time = 0
    theta_pos = 0
    antiFlag = False
    walkFlag = True
    appFlag = False
    kickFlag = False
    circFlag = False
    def run(self):
        goal_dist = 250
        commands.stand()
        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        robot = memory.world_objects.getObjPtr(5)
        line = memory.world_objects.getObjPtr(core.WO_OWN_PENALTY)
        self.b_time += 0.25
        if ball.seen:
            self.b_time = 0
        print("ball seen time", self.b_time)
        goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
        err_dist = ball.visionDistance-goal_dist
        err_x2 = self.x_pos-ball.imageCenterX
        err_y2 = self.y_pos-ball.imageCenterY
        err_t2 = goal.visionBearing-self.theta_pos
        dist_error = (float(err_x2)**2+float(err_y2)**2)**0.5
        beacons = {"WO_BEACON_BLUE_YELLOW" : memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW),
        "WO_BEACON_YELLOW_BLUE" : memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE),
        "WO_BEACON_BLUE_PINK" : memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK),
        "WO_BEACON_PINK_BLUE" : memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE),
        "WO_BEACON_PINK_YELLOW" : memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW),
        "WO_BEACON_YELLOW_PINK" : memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)}
        beacons_clock = ["WO_BEACON_BLUE_PINK","WO_BEACON_PINK_BLUE"]
        beacons_anti = ["WO_BEACON_BLUE_YELLOW","WO_BEACON_YELLOW_BLUE","WO_BEACON_PINK_YELLOW","WO_BEACON_YELLOW_PINK"]
        beacon_seen = [beacons[beacon].seen for beacon in beacons]
        beacon_seen_clock = [beacons[beacon].seen for beacon in beacons_clock]
        beacon_seen_anti = [beacons[beacon].seen for beacon in beacons_anti]
        no_seen_beacons = sum([int(bs) for bs in beacon_seen])
        print("Ball distance: ",ball.visionDistance," ")
        if (any(beacon_seen_anti) and abs(core.joint_values[core.HeadYaw]) <=0.1)  or self.antiFlag:
            self.antiFlag = True
        else:
            self.antiFlag = False
        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        maxYaw = 1.0
        minYaw = -1.0
        if self.b_time <3.0 and self.walkFlag and not self.kickFlag:
            if err_dist<=0:
                print("Done at ",ball.imageCenterX,", ",ball.imageCenterY)
                commands.setWalkVelocity(0, 0, 0)
                self.walkFlag = False
                self.circFlag = True
            else:
                self.postSignal("walkball")
        elif self.b_time <3.0 and not self.kickFlag and self.circFlag:
            if self.antiFlag:
                if goal.seen:
                    print("goal angle ",goal.visionBearing)
                    if abs(goal.visionBearing)<=0.25:
                        commands.setWalkVelocity(0,0,0)
                        self.circFlag = False
                        self.postSignal("dribbler")
                    else:
                        self.postSignal("circle_anti")
                else:
                    self.postSignal("circle_anti")
            else:
                if goal.seen:
                    print("goal angle ",goal.visionBearing)
                    if abs(goal.visionBearing)<=0.25:
                        commands.setWalkVelocity(0,0,0)
                        self.circFlag = False
                        self.postSignal("dribblel")                
                    else:
                        self.postSignal("circle")
                else:
                    self.postSignal("circle")
        elif self.b_time <3.0 and not self.kickFlag and not self.circFlag:
            print("Approaching ball")
            if line.visionDistance<= 500 or self.appFlag:
                self.appFlag = True
                if dist_error<= 6 and err_t2<=0.15:
                    print("Ready to kick. Ball at: ",ball.imageCenterX,", ",ball.imageCenterY)
                    commands.setWalkVelocity(0,0,0)
                    self.appFlag = False
                    self.kickFlag = True
                else:
                    self.postSignal("approach")

            else:
                self.postSignal("dribblel")
        elif self.kickFlag:
            print("kicking ball")
            self.kickFlag = False
            self.postSignal("kick")
            self.walkFlag = True
        elif self.b_time>=1.0:
            print("seraching for ball")
            self.postSignal("ballsearch")

          




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
        off = Offense()
        diag = Diagnostic()
        blocks = {"approach": Approach(),
                  "ballsearch":BallSearch(),
                  "circle_anti":CircleWalkAnti(),
                  "circle":CircleWalk(),
                  "dribbler":DribbleRight(),
                  "dribblel":DribbleLeft(),
                  "none": NoneClass(),
                  "sit": reinit(),
                  "walkball": Walk2Ball(),
                  "kick":Kick()
                  }
        kicklist = ["kick"]
        drib = ["dribbler","dribblel"]
        # sit = reinit()
        # self.trans(self.Stand(),C)
        for name in blocks:
            b = blocks[name]
            print("Block module: ",b)
            if name in kicklist:
                self.add_transition(off, S(name), b, T(10.5), off)
            elif name in drib:
                print(name)
                self.add_transition(off, S(name), b, T(15.0), off)
            else:
                self.add_transition(off, S(name), b, T(0.5), off)

        #             stand = self.Stand()
        # sit = pose.Sit()
        # # Goalseen = self.GoalSeen()
        # # Ballseen = self.BallSeen()
        # off = self.Off()
        # srch = self.BallSearch()
        # trn = self.Turn2Ball()
        # wlk = self.Walk2Ball()
        # ball = memory.world_objects.getObjPtr(core.WO_BALL)
        # crc = self.CircleWalk()
        # diag = self.Diagnostic()
        # app  = self.Approach()
        # drb  = self.Dribble()
        # #self.trans(stand,C)

        #     #print("a")
        # # self.trans(stand,C,diag,T(120.0),sit,C,off)

        # # self.trans(stand,C,crc,C,app,C,sit,C,off)
        # #self.trans(stand,C,app,C,self.Stable(),C,self.Kick(),C,sit,C,off)
        # # self.trans(stand,C,srch,C,trn,C, wlk,C,crc,C,app,C,self.Kick(),C,sit,C,off)
        # self.trans(stand,C,srch,C,trn,C, wlk,C,crc,C,drb,C,self.Stable(),C,app,C,self.Stable(),C,self.Kick(),C,sit,C,off)