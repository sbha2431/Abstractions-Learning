
from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import math
import commands
import mem_objects
import cfgpolicy
import cfgpolicy_learned
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

tilt_angle = -0.1
            
class Kick(Node):
    def run(self):
        if self.getFrames() <= 3:
            memory.walk_request.noWalk()
            memory.kick_request.setFwdKick()
        if self.getFrames() > 10 and not memory.kick_request.kick_running_:
            commands.stand()
            self.finish()

class NoneClass(Node):
    def run(self):
        print("None class")
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0,0,0)

class stepForward(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0.35,0,0.05)


class stepLeft(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0.0,0.25,0)


class stepRight(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0.0,-0.25,0)



class stepForwardLeft(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0.2,0.35,0)


class stepForwardRight(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0.2,-0.35,0)

class stepBack(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(-0.2,0,0)


class turnRight(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0,0,-0.1)

class turnLeft(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0,0,0.1)

class stopStill(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0,0,0)

class wheelLeft(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0.35,0,0.35)

class wheelRight(Node):
    def run(self):
        commands.setHeadTilt(tilt_angle)
        commands.setWalkVelocity(0.35,0,-0.25)


class Offense(Node):
    policy = cfgpolicy.policy
    stop_cnt = 0
    # policy = cfgpolicy_learned.policy
    test_flag = False
    def run(self):
        commands.setHeadTilt(tilt_angle)
        out_Pan = min([max([1-2*core.joint_values[core.HeadYaw],-0.7]),0.7])
        print("Head angle ",out_Pan)
        commands.setHeadPan(out_Pan,2)
        commands.stand()
        robot   = memory.world_objects.getObjPtr(5)
        ball    = memory.world_objects.getObjPtr(core.WO_BALL)
        goal    = memory.world_objects.getObjPtr(core.WO_OWN_GOAL)
        beacons = {"WO_BEACON_BLUE_YELLOW" : memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW),
        "WO_BEACON_YELLOW_BLUE" : memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE),
        "WO_BEACON_BLUE_PINK" : memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK),
        "WO_BEACON_PINK_BLUE" : memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE),
        "WO_BEACON_PINK_YELLOW" : memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW),
        "WO_BEACON_YELLOW_PINK" : memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)}
        x_start = -1200
        x_end = 1200
        y_start = -1200
        y_end = 1200
        discretize_loc = 100
        x_bins = [e for e in range(x_start,x_end+1,discretize_loc)]
        y_bins = [e for e in range(y_start,y_end+1,discretize_loc)]
        # xy_bins = [-800,-705,-611,-517,-423,-329,-235,-141,-47,47,141,235,329,423,517,611,705,800]
        t_start = 90
        t_end = 270
        discretize_trans = 30
        t_bins = [e for e in range(t_start,t_end+1,discretize_trans)]
        # t_bins = [0,18,36,54,72,108,126,144,162,180,198,216,234,252,270,288,306,324,342,360]
        rad_2_deg = 57.29
        r_orientation = (robot.orientation*rad_2_deg) % 360 #(robot.orientation+3.14) % (2*3.14)*rad_2_deg
        takeClosest = lambda num,collection:min(collection,key=lambda x:abs(x-num))
        # state = (int(robot.loc.x-robot.loc.x%discretize_loc),int(robot.loc.y-robot.loc.y%discretize_loc),int(r_orientation-(r_orientation%discretize_trans)))
        state = (takeClosest(robot.loc.x,x_bins),takeClosest(robot.loc.y,y_bins),takeClosest(r_orientation,t_bins))
        # ball_state = ball.
        print("Position ",robot.loc.x," ",robot.loc.y,"  -  Trans ",r_orientation)
        print("State ",state[0]," ",state[1],"  -  Trans ",state[2])
        pol_str = self.policy.get(state,"none")
        # print("Flag: ",self.test_flag)      
        # if self.test_flag is True:
        #     self.postSignal("none")
        # else:    
        #     self.test_flag = True
        #     self.postSignal("turnleft")
        if pol_str is 'stop':
            self.stop_cnt += 1
        if self.stop_cnt > 50:
            pol_str = 'stop' 
        print("Stop count ",self.stop_cnt," Action: ",pol_str)
        self.postSignal(pol_str)



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
        action_blocks = {"forward": stepForward(),
                  "back":stepBack(),
                  "turnleft":turnLeft(),
                  "turnright":turnRight(),
                  "left":stepLeft(),
                  "right": stepRight(),
                  "forwardleft": stepForwardLeft(),
                  "forwardright": stepForwardRight(),
                  "none": NoneClass(),
                  "stop": stopStill(),
                  "kick":Kick(),
                  "wheelleft":wheelLeft(),
                  "wheelright":wheelRight()
                  }
        kicklist = ["kick"]
        time_step  = 2.0
        for name in action_blocks:
            b = action_blocks[name]
            print("Block module: ",b)
            if name in kicklist:
                self.add_transition(off, S(name), b, T(10.5), off)
            else:
                self.add_transition(off, S(name), b, T(time_step), off)
