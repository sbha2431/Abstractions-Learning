"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from memory import localization_mem
from state_machine import Node, S, T, LoopingStateMachine
import UTdebug
import numpy
from numpy.linalg import inv



class BlockLeft(Node):
    def run(self):
        UTdebug.log(15, "Blocking left")



class BlockRight(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")


class BlockCenter(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")


class Blocker(Node):
    n=4
    m=2
    dt=1/5.0
    A=numpy.zeros((n,n))
    A[0,0]=1
    A[1,1]=1
    A[2,2]=0.966
    A[3,3]=0.966
    A[0,2]=dt
    A[1,3]=dt
    C=numpy.zeros((m,n))
    C[0,0]=1
    C[1,1]=1
    Q=numpy.zeros((n,n))
    Q[0,0]=1
    Q[1,1]=1
    Q[2,2]=1
    Q[3,3]=1
    R=numpy.zeros((m,m))
    R[0,0]=300
    R[1,1]=900
    P=numpy.zeros((n,n))
    P[0,0]=1000
    P[1,1]=1000
    P[2,2]=1000
    P[3,3]=1000
    xinit=numpy.zeros((n,1))
    xinit[0]=1500
    xinit[1]=0
    xinit[2]=0
    xinit[3]=0
    #file = open('sensorout.txt','w')
    number=0




    def run(self):
        # ball = mem_objects.world_objects[core.WO_BALL]
        print(localization_mem)
        # temp = localization_mem
        print(dir(localization_mem))
        # print(dir(mem_objects))
        # print("1")
        # print(dir(mem_objects.world_objects))
        #for object1 in mem_objects.world_objects:
            #print(object1)


            

            # self.P[0,0]=1000
            # self.P[1,1]=1000
            # self.P[2,2]=1000
            # self.P[3,3]=1000
            # self.xinit[0]=1500
            # self.xinit[1]=0
            # self.xinit[2]=0
            # self.xinit[3]=0



class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": BlockLeft(),
                  "right": BlockRight(),
                  "center": BlockCenter()
                  }
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(5), blocker)