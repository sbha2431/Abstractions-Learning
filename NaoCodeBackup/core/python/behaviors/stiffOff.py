"""Simple behavior that stands, kicks, and then sits down."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import commands
import cfgstiff
from state_machine import StateMachine, Node, C


class Playing(StateMachine):
    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            self.finish()

    def setup(self):
        # self.trans(self.Stand(), C, self.Kick(), C, self.Stand(),
        #            C, pose.Sit(), C, self.Off())
        self.trans(self.Off(),C)
