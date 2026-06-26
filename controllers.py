import numpy as np
from pydrake.all import (
    LeafSystem
)

from constants import *

class SelectorController(LeafSystem):
    def __init__(self):
        super().__init__()

        # self.mode = "swing_up"      # initialise with swing up mode

        self.state_port = self.DeclareVectorInputPort(name="estimated_state", size=6)
        self.swing_up = self.DeclareVectorInputPort(name="swing_up", size=1)
        self.up_up_balance = self.DeclareVectorInputPort(name="up_up_balance", size=1)

        self.fsm_state = self.DeclareDiscreteState(2)   # the current state of the system and time

        self.DeclareVectorOutputPort(name="actuation",
                                     size=1,
                                     calc=self.CalcOutput)

        self.DeclareVectorOutputPort("tvlqr_time",
                                     1,
                                     self.UpdateTVLQRTime,
                                     {self.all_state_ticket()})

        # periodic function call to update the state
        self.DeclarePeriodicDiscreteUpdateEvent(
                period_sec=0.02,
                offset_sec=0.0,
                update=self.UpdateMode
        )

    def UpdateTVLQRTime(self, context, output):

        fsm = context.get_discrete_state(self.fsm_state).get_value()
        start_time = fsm[1]

        output.SetFromVector([start_time])

    def UpdateMode(self, context, discrete_state):
        x = self.state_port.Eval(context)

        # mode = discrete_state.get_mutable_vector().GetAtIndex(0)
        # mode_start_time = discrete_state.get_mutable_vector().GetAtIndex(1)
        fsm = discrete_state.get_mutable_vector(self.fsm_state)
        mode = int(fsm.GetAtIndex(0))
        mode_start_time = fsm.GetAtIndex(1)
        time = context.get_time()

        elapsed_time = time - mode_start_time

        if mode == DOWN_BALANCE:
            if elapsed_time >= 5:
                # mode = SWING_UP
                print("Switching to swing up")
                fsm.SetAtIndex(0, SWING_UP)
                fsm.SetAtIndex(1, time)
        elif mode == SWING_UP:
            # check if the goal state has been reached

            print(x[1], x[2], x[4], x[5])

            # if(
            #     abs(abs(x[1]) - np.pi) < 1.0 and
            #     abs(abs(x[2]) - 0.0) < 1.0 and
            #     abs(x[4]) < 1.5 and
            #     abs(x[5]) < 1.5):
            if (
                    abs(abs(x[1]) - np.pi) < 0.2 and
                    abs(abs(x[2]) - 0.0) < 0.2
                    ):

                fsm.SetAtIndex(0, UP_BALANCE)
                fsm.SetAtIndex(1, time)

                print("Switching to up balance")

        elif mode == UP_BALANCE:
            if elapsed_time >= 5:
                fsm.SetAtIndex(0, SWING_DOWN)
                fsm.SetAtIndex(1, time)

                print("Switching to swing down")

        elif mode == SWING_DOWN:

            if(
                abs(abs(x[1])) < 0.1 and
                abs(abs(x[2])) < 0.1 and
                abs(x[4]) < 0.5 and
                abs(x[5]) < 0.5):

                fsm.SetAtIndex(0, DOWN_BALANCE)
                fsm.SetAtIndex(1, time)

                print("Switching to down balance")

    def CalcOutput(self, context, output):

        fsm = context.get_discrete_state(self.fsm_state).get_value()
        mode = int(fsm[0])

        if mode == SWING_UP:
            output.SetFromVector(self.swing_up.Eval(context))
        elif mode == DOWN_BALANCE:
            output.SetFromVector([0.0])
        else:
            output.SetFromVector(self.up_up_balance.Eval(context))


class TVLQRController(LeafSystem):
    def __init__(self, K_traj, x_traj, u_traj):
        super().__init__()

        self.K_traj = K_traj
        self.x_traj = x_traj
        self.u_traj = u_traj

        self.state_port = self.DeclareVectorInputPort(name="estimated_state", size=6)
        self.start_time = self.DeclareVectorInputPort(name="start_time", size=1)
        self.DeclareVectorOutputPort(name="force",
                                       size=1,
                                       calc=self.CalcOutput)

    def CalcOutput(self, context, output):
        current_time = context.get_time()
        start_time = self.start_time.Eval(context)[0]

        t = current_time - start_time

        x = self.state_port.Eval(context)
        x_des = self.x_traj.value(t).flatten()
        u_des = self.u_traj.value(t)[0]
        K = self.K_traj.value(t)

        u = u_des - K @ (x - x_des)
        output.SetFromVector(u)


class LQRController(LeafSystem):
    def __init__(self, K):
        super().__init__()

        self.K = K

        self.state_port = self.DeclareVectorInputPort(name="estimated_state", size=6)
        self.target_state_port = self.DeclareVectorInputPort(name="target_state", size=6)

        self.DeclareVectorOutputPort(name="force",
                                     size=1,
                                     calc=self.CalcOutput)

    def CalcOutput(self, context, output):
        estimated_state = self.state_port.Eval(context)
        target_state = self.target_state_port.Eval(context)

        u = -self.K @ (estimated_state - target_state)

        output.SetFromVector([u])
