import numpy as np
from pydrake.all import (
    LeafSystem
)

from constants import *

class SelectorController(LeafSystem):
    def __init__(self, initial_state, lqr_gains, tvlqr_gains, state_dict):
        super().__init__()

        self.current_equilibrium = initial_state
        self.target_equilibrium = initial_state
        self.state_dict = state_dict
        self.lqr_gains = lqr_gains
        self.tvlqr_gains = tvlqr_gains
        self.transition = None

        self.state_port = self.DeclareVectorInputPort(name="estimated_state", size=6)
        self.fsm_state = self.DeclareDiscreteState(2)   # the current state of the system and time

        self.DeclareVectorOutputPort(name="actuation",
                                     size=1,
                                     calc=self.CalcOutput)

        # periodic function call to update the state
        self.DeclarePeriodicDiscreteUpdateEvent(
                period_sec=0.01,
                offset_sec=0.0,
                update=self.UpdateMode
        )

    def SetTargetState(self, target_state):
        self.target_equilibrium = target_state

    def UpdateMode(self, context, discrete_state):

        # mode = discrete_state.get_mutable_vector().GetAtIndex(0)
        # trans_start_time = discrete_state.get_mutable_vector().GetAtIndex(1)
        fsm = discrete_state.get_mutable_vector(self.fsm_state)
        mode = int(fsm.GetAtIndex(0))
        mode_start_time = fsm.GetAtIndex(1)

        time = context.get_time()

        if (mode == BALANCE) and (self.target_equilibrium != self.current_equilibrium):
            # request to transition
            # may have to add a stability check in the future to
            # make sure that it is currently balanced at initial state
            transition = f"{self.current_equilibrium}_{self.target_equilibrium}"

            # check that the transition is in the tvlqr_gains
            computed_transitions = self.tvlqr_gains.keys()
            if transition not in computed_transitions:
                print(f"Error: Transition trajectory '{transition}' not in tvlqr gain set!")
                self.target_equilibrium = self.current_equilibrium

            else:
                self.transition = f"{self.current_equilibrium}_{self.target_equilibrium}"
                fsm.SetAtIndex(0, TRANSITION)
                # set the start time of the transition
                fsm.SetAtIndex(1, time)

        elif mode == TRANSITION:
            transition_time = time - mode_start_time
            trans_end_time = self.tvlqr_gains[self.transition][1].end_time()

            # check if the transition time has elapsed
            # TODO: add state check if necessary
            if transition_time >= trans_end_time:
                # transition finished
                self.current_equilibrium = self.target_equilibrium
                fsm.SetAtIndex(0, BALANCE)
                fsm.SetAtIndex(1, time)

        # if we get into a weird state, just balance for now
        else:
            fsm.SetAtIndex(0, BALANCE)

    def CalcOutput(self, context, output):

        fsm = context.get_discrete_state(self.fsm_state).get_value()
        mode = int(fsm[0])
        mode_start_time = fsm[1]
        x = self.state_port.Eval(context)

        # if the current mode is balance, then draw the output from lqr controller
        if mode == BALANCE:
            target_state = self.state_dict[self.current_equilibrium]
            lqr_K = self.lqr_gains[self.current_equilibrium]

            force = self.lqr_controller(target_state, x, lqr_K)

        # if current mode is transition, output from tvlqr controller
        elif mode == TRANSITION:
            current_time = context.get_time()
            transition_time = current_time - mode_start_time
            tvlqr_K = self.tvlqr_gains[self.transition]

            force = self.tvlqr_controller(x, tvlqr_K, transition_time)
        
        # else we are in an unknown state
        else:
            force = 0.0

        output.SetFromVector([force])

    def lqr_controller(self, target_state, x, K):

        u = -K @ (x - target_state)
        return u

    def tvlqr_controller(self, x, transition_trajectories, t):

        K_traj, x_traj, u_traj = transition_trajectories

        x_des = x_traj.value(t).flatten()
        u_des = u_traj.value(t)[0]
        K = K_traj.value(t)
        u = u_des - K @ (x - x_des)

        return u

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
