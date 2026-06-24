import numpy as np
from pydrake.all import (
    LeafSystem
)

class SelectorController(LeafSystem):
    def __init__(self):
        super().__init__()

        self.mode = "swing_up"      # initialise with swing up mode

        self.state_port = self.DeclareVectorInputPort(name="estimated_state", size=6)
        self.swing_up = self.DeclareVectorInputPort(name="swing_up", size=1)
        self.up_up_balance = self.DeclareVectorInputPort(name="up_up_balance", size=1)

        self.DeclareVectorOutputPort(name="force",
                                     size=1,
                                     calc=self.CalcOutput)

    def CalcOutput(self, context, output):

        if self.mode == "swing_up":
            x = self.state_port.Eval(context)

            # check if the state is close to the balance point
            if(
                abs(abs(x[1]) - np.pi) < 0.1 and
                abs(abs(x[2]) - 0.0) < 0.1 and
                abs(x[4]) < 0.5 and
                abs(x[5]) < 0.5):

                self.mode = "balance"
                print("Switching to balancing mode!")

        if self.mode == "swing_up":
            output.SetFromVector(self.swing_up.Eval(context))
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
        t = context.get_time()

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
