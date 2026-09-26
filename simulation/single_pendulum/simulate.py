import numpy as np

import pydrake.symbolic as sym
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    MeshcatVisualizer,
Parser,
    Simulator,
    StartMeshcat,
    ConstantVectorSource,
    Linearize,
    LinearQuadraticRegulator,
    ControllabilityMatrix,
    PiecewisePolynomial,
    Solve,
    FiniteHorizonLinearQuadraticRegulator,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    MultibodyPlant,
    DirectTranscription,
    LeafSystem,
)

import time
import sys
import os
import pickle
from pathlib import Path

class LQRController(LeafSystem):
    def __init__(self, lqr_gain, target_state):
        super().__init__()

        self.lqr_gain = lqr_gain
        self.target_state = target_state

        self.state_port = self.DeclareVectorInputPort(name="estimated_state", size=4)

        self.DeclareVectorOutputPort(name="actuation", size=1, calc=self.CalcOutput)

    def CalcOutput(self, context, output):

        x = self.state_port.Eval(context)

        u = -self.lqr_gain @ (x - self.target_state)

        output.SetFromVector([u])


# Find the absolute path to the root 'sliding_double_pendulum' directory
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# Define the specific directories that need to be searchable
simulation_dir = os.path.join(project_root, 'simulation')
double_pend_dir = os.path.join(simulation_dir, 'double_pendulum')
single_pend_dir = os.path.dirname(os.path.abspath(__file__))

# Inject them all at the front of Python's search path
for path in [project_root, simulation_dir, double_pend_dir, single_pend_dir]:
    if path not in sys.path:
        sys.path.insert(0, path)

from double_pendulum.pendulum import get_plant, get_diagram

equilibriums = {
        "0" : [0, 0, 0, 0],
        "1" : [0, np.pi, 0, 0]
}

# state space is:
#[cart pose, shoulder angle, cart vel, shoulder angular vel]

def get_single_lqr(plant, state, print_detail=False):

    shoulder_angle = state[1]

    context = plant.CreateDefaultContext()

    # set the equilibrium point
    joint = plant.GetJointByName("shoulder")
    joint.set_angle(context, shoulder_angle)

    # set the actuation input port to 0
    plant.get_actuation_input_port().FixValue(context, [0.0])

    # linearise the system at this equilibrium point
    linear_system = Linearize(
            system=plant,
            context=context,
            input_port_index=plant.get_actuation_input_port().get_index(),
            output_port_index=plant.get_state_output_port().get_index()
    )

    
    if print_detail:
        Wc = ControllabilityMatrix(linear_system)
        print("Rank:")
        print(np.linalg.matrix_rank(Wc))

        U, S, Vt = np.linalg.svd(Wc)
        print("\nS from svd:")
        print(S)

        eigvals = np.linalg.eigvals(linear_system.A())
        print("\nEigvals of A:")
        print(eigvals)

    Q = np.diag([
        20,     # slider position
        100,    # angle of shoulder
        1,      # slider velocity
        10      # shoulder angular velocity
    ])

    R = np.array([[0.5]])

    K, S = LinearQuadraticRegulator(
            linear_system.A(),
            linear_system.B(),
            Q,
            R
    )

    return K

def simulate_single_pendulum(initial_state, lqr_gain, disturbance=0.0):

    plant, builder, scene_graph = get_diagram(file="./single_pendulum.urdf")

    controller = builder.AddNamedSystem("Controller",
                                        LQRController(lqr_gain, initial_state))

    # connect the plant to the controller
    builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(), controller.state_port)

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)

    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()

    plant_context = plant.GetMyContextFromRoot(context)

    # set the initial positions
    joint = plant.GetJointByName("shoulder")
    joint.set_angle(plant_context, initial_state[1] + disturbance)

    # everything else should be 0

    simulator.AdvanceTo(20.0)

if __name__ == "__main__":
    
    # start the meshcat server
    meshcat = StartMeshcat()
    while meshcat.GetNumActiveConnections() == 0:
        print(meshcat.GetNumActiveConnections(), "Connections")
        time.sleep(1)

    print("Simulating...")

    plant = get_plant(file="./single_pendulum.urdf")

    # get the lqr for each equilibrium
    states = equilibriums.keys()

    gains = {}

    for state in states:
        print("Pendulum state:", state)
        gain = get_single_lqr(plant, equilibriums[state], print_detail=True)

        gains[state] = gain

    # simulate the system
    up_state = equilibriums["1"]
    up_gain = gains["1"]

    simulate_single_pendulum(up_state, up_gain, disturbance=0.1)

    
