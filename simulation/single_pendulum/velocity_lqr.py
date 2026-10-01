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
    ExternallyAppliedSpatialForce,
)

import time
import sys
import os
import pickle
from pathlib import Path

from pydrake.systems.framework import LeafSystem_
from pydrake.systems.scalar_conversion import TemplateSystem

equilibriums = {
        "0" : [0, 0, 0, 0],
        "1" : [0, np.pi, 0, 0]
}


@TemplateSystem.define("VelocityController_")
def VelocityController_(T):

    class Impl(LeafSystem_[T]):

        # def _construct(self, converter=None, Kv=100.0):
        def _construct(self, Kv=100.0, converter=None):
            super().__init__(converter)

            self._Kv = Kv

            self._state_port = self.DeclareVectorInputPort(
                "state", 4
            )

            self._velocity_command_port = self.DeclareVectorInputPort(
                "velocity_command", 1
            )

            self.DeclareVectorOutputPort(
                "force",
                1,
                self.CalcOutput
            )

        def _construct_copy(self, other, converter=None):
            Impl._construct(
                self,
                converter=converter,
                Kv=other._Kv
            )

        def CalcOutput(self, context, output):

            x = self._state_port.Eval(context)
            v_cmd = self._velocity_command_port.Eval(context)[0]

            cart_velocity = x[2]

            force = self._Kv * (v_cmd - cart_velocity)

            output[0] = force

    return Impl


VelocityController = VelocityController_[None]


class LQRController(LeafSystem):
    def __init__(self, lqr_gain, target_state):
        super().__init__()

        self.lqr_gain = lqr_gain
        self.target_state = target_state

        self.state_port = self.DeclareVectorInputPort(
            name="estimated_state",
            size=4
        )

        self.DeclareVectorOutputPort(
            name="velocity_command",
            size=1,
            calc=self.CalcOutput
        )

    def CalcOutput(self, context, output):

        x = self.state_port.Eval(context)

        velocity_command = (
            -self.lqr_gain @ (x - self.target_state)
        )

        output.SetFromVector([velocity_command])


def get_velocity_controlled_diagram(file, Kv, export_velocity_command=False):

    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=0.0
    )

    Parser(plant).AddModels(file)

    # Physical damping
    shoulder = plant.GetJointByName("shoulder")
    shoulder.set_default_damping(2.54e-4)

    plant.Finalize()

    velocity_controller = builder.AddNamedSystem(
        "VelocityController",
        VelocityController(Kv)
    )

    # Controller -> plant
    builder.Connect(
        velocity_controller.get_output_port(0),
        plant.get_actuation_input_port()
    )

    # Plant state -> controller state input
    builder.Connect(
        plant.get_state_output_port(),
        velocity_controller.get_input_port(0)
    )

    # Expose v_cmd as an input to the whole diagram
    if export_velocity_command:
        builder.ExportInput(
            velocity_controller.get_input_port(1),
            "velocity_command"
        )

    # Expose plant state as the diagram output
    builder.ExportOutput(
        plant.get_state_output_port(),
        "state"
    )

    return plant, builder, scene_graph


def get_velocity_lqr(file, equilibrium, Kv):

    plant, builder, scene_graph = get_velocity_controlled_diagram(
            file,
            Kv,
            export_velocity_command=True
        )

    diagram = builder.Build()

    context = diagram.CreateDefaultContext()

    # Set equilibrium state
    plant_context = plant.GetMyContextFromRoot(context)

    shoulder = plant.GetJointByName("shoulder")
    shoulder.set_angle(
        plant_context,
        equilibrium[1]
    )

    # v_cmd = 0 at equilibrium
    diagram.GetInputPort("velocity_command").FixValue(
        context,
        [0.0]
    )

    # Linearise the COMPLETE diagram.
    #
    # Because VelocityController is a TemplateSystem, Drake can
    # automatically convert the diagram to AutoDiffXd here.
    linear_system = Linearize(
        diagram,
        context=context,
        input_port_index=diagram.GetInputPort(
            "velocity_command"
        ).get_index(),
        output_port_index=diagram.GetOutputPort(
            "state"
        ).get_index()
    )

    A = linear_system.A()
    B = linear_system.B()

    print("A:")
    print(A)

    print("\nB:")
    print(B)

    Q = np.diag([
        35,     # slider position
        50,     # angle of shoulder
        5,     # slider velocity
        10      # shoulder angular velocity
    ])

    R = np.array([[0.1]])

    K, S = LinearQuadraticRegulator(A, B, Q, R)

    return K


# class LQRController(LeafSystem):
#     def __init__(self, lqr_gain, target_state):
#         super().__init__()
#
#         self.lqr_gain = lqr_gain
#         self.target_state = target_state
#
#         self.state_port = self.DeclareVectorInputPort(
#             name="estimated_state",
#             size=4
#         )
#
#         self.DeclareVectorOutputPort(
#             name="velocity_command",
#             size=1,
#             calc=self.CalcOutput
#         )
#
#     def CalcOutput(self, context, output):
#
#         x = self.state_port.Eval(context)
#
#         velocity_command = (
#             -self.lqr_gain @ (x - self.target_state)
#         )
#
#         output.SetFromVector([velocity_command])

def simulate_velocity_pendulum(file, initial_state, lqr_gain, disturbance=0.0, Kv=100):

    plant, builder, scene_graph = get_velocity_controlled_diagram(
            file,
            Kv
        )

    controller = builder.AddNamedSystem("Controller",
                                        LQRController(lqr_gain, initial_state))

    builder.Connect(controller.get_output_port(0), 
                    builder.GetSubsystemByName("VelocityController").get_input_port(1))

    builder.Connect(plant.get_state_output_port(),
                    controller.get_input_port(0))

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

    Kv = 100
    # state = equilibriums["1"]
    #
    # lqr_gain = get_velocity_lqr("./single_pendulum.urdf", state, Kv)
    #
    # print("\nLQR Gain:\n", lqr_gain)
    #
    # simulate_velocity_pendulum("./single_pendulum.urdf", state,
    #                            lqr_gain, Kv=Kv, disturbance=0.1)

    lqr_gains = {}
    states = equilibriums.keys()

    for state in states:
        lqr_gain = get_velocity_lqr("./single_pendulum.urdf", equilibriums[state], Kv)
        lqr_gains[state] = lqr_gain


    print("\n\nLQR Gains:\n",lqr_gains)




