import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LogVectorOutput,
    MeshcatVisualizer,
    Parser,
    Simulator,
    StartMeshcat,
    ConstantVectorSource,
    Linearize,
    LinearQuadraticRegulator,
    ControllabilityMatrix,
    DirectCollocation,
    PiecewisePolynomial,
    Solve,
    FiniteHorizonLinearQuadraticRegulator,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    MultibodyPlant,
)

import time
import sys
from controllers import *

def get_plant(file="./sliding_double_pendulum.urdf"):
    plant = MultibodyPlant(time_step=0.0)
    Parser(plant).AddModels(file)
    plant.Finalize()

    return plant

def get_diagram(file="./sliding_double_pendulum.urdf"):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels(file)
    plant.Finalize()

    return plant, builder, scene_graph

def animate_trajectory(x_trajectory):

    plant, builder, scene_graph = get_diagram()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    t0 = x_trajectory.start_time()
    tf = x_trajectory.end_time()

    # animate to a real speed of 1s = 1s (+ small calculation time)
    playback_fps = 30
    speed = 0.2

    fps = playback_fps / speed
    dt_play = 1 / playback_fps    # seconds between frames
    dt_sim = 1 / fps
    sim_time = t0

    print("")
    while sim_time <= tf:
        sys.stdout.write(f"Time: {sim_time:.1f}\r")

        x = x_trajectory.value(sim_time).flatten()

        plant.SetPositionsAndVelocities(plant_context, x)

        diagram.ForcedPublish(context)

        time.sleep(dt_play)
        sim_time += dt_sim

def collocation_copy(plant, initial_state, final_state, x_guess, u_guess, time_cut=None):

    if time_cut is not None:
        times = np.linspace(x_guess.start_time(), time_cut, 100)

        X = np.column_stack([
            x_guess.value(t).flatten()
            for t in times
        ])

        X = np.column_stack((
            X, final_state
        ))

        U = np.column_stack([
            u_guess.value(t).flatten()
            for t in times
        ])

        U = np.column_stack((
            U, [0]
        ))

        # times = list(times).append(time_cut + 1.0)
        times = list(times)
        times.append(time_cut + 1.0)

        x_guess = PiecewisePolynomial.FirstOrderHold(times, X)
        u_guess = PiecewisePolynomial.FirstOrderHold(times, U)       

    context = plant.CreateDefaultContext()
    dircol = DirectCollocation(
            plant,
            context,
            num_time_samples=40,
            minimum_time_step=0.05,
            maximum_time_step=0.5,
            input_port_index=plant.get_actuation_input_port().get_index()
    )

    dircol.AddEqualTimeIntervalsConstraints()

    dircol.prog().AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())

    dircol.prog().AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())

    # slider position constraint (DON'T GO OFF THE EDGE!)
    # dircol.AddConstraintToAllKnotPoints(dircol.state()[0] <= 1.0)
    # dircol.AddConstraintToAllKnotPoints(dircol.state()[0] >= -1.0)
    #
    # # actuation limits
    # dircol.AddConstraintToAllKnotPoints(dircol.input()[0] <= 10)
    # dircol.AddConstraintToAllKnotPoints(dircol.input()[0] >= -10)

    # print("Keyframes:\n", keyframes)
    # print("Times:", times)

    # initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
    #         times, keyframes
    # )

    u = dircol.input()[0]
    dircol.AddRunningCost(u**2)

    dircol.AddFinalCost(dircol.time())

    dircol.SetInitialTrajectory(u_guess, x_guess)

    result = Solve(dircol.prog())
    assert result.is_success()

    u_trajectory = dircol.ReconstructInputTrajectory(result)
    x_trajectory = dircol.ReconstructStateTrajectory(result)

    print(f"Solution found in {x_trajectory.end_time()} seconds")
    
    return x_trajectory, u_trajectory


def collocation_trajectory(plant, initial_state=STATE_DICT["00"],
                           final_state=STATE_DICT["11"], keyframe_file=None):

    context = plant.CreateDefaultContext()
    dircol = DirectCollocation(
            plant,
            context,
            num_time_samples=60,
            minimum_time_step=0.05,
            maximum_time_step=0.5,
            input_port_index=plant.get_actuation_input_port().get_index()
    )

    dircol.AddEqualTimeIntervalsConstraints()

    dircol.prog().AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())

    dircol.prog().AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())

    # slider position constraint (DON'T GO OFF THE EDGE!)
    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] <= 0.5)
    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] >= -0.5)

    # actuation limits
    dircol.AddConstraintToAllKnotPoints(dircol.input()[0] <= 10)
    dircol.AddConstraintToAllKnotPoints(dircol.input()[0] >= -10)

    # dircol.AddConstraintToAllKnotPoints(dircol.state()[5] <= 1000)
    # dircol.AddConstraintToAllKnotPoints(dircol.state()[5] >= -1000)


    if keyframe_file is not None:
        # Load the initial trajectory from the keyframe generation
        data = np.load(keyframe_file)
        keyframes = data["keyframes"].T
        # times = data["times"]
        times = [0, 0.5, 1, 1.7, 2.4]
    else:
        times = [0.0, 4.0]
        keyframes = np.column_stack((initial_state, final_state))

        # dircol.AddRunningCost(dircol.state()[1]**2)
        # dircol.AddRunningCost(dircol.state()[2]**2)

        # u = dircol.input()[0]
        # dircol.AddRunningCost(1 * u**2)
        #
        # x = dircol.state()
        # dircol.AddRunningCost(0.001 * (x[1]**2 + x[2]**2))

        # x = dircol.state()
        # dircol.AddRunningCost(1 * x[5]**2)

        # dircol.AddFinalCost(dircol.time())

    print("Keyframes:\n", keyframes)
    print("Times:", times)

    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
            times, keyframes
    )

    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

    result = Solve(dircol.prog())
    assert result.is_success()

    u_trajectory = dircol.ReconstructInputTrajectory(result)
    x_trajectory = dircol.ReconstructStateTrajectory(result)

    print(f"Solution found in {x_trajectory.end_time()} seconds")
    
    return x_trajectory, u_trajectory

def lqr(plant, state, print_detail=False):

    shoulder_angle = state[1]
    elbow_angle = state[2]

    context = plant.CreateDefaultContext()

    # set the equilibrium point
    joint = plant.GetJointByName("shoulder")
    joint.set_angle(context, shoulder_angle)

    joint = plant.GetJointByName("elbow")
    joint.set_angle(context, elbow_angle)

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

    # ['sliding_pendulum_slider_joint_x', 'sliding_pendulum_shoulder_q', 'sliding_pendulum_elbow_q', 
    #  'sliding_pendulum_slider_joint_v', 'sliding_pendulum_shoulder_w', 'sliding_pendulum_elbow_w']

    # define Q: the cost for each of the states
    Q = np.diag([
        20,     # slider position
        100,    # angle of first link
        100,    # angle of second link
        1,      # velocity of slider
        10,     # shoulder angular velocity
        10,     # elbow angular velocity
    ])

    # define R: the cost of actuation
    R = np.array([[0.5]])

    K, S = LinearQuadraticRegulator(
            linear_system.A(),
            linear_system.B(),
            Q,
            R
    )

    return K

def animate_lqr(K, state, disturbance=0.1):
    shoulder_angle = state[1]
    elbow_angle = state[2]

    plant, builder, scene_graph = get_diagram()

    # add the controller
    lqr_controller = builder.AddNamedSystem("lqr_controller", LQRController(K))

    # connect the controller to the plant
    builder.Connect(plant.get_state_output_port(),
                    lqr_controller.state_port)

    builder.Connect(lqr_controller.get_output_port(),
                    plant.get_actuation_input_port())

    # set the desired state for the controller
    desired_state = builder.AddSystem(ConstantVectorSource([0, shoulder_angle, elbow_angle, 0, 0, 0]))
    builder.Connect(desired_state.get_output_port(), lqr_controller.target_state_port)

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)

    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()

    plant_context = plant.GetMyContextFromRoot(context)

    # set the initial positions
    joint = plant.GetJointByName("shoulder")
    joint.set_angle(plant_context, shoulder_angle + disturbance)
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("elbow")
    joint.set_angle(plant_context, elbow_angle - disturbance)
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("slider_joint")
    joint.set_translation(plant_context, 0.0)
    joint.set_translation_rate(plant_context, 0.0)

    simulator.AdvanceTo(7.0)

def double_pendulum():

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels("./sliding_double_pendulum.urdf")
    plant.Finalize()

    builder.ExportInput(plant.get_actuation_input_port())
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    logger = LogVectorOutput(plant.get_state_output_port(), builder)

    diagram = builder.Build()
    simulator = Simulator(diagram)

    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()

    plant_context = plant.GetMyContextFromRoot(context)

    # set the initial positions
    joint = plant.GetJointByName("shoulder")
    joint.set_angle(plant_context, np.pi)
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("elbow")
    joint.set_angle(plant_context, np.pi)
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("slider_joint")
    joint.set_translation(plant_context, 0.0)
    joint.set_translation_rate(plant_context, 0.0)

    # torque input for the sliding joint
    diagram.get_input_port(0).FixValue(context, [0.0])    

    simulator.AdvanceTo(10.0)

def animate_tvlqr(K, x_traj, u_traj):

    plant, builder, scene_graph = get_diagram()
    # add the tvlqr controller

    tvlqr_controller = builder.AddNamedSystem(
            "tvlqr_controller",
            TVLQRController(K_traj=K, x_traj=x_traj, u_traj=u_traj))

    # connect the controller to the plant
    builder.Connect(plant.get_state_output_port(), tvlqr_controller.state_port)
    builder.Connect(tvlqr_controller.get_output_port(), plant.get_actuation_input_port())

    start_time = builder.AddSystem(ConstantVectorSource([0.0]))
    builder.Connect(start_time.get_output_port(), tvlqr_controller.start_time)

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)

    simulator.set_target_realtime_rate(1.0)

    # uncomment these to set initial state
    # context = simulator.get_mutable_context()
    # plant_context = plant.GetMyContextFromRoot(context)

    simulator.AdvanceTo(x_traj.end_time())


def tvlqr(x_trajectory, u_trajectory, plant):

    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.x0 = x_trajectory
    options.u0 = u_trajectory
    options.input_port_index = plant.get_actuation_input_port().get_index()

    context = plant.CreateDefaultContext()

    # define Q: the cost for each of the states
    Q = np.diag([
        10,      # slider position
        100,    # angle of first link
        100,    # angle of second link
        1,      # velocity of slider
        10,     # shoulder angular velocity
        10,     # elbow angular velocity
    ])

    # define R: the cost of actuation
    R = np.array([[1]])

    t0 = x_trajectory.start_time()
    tf = x_trajectory.end_time()

    tvlqr = FiniteHorizonLinearQuadraticRegulator(
            plant,
            context,
            t0,
            tf,
            Q,
            R,
            options
    )

    return tvlqr.K


def animate_full_system(lqr_K, tvlqr_K, x_trajectory, u_trajectory):

    plant, builder, scene_graph = get_diagram()

    # add the selector controller
    selector_controller = builder.AddNamedSystem("selector_controller", SelectorController())

    # add the lqr controller
    lqr_controller = builder.AddNamedSystem("lqr_controller", LQRController(lqr_K))

    # add the tvlqr controller
    tvlqr_controller = builder.AddNamedSystem("tvlqr_controller", 
                                              TVLQRController(K_traj=tvlqr_K, x_traj=x_trajectory, u_traj=u_trajectory))

    # connect each controller to the selector
    builder.Connect(lqr_controller.get_output_port(), selector_controller.up_up_balance)
    builder.Connect(tvlqr_controller.get_output_port(), selector_controller.swing_up)

    # connect the state of the plant to each controller
    builder.Connect(plant.get_state_output_port(), selector_controller.state_port)
    builder.Connect(plant.get_state_output_port(), lqr_controller.state_port)
    builder.Connect(plant.get_state_output_port(), tvlqr_controller.state_port)

    # set the desired state for the lqr controller
    desired_state = builder.AddSystem(ConstantVectorSource([0, np.pi, 0, 0, 0, 0]))
    builder.Connect(desired_state.get_output_port(), lqr_controller.target_state_port)

    # connect the selector to the actuator
    builder.Connect(selector_controller.get_output_port(0), plant.get_actuation_input_port())
    builder.Connect(selector_controller.get_output_port(1), tvlqr_controller.start_time)

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)

    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()

    plant_context = plant.GetMyContextFromRoot(context)

    # set the initial positions
    joint = plant.GetJointByName("shoulder")
    joint.set_angle(plant_context, 0.0)
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("elbow")
    joint.set_angle(plant_context, 0.0)
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("slider_joint")
    joint.set_translation(plant_context, 0.0)
    joint.set_translation_rate(plant_context, 0.0)

    simulator.AdvanceTo(20.0)

if __name__ == "__main__":

    # start the meshcat server and wait until there is a connection
    meshcat = StartMeshcat()
    while meshcat.GetNumActiveConnections() == 0:
        print(meshcat.GetNumActiveConnections(), "Connections")
        time.sleep(1)

    time.sleep(1)
    print("Simulating...")
    plant = get_plant()

    # for key in STATE_DICT.keys():
    #     lqr_K = lqr(plant, STATE_DICT[key])
    #     animate_lqr(K=lqr_K, state=STATE_DICT[key], disturbance=0.1)

    # lqr_K = lqr(plant, STATE_DICT["11"], print_detail=True)
    # animate_lqr(K=lqr_K, state=STATE_DICT["01"], disturbance=0.01)

    # x_trajectory, u_trajectory = collocation_trajectory(plant,
    #                                                     initial_state=STATE_DICT["10"],
    #                                                     final_state=STATE_DICT["11"],
    #                                                     keyframe_file=None)

    # x_trajectory, u_trajectory = collocation_trajectory(plant,
    #                                                     initial_state=STATE_DICT["00"],
    #                                                     final_state=STATE_DICT["11"],
    #                                                     keyframe_file="./keyframes/swing_up_2.npz")

    x_trajectory, u_trajectory = collocation_trajectory(plant,
                                                        initial_state=STATE_DICT["00"],
                                                        final_state=STATE_DICT["11"],
                                                        keyframe_file="./keyframes/00_11_1.npz")

    times = np.linspace(
            x_trajectory.start_time(),
            x_trajectory.end_time(),
            200
    )

    thresh = 1.5

    for t in times:
        x = x_trajectory.value(t).flatten()

        if (abs(x[1] - np.pi) < thresh and abs(x[2] - np.pi) < thresh):
            print(x)

    print("Done...")

    # x_trajectory, u_trajectory = collocation_trajectory_iteration(plant, STATE_DICT["00"], STATE_DICT["10"])

    # x_trajectory, u_trajectory = collocation_trajectory_soft_goal(plant, initial_state=STATE_DICT["00"],
    #                                                               goal_state=STATE_DICT["11"])

    x_trajectory, u_trajectory = collocation_copy(plant, STATE_DICT["00"], STATE_DICT["10"],
                                                  x_guess=x_trajectory, u_guess=u_trajectory, time_cut=2.0)

    animate_trajectory(x_trajectory)

    tvlqr_K = tvlqr(x_trajectory, u_trajectory, plant)
    animate_tvlqr(tvlqr_K, x_traj=x_trajectory, u_traj=u_trajectory)

    # animate_full_system(lqr_K=lqr_K, tvlqr_K=tvlqr_K, x_trajectory=x_trajectory, u_trajectory=u_trajectory)

