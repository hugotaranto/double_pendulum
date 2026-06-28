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

def animate_trajectory(x_trajectory, speed=1.0):

    plant, builder, scene_graph = get_diagram()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    t0 = x_trajectory.start_time()
    tf = x_trajectory.end_time()

    # animate to a real speed of 1s = 1s (+ small calculation time)
    playback_fps = 30

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


def trajectory_cleanup(x_trajectory, u_trajectory, goal_state, threshold, num_samples, idxs=[1, 2, 4, 5]):

    goal_state = np.asarray(goal_state)

    # Sample trajectory densely
    times = np.linspace(x_trajectory.start_time(),
                         x_trajectory.end_time(), num_samples)

    X = np.column_stack([x_trajectory.value(t).flatten() for t in times])
    U = np.column_stack([u_trajectory.value(t).flatten() for t in times])

    # Find last index where NOT close to goal
    last_bad_idx = 0

    for i in reversed(range(num_samples)):
        error = np.abs(X[idxs, i] - goal_state[idxs])

        if np.any(error > threshold):
            last_bad_idx = i
            break

    # Keep everything up to that point (+ a small buffer)
    last_idx = min(last_bad_idx + 10, num_samples - 1)

    times_cut = times[:last_idx + 1]
    X_cut = X[:, :last_idx + 1]
    U_cut = U[:, :last_idx + 1]

    # Rebuild trajectories
    x_trimmed = PiecewisePolynomial.FirstOrderHold(times_cut, X_cut)
    u_trimmed = PiecewisePolynomial.FirstOrderHold(times_cut, U_cut)

    return x_trimmed, u_trimmed

def direct_transcription(plant, initial_state, final_state, params):

    context = plant.CreateDefaultContext()

    target_time = params[TranscriptionParams.TARGET_TIME.value]
    num_time_steps = params[TranscriptionParams.NUM_TIME_STEPS.value]

    time_step = target_time / num_time_steps

    dirtran = DirectTranscription(
            plant,
            context,
            num_time_samples=num_time_steps,
            fixed_time_step=DirectTranscription.TimeStep(time_step),
            input_port_index=plant.get_actuation_input_port().get_index()
    )

    dirtran.prog().AddBoundingBoxConstraint(initial_state, initial_state,
                                dirtran.initial_state())

    # slider position constraint (DON'T GO OFF THE EDGE!)
    dirtran.AddConstraintToAllKnotPoints(dirtran.state()[0] <= 0.4)
    dirtran.AddConstraintToAllKnotPoints(dirtran.state()[0] >= -0.4)

    # final slider position
    final_pose_contraint = params[TranscriptionParams.FINISH_POSE.value]
    if final_pose_contraint is not None:
        dirtran.prog().AddBoundingBoxConstraint(-final_pose_contraint,
                                                final_pose_contraint,
                                                dirtran.final_state()[0])

    # actuation limits
    dirtran.AddConstraintToAllKnotPoints(sym.abs(dirtran.input()[0]) <= params[TranscriptionParams.ACTUATION_CONSTRAINT.value])

    idxs = params[TranscriptionParams.CONSTRAINT_IDXS.value]
    if idxs is None:
        dirtran.prog().AddBoundingBoxConstraint(final_state, final_state, dirtran.final_state())
    else:
        for i in idxs:
            dirtran.prog().AddBoundingBoxConstraint(final_state[i], final_state[i], dirtran.final_state()[i])

    u = dirtran.input()[0]
    dirtran.AddRunningCost(params[TranscriptionParams.INPUT_COST.value] * u**2)

    # slow down the swing speed
    state = dirtran.state()
    dirtran.AddRunningCost(params[TranscriptionParams.SHOULDER_VELOCITY_COST.value] * state[4]**2)
    dirtran.AddRunningCost(params[TranscriptionParams.ELBOW_VELOCITY_COST.value] * state[5]**2)

    # dirtran.AddRunningCost(1.0 * sym.abs(final_state[2] - state[2])**2)
    # dirtran.AddRunningCost(2.0 * state[2]**2)

    keyframe_file = params[TranscriptionParams.KEY_FRAME_FILE.value]

    if keyframe_file is not None:
        data = np.load(keyframe_file)
        keyframes = data["keyframes"].T
        times = np.linspace(0, target_time, len(data["keyframes"]))
    else:
        times = [0.0, target_time]
        keyframes = np.column_stack((initial_state, final_state))

    print("Keyframes:\n", keyframes)
    print("Times:", times)

    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
            times, keyframes
    )

    dirtran.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

    result = Solve(dirtran.prog())
    assert result.is_success()

    u_trajectory = dirtran.ReconstructInputTrajectory(result)
    x_trajectory = dirtran.ReconstructStateTrajectory(result)

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


def animate_tvlqr(K, x_traj, u_traj, initial_state, speed=1.0):

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

    simulator.set_target_realtime_rate(speed)

    # uncomment these to set initial state
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)

    # set the initial positions
    joint = plant.GetJointByName("shoulder")
    joint.set_angle(plant_context, initial_state[1])
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("elbow")
    joint.set_angle(plant_context, initial_state[2])
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("slider_joint")
    joint.set_translation(plant_context, 0.0)
    joint.set_translation_rate(plant_context, 0.0)

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


def animate_full_system(lqr_K, tvlqr_K, x_trajectory, u_trajectory, target_state, initial_state):

    plant, builder, scene_graph = get_diagram()

    # add the selector controller
    selector_controller = builder.AddNamedSystem("selector_controller", SelectorController())

    # add the lqr controller
    lqr_controller = builder.AddNamedSystem("lqr_controller", LQRController(lqr_K))

    # add the tvlqr controller
    tvlqr_controller = builder.AddNamedSystem("tvlqr_controller", 
                                              TVLQRController(K_traj=tvlqr_K, x_traj=x_trajectory, u_traj=u_trajectory))

    swing_time = builder.AddSystem(ConstantVectorSource([x_trajectory.end_time()]))
    
    # connect each controller to the selector
    builder.Connect(lqr_controller.get_output_port(), selector_controller.up_up_balance)
    builder.Connect(tvlqr_controller.get_output_port(), selector_controller.swing_up)
    builder.Connect(swing_time.get_output_port(), selector_controller.swing_time)

    # connect the state of the plant to each controller
    builder.Connect(plant.get_state_output_port(), selector_controller.state_port)
    builder.Connect(plant.get_state_output_port(), lqr_controller.state_port)
    builder.Connect(plant.get_state_output_port(), tvlqr_controller.state_port)

    # set the desired state for the lqr controller
    desired_state = builder.AddSystem(ConstantVectorSource(target_state))
    builder.Connect(desired_state.get_output_port(), lqr_controller.target_state_port)
    builder.Connect(desired_state.get_output_port(), selector_controller.target)

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
    joint.set_angle(plant_context, initial_state[1])
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("elbow")
    joint.set_angle(plant_context, initial_state[2])
    joint.set_angular_rate(plant_context, 0.0)

    joint = plant.GetJointByName("slider_joint")
    joint.set_translation(plant_context, 0.0)
    joint.set_translation_rate(plant_context, 0.0)

    simulator.AdvanceTo(10.0)


def time_cut(x_traj, u_traj, time_cut, num_samples=300):
    times = np.linspace(0, time_cut, num_samples)

    states = []
    inputs = []

    for time in times:
        states.append(x_traj.value(time))
        inputs.append(u_traj.value(time))

    x_trajectory = PiecewisePolynomial.FirstOrderHold(times, states)
    u_trajectory = PiecewisePolynomial.FirstOrderHold(times, inputs)

    return x_trajectory, u_trajectory

def test_trajectories(transition):

    transcription_params = TRANSCRIPTION_PARAMS[transition]
    states = transition.split("_")
    initial_state = STATE_DICT[states[0]]
    goal_state = STATE_DICT[states[1]]

    # test direct transcription
    x_traj, u_traj = direct_transcription(plant,
                                          initial_state=initial_state,
                                          final_state=goal_state,
                                          params=transcription_params)

    animate_trajectory(x_traj, speed=0.5)

    while(1):

        thresh = input("Threshold input: ")
        try:
            thresh = float(thresh)
        except:
            continue

        x_trajectory, u_trajectory = time_cut(x_traj, u_traj, thresh)

        animate_trajectory(x_trajectory, speed=0.5)

        text = input("Go again? (y/n): ")

        if text == "y":
            continue

        lqr_K = lqr(plant, goal_state)

        tvlqr_K = tvlqr(x_trajectory, u_trajectory, plant)
        animate_tvlqr(tvlqr_K, x_traj=x_trajectory, u_traj=u_trajectory, 
                      initial_state=initial_state, speed=0.2)

        text = input("Do full animation? (y/n): ")

        if text == "n":
            continue

        animate_full_system(lqr_K=lqr_K, tvlqr_K=tvlqr_K, x_trajectory=x_trajectory,
                            u_trajectory=u_trajectory, target_state=goal_state, initial_state=initial_state)

def test_full_system(transition):

    transcription_params = TRANSCRIPTION_PARAMS[transition]
    states = transition.split("_")
    initial_state = STATE_DICT[states[0]]
    goal_state = STATE_DICT[states[1]]

    lqr_K = lqr(plant, goal_state)    
    x_trajectory, u_trajectory = direct_transcription(plant,
                                                      initial_state,
                                                      goal_state,
                                                      transcription_params)

    x_trajectory, u_trajectory = time_cut(x_trajectory, u_trajectory,
                                          transcription_params[TranscriptionParams.TIME_CUTOFF.value])

    tvlqr_K = tvlqr(x_trajectory, u_trajectory, plant)
    animate_full_system(lqr_K, tvlqr_K, x_trajectory, u_trajectory, goal_state, initial_state)

if __name__ == "__main__":

    # start the meshcat server and wait until there is a connection
    meshcat = StartMeshcat()
    while meshcat.GetNumActiveConnections() == 0:
        print(meshcat.GetNumActiveConnections(), "Connections")
        time.sleep(1)

    time.sleep(1)
    print("Simulating...")
    plant = get_plant()

    transition = "10_00"
    test_trajectories(transition)

    # transitions = TRANSCRIPTION_PARAMS.keys()
    # for transition in transitions:
    #     print("-=-=-=-=-=-=-=-=-=-=-=-=- Testing Transition:", transition, "-=-=-=-=-=-=-\n")
    #     test_full_system(transition)


