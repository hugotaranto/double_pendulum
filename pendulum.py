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
import pickle
from pathlib import Path
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

    # final state constraint
    idxs = params[TranscriptionParams.CONSTRAINT_IDXS.value]
    if idxs is None:
        dirtran.prog().AddBoundingBoxConstraint(final_state, final_state, dirtran.final_state())
    else:
        for i in idxs:
            dirtran.prog().AddBoundingBoxConstraint(final_state[i], final_state[i], dirtran.final_state()[i])

    # input "actuation" cost
    u = dirtran.input()[0]
    dirtran.AddRunningCost(params[TranscriptionParams.INPUT_COST.value] * u**2)

    # cost of rotational velocity of joints
    state = dirtran.state()
    dirtran.AddRunningCost(params[TranscriptionParams.SHOULDER_VELOCITY_COST.value] * state[4]**2)
    dirtran.AddRunningCost(params[TranscriptionParams.ELBOW_VELOCITY_COST.value] * state[5]**2)

    # load the keyframes in from file (I didn't end up using this)
    keyframe_file = params[TranscriptionParams.KEY_FRAME_FILE.value]
    if keyframe_file is not None:
        data = np.load(keyframe_file)
        keyframes = data["keyframes"].T
        times = np.linspace(0, target_time, len(data["keyframes"]))

    # if no initial guess, just make linear guess from initial -> goal states
    else:
        times = [0.0, target_time]
        keyframes = np.column_stack((initial_state, final_state))

    print("Keyframes:\n", keyframes)
    print("Times:", times)

    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
            times, keyframes
    )
    dirtran.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

    # compute the trajectory
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
        10,     # slider position
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

def simulate_full_system(lqr_gains, tvlqr_gains, initial_state="00"):

    while 1:
        plant, builder, scene_graph = get_diagram()

        # add the controller
        controller = builder.AddNamedSystem("Controller",
                                            SelectorController(initial_state, lqr_gains,
                                                               tvlqr_gains, STATE_DICT))

        # connect the plant to the controller
        builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())
        builder.Connect(plant.get_state_output_port(), controller.state_port)

        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

        diagram = builder.Build()
        simulator = Simulator(diagram)

        simulator.set_target_realtime_rate(1.0)
        context = simulator.get_mutable_context()

        plant_context = plant.GetMyContextFromRoot(context)
        init_vec = STATE_DICT[initial_state]

        # set the initial positions
        joint = plant.GetJointByName("shoulder")
        joint.set_angle(plant_context, init_vec[1])
        joint.set_angular_rate(plant_context, 0.0)

        joint = plant.GetJointByName("elbow")
        joint.set_angle(plant_context, init_vec[2])
        joint.set_angular_rate(plant_context, 0.0)

        joint = plant.GetJointByName("slider_joint")
        joint.set_translation(plant_context, 0.0)
        joint.set_translation_rate(plant_context, 0.0)

        # Add in control buttons
        meshcat.AddButton("00")
        meshcat.AddButton("01")
        meshcat.AddButton("10")
        meshcat.AddButton("11")
        meshcat.AddButton("Reset")
        meshcat.AddButton("Quit")
        last_00 = 0
        last_01 = 0
        last_10 = 0
        last_11 = 0
        last_reset = 0
        last_quit = 0

        dt = 0.2
        while 1:
            if meshcat.GetButtonClicks("Quit") > last_quit:
                return
            elif meshcat.GetButtonClicks("Reset") > last_reset:
                break
            elif meshcat.GetButtonClicks("00") > last_00:
                controller.SetTargetState("00")
                last_00 += 1
            elif meshcat.GetButtonClicks("01") > last_01:
                controller.SetTargetState("01")
                last_01 += 1
            elif meshcat.GetButtonClicks("10") > last_10:
                controller.SetTargetState("10")
                last_10 += 1
            elif meshcat.GetButtonClicks("11") > last_11:
                controller.SetTargetState("11")
                last_11 += 1

            simulator.AdvanceTo(simulator.get_context().get_time() + dt)


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

    plant = get_plant()

    transcription_params = TRANSCRIPTION_PARAMS[transition]
    states = transition.split("_")
    initial_state = STATE_DICT[states[0]]
    goal_state = STATE_DICT[states[1]]

    lqr_gains = load_pickle(LQR_FILE)

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

        tvlqr_K = tvlqr(x_trajectory, u_trajectory, plant)
        animate_tvlqr(tvlqr_K, x_traj=x_trajectory, u_traj=u_trajectory, 
                      initial_state=initial_state, speed=0.2)

        text = input("Do full animation? (y/n): ")

        if text == "n":
            continue

        transition_set = {}
        transition_set[transition] = (tvlqr_K, x_trajectory, u_trajectory)

        simulate_full_system(lqr_gains, transition_set, states[0])

def test_full_system(transition):

    plant = get_plant()

    transcription_params = TRANSCRIPTION_PARAMS[transition]
    states = transition.split("_")
    initial_state = STATE_DICT[states[0]]
    goal_state = STATE_DICT[states[1]]

    x_trajectory, u_trajectory = direct_transcription(plant,
                                                      initial_state,
                                                      goal_state,
                                                      transcription_params)

    time_cutoff = transcription_params[TranscriptionParams.TIME_CUTOFF.value]

    if time_cutoff is not None:
        x_trajectory, u_trajectory = time_cut(x_trajectory, u_trajectory,
                                              time_cutoff)

    tvlqr_K = tvlqr(x_trajectory, u_trajectory, plant)

    transition_set = {}
    transition_set[transition] = (tvlqr_K, x_trajectory, u_trajectory)

    lqr_gains = load_pickle(LQR_FILE)
    simulate_full_system(lqr_gains, transition_set, states[0])

def compute_lqrs(plant, states, path=None):
    gains = {}
    for state_name in states:
        state = STATE_DICT[state_name]

        lqr_K = lqr(plant, state)

        gains[state_name] = lqr_K

    if path is not None:
        save_pickle(gains, path)

    return gains

def compute_tvlqrs(plant, transitions, path=None):
    gains = {}
    count = 0
    for transition in transitions:
        transcription_params = TRANSCRIPTION_PARAMS[transition]
        states = transition.split("_")
        initial_state = STATE_DICT[states[0]]
        goal_state = STATE_DICT[states[1]]
        count += 1

        print(f"Computing Transition: {transition} {count}/{len(transitions)}")

        x_trajectory, u_trajectory = direct_transcription(plant,
                                                          initial_state,
                                                          goal_state,
                                                          transcription_params)

        time_cutoff = transcription_params[TranscriptionParams.TIME_CUTOFF.value]

        if time_cutoff is not None:
            x_trajectory, u_trajectory = time_cut(x_trajectory, u_trajectory,
                                                  time_cutoff)

        tvlqr_K = tvlqr(x_trajectory, u_trajectory, plant)

        gains[transition] = (tvlqr_K, x_trajectory, u_trajectory)

    if path is not None:
        save_pickle(gains, path)

    return gains

def save_pickle(obj, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("wb") as f:
        pickle.dump(obj, f)

def load_pickle(path):
    path = Path(path)
    assert path.exists()

    with path.open("rb") as f:
        data = pickle.load(f)

    return data

if __name__ == "__main__":

    # start the meshcat server and wait until there is a connection
    meshcat = StartMeshcat()
    while meshcat.GetNumActiveConnections() == 0:
        print(meshcat.GetNumActiveConnections(), "Connections")
        time.sleep(1)

    time.sleep(1)
    print("Simulating...")
    # plant = get_plant()

    # states = STATE_DICT.keys()
    # lqrs = compute_lqrs(plant, states, LQR_FILE)
    #
    # transitions = TRANSCRIPTION_PARAMS.keys()
    # tvlqrs = compute_tvlqrs(plant, transitions, TVLQR_FILE)


    # -=-=-=-=-= simulate the full system -=-=-=-=-=

    # first load the gains
    # lqr_gains = load_pickle(LQR_FILE)
    # tvlqr_gains = load_pickle(TVLQR_FILE)
    #
    # # then simulate!
    # simulate_full_system(lqr_gains, tvlqr_gains, initial_state="00")


    # -=-=-=-=-=-=- Test single transition (Used for creating trajectories) -=-=-=-=-=-=-

    test_trajectories("10_11")
