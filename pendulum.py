import numpy as np
import pydrake.symbolic as sym
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
    LeafSystem,
    ControllabilityMatrix,
    DirectCollocation,
    PiecewisePolynomial,
    Solve,
    FiniteHorizonLinearQuadraticRegulator,
    FiniteHorizonLinearQuadraticRegulatorOptions,
)

import time

# start the meshcat server and wait until there is a connection
meshcat = StartMeshcat()
while meshcat.GetNumActiveConnections() == 0:
    print(meshcat.GetNumActiveConnections(), "Connections")
    time.sleep(1)

time.sleep(1)

print("Simulating...")

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

def smooth_barrier(x, limit=0.5, alpha=20.0):
    # softplus-like barrier
    return np.log(1 + np.exp(alpha * (np.abs(x) - limit))) / alpha

def collocation_trajectory(animate=False):

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels("./sliding_double_pendulum.urdf")
    plant.Finalize()

    context = plant.CreateDefaultContext()
    dircol = DirectCollocation(
            plant,
            context,
            num_time_samples=40,
            minimum_time_step=0.1,
            maximum_time_step=0.6,
            input_port_index=plant.get_actuation_input_port().get_index()
    )

    dircol.AddEqualTimeIntervalsConstraints()

    initial_state = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    dircol.prog().AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())

    final_state = (0.0, np.pi, 0.0, 0.0, 0.0, 0.0)

    dircol.prog().AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())

    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] <= 1)
    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] >= -1)

    print(np.column_stack((initial_state, final_state)))

    # Load the initial trajectory from the keyframe generation
    data = np.load("./keyframes/swing_up_2.npz")
    keyframes = data["keyframes"]
    times = data["times"]

    print("Keyframes:\n", keyframes.T)
    print("Times:", times)

    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
            times, keyframes.T
    )

    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

    result = Solve(dircol.prog())
    assert result.is_success()

    u_trajectory = dircol.ReconstructInputTrajectory(result)
    times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100)

    u_values = np.array([
        u_trajectory.value(t).item()
        for t in times
    ])

    x_trajectory = dircol.ReconstructStateTrajectory(result)

    print("Done Solving")

    print(f"Start (t = {x_trajectory.start_time()}):")
    print(x_trajectory.value(x_trajectory.start_time()))

    print(f"End: (t = {x_trajectory.end_time()})")
    print(x_trajectory.value(x_trajectory.end_time()))

    # animate it
    if animate:

        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(context)

        t0 = x_trajectory.start_time()
        tf = x_trajectory.end_time()

        dt = 0.01

        t = t0
        while t <= tf:
            x = x_trajectory.value(t).flatten()

            plant.SetPositionsAndVelocities(plant_context, x)

            diagram.ForcedPublish(context)

            time.sleep(dt)
            t += dt
    
    return x_trajectory, u_trajectory

def lqr_double_swing_up():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels("./sliding_double_pendulum.urdf")
    plant.Finalize()

    context = plant.CreateDefaultContext()
    dircol = DirectCollocation(
            plant,
            context,
            num_time_samples=20,
            minimum_time_step=0.1,
            maximum_time_step=0.6,
            input_port_index=plant.get_actuation_input_port().get_index()
    )

    dircol.AddEqualTimeIntervalsConstraints()

    initial_state = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    dircol.prog().AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())

    final_state = (0.0, np.pi, 0.0, 0.0, 0.0, 0.0)

    lower_final_state = (-0.5, np.pi, 0.0, 0.0, 0.0, 0.0)
    upper_final_state = (0.5, np.pi, 0.0, 0.0, 0.0, 0.0)

    dircol.prog().AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())

    # constrain the movement of the cart
    # dircol.prog().AddBoundingBoxConstraint([-1.5], [1.5], dircol.state()[0])
    # x = dircol.state()[0]
    # dircol.prog().AddConstraint(-1.0 <= x)

    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] <= 1.5)
    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] >= -1.5)

    # cost of actuation
    dircol.AddRunningCost(1 * (dircol.input()[0])**2)

    # dircol.AddRunningCost(10 / sym.abs((dircol.state()[1]) + 1e-6))

    # cost of bending the second link
    dircol.AddRunningCost(1 * (dircol.state()[2])**2)

    # cost of leaving the center
    dircol.AddRunningCost(1 * (dircol.state()[0])**4)
    # x = dircol.state()[0]
    # dircol.AddRunningCost(20 * np.square(np.maximum(0.0, np.abs(x) - 0.5)))
    # dircol.AddRunningCost(lambda x, u: smooth_barrier(x[0]))
    # dircol.AddRunningCost((x / 1)**4)

    # dircol.AddFinalCost(dircol.time())

    # initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
    #         [0.0, 4.0], np.column_stack((initial_state, final_state))
    # )

    # build an initial trajectory
    # t = np.linspace(0, 4, 50)
    # q1 = np.sin(np.pi * t / 2) # smooth swing
    # q2 = np.zeros_like(t)
    # x = np.zeros_like(t)
    # 
    # dq1 = np.gradient(q1, t)
    # dq2 = np.zeros_like(t)
    # dx = np.zeros_like(t)

    num_points = 8

    t = np.linspace(0, 4, num_points)
    cart_disp = 0.75
    x = []
    for i in range(num_points):
        if i % 2 == 0:
            x.append(cart_disp)
        else:
            x.append(-cart_disp)

    q1 = np.zeros_like(t)
    q2 = np.zeros_like(t)
    dq1 = np.zeros_like(t)
    dq2 = np.zeros_like(t)
    dx = np.zeros_like(t)

    X = np.vstack([x, q1, q2, dx, dq1, dq2])

    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
            t,
            X
    )

    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

    result = Solve(dircol.prog())
    assert result.is_success()

    u_trajectory = dircol.ReconstructInputTrajectory(result)

    # print(u_trajectory.value(0.5))
    # print(type(u_trajectory.value(0.5)))
    # print(u_trajectory.value(0.5).shape)

    times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100)
    # u_lookup = np.vectorize(u_trajectory.value)
    # u_values = u_lookup(times)

    u_values = np.array([
        u_trajectory.value(t).item()
        for t in times
    ])

    x_trajectory = dircol.ReconstructStateTrajectory(result)

    print("Done Solving")

    print(f"Start (t = {x_trajectory.start_time()}):")
    print(x_trajectory.value(x_trajectory.start_time()))

    print(f"End: (t = {x_trajectory.end_time()})")
    print(x_trajectory.value(x_trajectory.end_time()))

    # animate it
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    t0 = x_trajectory.start_time()
    tf = x_trajectory.end_time()

    dt = 0.01

    t = t0
    while t <= tf:
        x = x_trajectory.value(t).flatten()

        plant.SetPositionsAndVelocities(plant_context, x)

        diagram.ForcedPublish(context)

        time.sleep(dt)
        t += dt

def lqr_double_pendulum(shoulder_angle=np.pi, elbow_angle=0, shoulder_deviation=0.2, elbow_deviation=0.2, animate=False):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels("./sliding_double_pendulum.urdf")
    plant.Finalize()

    print(plant.GetStateNames()) 
    # ['sliding_pendulum_slider_joint_x', 'sliding_pendulum_shoulder_q', 'sliding_pendulum_elbow_q', 
    #  'sliding_pendulum_slider_joint_v', 'sliding_pendulum_shoulder_w', 'sliding_pendulum_elbow_w']

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
        10,      # slider position
        100,    # angle of first link
        100,    # angle of second link
        1,      # velocity of slider
        10,     # shoulder angular velocity
        10,     # elbow angular velocity
    ])

    # define R: the cost of actuation
    R = np.array([[1]])

    K, S = LinearQuadraticRegulator(
            linear_system.A(),
            linear_system.B(),
            Q,
            R
    )

    if animate:
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

        logger = LogVectorOutput(plant.get_state_output_port(), builder)

        diagram = builder.Build()
        simulator = Simulator(diagram)

        simulator.set_target_realtime_rate(1.0)
        context = simulator.get_mutable_context()

        plant_context = plant.GetMyContextFromRoot(context)

        # set the initial positions
        joint = plant.GetJointByName("shoulder")
        joint.set_angle(plant_context, shoulder_angle - shoulder_deviation)
        joint.set_angular_rate(plant_context, 0.0)

        joint = plant.GetJointByName("elbow")
        joint.set_angle(plant_context, elbow_angle + elbow_deviation)
        joint.set_angular_rate(plant_context, 0.0)

        joint = plant.GetJointByName("slider_joint")
        joint.set_translation(plant_context, 0.0)
        joint.set_translation_rate(plant_context, 0.0)

        simulator.AdvanceTo(20.0)

    return K


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

def tvlqr(x_trajectory, u_trajectory, up_up_K):

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels("./sliding_double_pendulum.urdf")
    plant.Finalize()

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

    # add the selector controller
    selector_controller = builder.AddNamedSystem("selector_controller", SelectorController())

    # add the lqr controller
    lqr_controller = builder.AddNamedSystem("lqr_controller", LQRController(up_up_K))

    # add the tvlqr controller
    tvlqr_controller = builder.AddNamedSystem("tvlqr_controller", 
                                              TVLQRController(K_traj=tvlqr.K, x_traj=x_trajectory, u_traj=u_trajectory))

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

    # connec the selector to the actuator
    builder.Connect(selector_controller.get_output_port(), plant.get_actuation_input_port())

    # connect the controller to the plant
    # builder.Connect(plant.get_state_output_port(),
    #                 tvlqr_controller.state_port)
    #
    # builder.Connect(tvlqr_controller.get_output_port(),
    #                 plant.get_actuation_input_port())

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

# double_pendulum()
# lqr_double_pendulum(shoulder_angle=np.pi, elbow_angle=0, shoulder_deviation=0.2, elbow_deviation=-0.2)
# lqr_double_pendulum(shoulder_angle=np.pi, elbow_angle=np.pi, shoulder_deviation=0.1, elbow_deviation=-0.1)
# lqr_double_swing_up()
up_up_K = lqr_double_pendulum(shoulder_angle=np.pi, elbow_angle=0)

x_trajectory, u_trajectory = collocation_trajectory(animate=False)
tvlqr(x_trajectory, u_trajectory, up_up_K)

