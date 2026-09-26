import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    MeshcatVisualizer,
    Parser,
    StartMeshcat,
)

import time
import os

meshcat = StartMeshcat()
while meshcat.GetNumActiveConnections() == 0:
    print(meshcat.GetNumActiveConnections(), "Connections")
    time.sleep(1)

time.sleep(1)

def double_pendulum_animator():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels("./sliding_double_pendulum.urdf")
    plant.Finalize()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    diagram_context = context = diagram.CreateDefaultContext()
    context = plant.GetMyContextFromRoot(diagram_context)

    meshcat.AddSlider(
            name="cart",
            min=-1.0,
            max=1.0,
            step=0.01,
            value=0.0
    )

    meshcat.AddSlider(
            name="shoulder",
            min=-np.pi,
            max=np.pi,
            step=0.01,
            value=0.0
    )

    meshcat.AddSlider(
            name="elbow",
            min=-np.pi,
            max=np.pi,
            step=0.01,
            value=0.0
    )

    meshcat.AddSlider(
            name="time",
            min=0,
            max=30,
            step=0.01,
            value=0.0
    )

    meshcat.AddButton("Save Keyframe")
    meshcat.AddButton("Finish")


    keyframes = []
    times = []
    keyframe_clicks = 0

    while(1):
        cart = meshcat.GetSliderValue("cart")
        shoulder = meshcat.GetSliderValue("shoulder")
        elbow = meshcat.GetSliderValue("elbow")
        current_time = meshcat.GetSliderValue("time")

        slider_joint = plant.GetJointByName("slider_joint")
        slider_joint.set_translation(context, cart)

        shoulder_joint = plant.GetJointByName("shoulder")
        shoulder_joint.set_angle(context, shoulder)

        elbow_joint = plant.GetJointByName("elbow")
        elbow_joint.set_angle(context, elbow)

        diagram.ForcedPublish(diagram_context)

        keyframe = meshcat.GetButtonClicks("Save Keyframe")
        save = meshcat.GetButtonClicks("Finish")

        if keyframe > keyframe_clicks:
            keyframe_clicks = keyframe

            state = plant.GetPositionsAndVelocities(context)

            keyframes.append(state.copy())
            times.append(current_time)

            print(f"Saved State at time: {current_time}:\n", state)

        if save:
            file_name = input("Enter file save name: ")
            keyframes = np.array(keyframes).astype(np.float64)
            times = np.array(times).astype(np.float64)

            if not os.path.exists("keyframes"):
                os.mkdir("keyframes")

            # np.save(os.path.join("keyframes", f"{file_name}.npy"), data)
            np.savez(
                os.path.join("keyframes", f"{file_name}.npz"),
                keyframes=keyframes,
                times=times
            )

            break

        time.sleep(0.01)

double_pendulum_animator()
