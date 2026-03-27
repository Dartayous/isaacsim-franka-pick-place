from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import UsdGeom, Gf
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction

ENV_PATH = "/root/.local/share/ov/data/USD_PROJECTS/project_02_isaac_robotics_starter_twin/usd/robotics_starter_v05_small_cube.usda"

context = omni.usd.get_context()
context.new_stage()
stage = context.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

print("Created fresh runtime stage.")
print("Runtime Up Axis:", UsdGeom.GetStageUpAxis(stage))
print("Runtime Meters Per Unit:", UsdGeom.GetStageMetersPerUnit(stage))

# Create clean world structure
UsdGeom.Xform.Define(stage, "/World")
wrapper = UsdGeom.Xform.Define(stage, "/World/EnvWrapper")
UsdGeom.Xform.Define(stage, "/World/Robot")

# Reference the authored Y-up environment under a wrapper
wrapper_prim = stage.GetPrimAtPath("/World/EnvWrapper")
wrapper_prim.GetReferences().AddReference(ENV_PATH)

# Rotate only the wrapper into runtime orientation
wrapper_xf = UsdGeom.Xformable(wrapper_prim)
wrapper_xf.ClearXformOpOrder()
wrapper_xf.AddRotateXOp().Set(90.0)

for _ in range(120):
    simulation_app.update()

# Build world and add robot separately at world level
world = World(stage_units_in_meters=1.0)

franka = world.scene.add(
    Franka(
        prim_path="/World/Franka",
        name="franka",
        position=np.array([-1.0, 0.40, 0.0], dtype=np.float32),
        orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    )
)

print("Franka wrapper added.")
world.reset()
print("World reset complete.")
print("DOF names:", franka.dof_names)

neutral_joint_positions = np.array([
    0.0,    # panda_joint1
    0.05,   # panda_joint2
    0.0,    # panda_joint3
   -0.75,   # panda_joint4
    0.0,    # panda_joint5
    0.65,   # panda_joint6
    0.60,   # panda_joint7
    0.04,   # panda_finger_joint1
    0.04    # panda_finger_joint2
], dtype=np.float32)

franka.set_joint_positions(neutral_joint_positions)
print("Neutral joint pose applied.")
print(neutral_joint_positions)

# Hold neutral longer
for _ in range(120):
    world.step(render=True)

controller = franka.get_articulation_controller()

hover_positions = np.array([
    1.50,
    0.05,
    0.00,
   -0.50,
    0.00,
    1.60,
    0.70,
    0.04,
    0.04
], dtype=np.float32)

descend_positions = np.array([
    1.50,
    0.05,
    0.00,
   -0.83,
    0.00,
    1.60,
    0.70,
    0.04,
    0.04
], dtype=np.float32)

close_positions = np.array([
    1.50,
    0.05,
    0.00,
   -0.83,
    0.00,
    1.60,
    0.70,
    0.00,
    0.00
], dtype=np.float32)

lift_positions = np.array([
    1.50,
    0.05,
    0.00,
   -0.50,
    0.00,
    1.60,
    0.70,
    0.00,
    0.00
], dtype=np.float32)

hover_action = ArticulationAction(joint_positions=hover_positions)
descend_action = ArticulationAction(joint_positions=descend_positions)
close_action = ArticulationAction(joint_positions=close_positions)
lift_action = ArticulationAction(joint_positions=lift_positions)

print("Moving to hover...")
for _ in range(240):
    controller.apply_action(hover_action)
    world.step(render=True)

print("Descending...")
for _ in range(240):
    controller.apply_action(descend_action)
    world.step(render=True)

print("Settling at descend pose...")
for _ in range(120):
    controller.apply_action(descend_action)
    world.step(render=True)

print("Closing fingers at descend pose...")
for _ in range(180):
    controller.apply_action(close_action)
    world.step(render=True)

print("Holding closed grasp before lift...")
for _ in range(200):
    controller.apply_action(close_action)
    world.step(render=True)

print("Lifting slowly...")

lift_steps = 240  # increase for even gentler lift

for i in range(lift_steps):
    alpha = (i + 1) / lift_steps
    blended_positions = (1.0 - alpha) * close_positions + alpha * lift_positions
    slow_lift_action = ArticulationAction(joint_positions=blended_positions)
    controller.apply_action(slow_lift_action)
    world.step(render=True)

print("Holding lifted pose...")
for _ in range(120):
    controller.apply_action(ArticulationAction(joint_positions=lift_positions))
    world.step(render=True)

print("Done.")
simulation_app.close()