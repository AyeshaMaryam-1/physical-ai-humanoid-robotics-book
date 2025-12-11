"""
Isaac Sim Script for Humanoid Robot Setup

This script demonstrates how to programmatically create and configure
a humanoid robot in Isaac Sim using the Omniverse Kit API.
"""

import omni
import omni.ext
import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
import carb


def create_humanoid_robot_stage():
    """
    Create a new USD stage with a humanoid robot setup
    """
    # Create a new stage
    stage = omni.usd.get_context().get_stage()

    # Set up the root prim
    root_path = Sdf.Path("/World")
    if not stage.GetPrimAtPath(root_path):
        UsdGeom.Xform.Define(stage, root_path)

    # Create robot root
    robot_path = Sdf.Path("/World/humanoid_robot")
    robot_prim = UsdGeom.Xform.Define(stage, robot_path)

    # Create base link
    base_path = Sdf.Path("/World/humanoid_robot/base_link")
    base_prim = UsdGeom.Capsule.Define(stage, base_path)
    base_prim.CreateRadiusAttr(0.3)
    base_prim.CreateHeightAttr(0.6)

    # Create torso
    torso_path = Sdf.Path("/World/humanoid_robot/torso")
    torso_prim = UsdGeom.Cylinder.Define(stage, torso_path)
    torso_prim.GetPrim().SetMetadata("xformOp:translate", Gf.Vec3f(0, 0.5, 0))
    torso_prim.CreateRadiusAttr(0.25)
    torso_prim.CreateHeightAttr(0.5)

    # Create head
    head_path = Sdf.Path("/World/humanoid_robot/head")
    head_prim = UsdGeom.Sphere.Define(stage, head_path)
    head_prim.GetPrim().SetMetadata("xformOp:translate", Gf.Vec3f(0, 0.9, 0))
    head_prim.CreateRadiusAttr(0.15)

    # Create left arm
    left_arm_path = Sdf.Path("/World/humanoid_robot/left_arm")
    left_arm_prim = UsdGeom.Cylinder.Define(stage, left_arm_path)
    left_arm_prim.GetPrim().SetMetadata("xformOp:translate", Gf.Vec3f(-0.4, 0.3, 0))
    left_arm_prim.GetPrim().SetMetadata("xformOp:rotateY", 45)
    left_arm_prim.CreateRadiusAttr(0.075)
    left_arm_prim.CreateHeightAttr(0.4)

    # Create right arm
    right_arm_path = Sdf.Path("/World/humanoid_robot/right_arm")
    right_arm_prim = UsdGeom.Cylinder.Define(stage, right_arm_path)
    right_arm_prim.GetPrim().SetMetadata("xformOp:translate", Gf.Vec3f(0.4, 0.3, 0))
    right_arm_prim.GetPrim().SetMetadata("xformOp:rotateY", -45)
    right_arm_prim.CreateRadiusAttr(0.075)
    right_arm_prim.CreateHeightAttr(0.4)

    # Create left leg
    left_leg_path = Sdf.Path("/World/humanoid_robot/left_leg")
    left_leg_prim = UsdGeom.Cylinder.Define(stage, left_leg_path)
    left_leg_prim.GetPrim().SetMetadata("xformOp:translate", Gf.Vec3f(-0.15, -0.5, 0))
    left_leg_prim.CreateRadiusAttr(0.075)
    left_leg_prim.CreateHeightAttr(0.5)

    # Create right leg
    right_leg_path = Sdf.Path("/World/humanoid_robot/right_leg")
    right_leg_prim = UsdGeom.Cylinder.Define(stage, right_leg_path)
    right_leg_prim.GetPrim().SetMetadata("xformOp:translate", Gf.Vec3f(0.15, -0.5, 0))
    right_leg_prim.CreateRadiusAttr(0.075)
    right_leg_prim.CreateHeightAttr(0.5)

    # Apply basic materials
    apply_basic_materials(stage)

    # Apply physics properties
    apply_physics_properties(stage)

    carb.log_info("Humanoid robot created successfully")


def apply_basic_materials(stage):
    """
    Apply basic materials to the robot parts
    """
    # Create a simple material
    material_path = Sdf.Path("/World/Looks/RobotMaterial")
    material = UsdShade.Material.Define(stage, material_path)

    # Create a preview surface shader
    shader_path = Sdf.Path("/World/Looks/RobotMaterial/PreviewSurface")
    shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")

    # Set material properties
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.8, 0.8, 0.8))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)

    # Connect the shader to the material
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # Bind the material to all robot parts
    robot_parts = [
        "/World/humanoid_robot/base_link",
        "/World/humanoid_robot/torso",
        "/World/humanoid_robot/head",
        "/World/humanoid_robot/left_arm",
        "/World/humanoid_robot/right_arm",
        "/World/humanoid_robot/left_leg",
        "/World/humanoid_robot/right_leg"
    ]

    for part_path in robot_parts:
        part_prim = stage.GetPrimAtPath(part_path)
        if part_prim:
            UsdShade.MaterialBindingAPI(part_prim).Bind(material)


def apply_physics_properties(stage):
    """
    Apply basic physics properties to the robot
    """
    # Create physics scene
    scene_path = Sdf.Path("/World/physicsScene")
    scene = UsdPhysics.Scene.Define(stage, scene_path)

    # Set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Apply rigid body properties to each part
    robot_parts = [
        "/World/humanoid_robot/base_link",
        "/World/humanoid_robot/torso",
        "/World/humanoid_robot/head",
        "/World/humanoid_robot/left_arm",
        "/World/humanoid_robot/right_arm",
        "/World/humanoid_robot/left_leg",
        "/World/humanoid_robot/right_leg"
    ]

    for part_path in robot_parts:
        part_prim = stage.GetPrimAtPath(part_path)
        if part_prim:
            # Apply rigid body API
            UsdPhysics.RigidBodyAPI.Apply(part_prim)

            # Set mass properties
            phys_body = UsdPhysics.RigidBodyAPI(part_prim)
            phys_body.CreateMassAttr(1.0)
            phys_body.CreateLinearVelocityAttr(Gf.Vec3f(0, 0, 0))
            phys_body.CreateAngularVelocityAttr(Gf.Vec3f(0, 0, 0))


def setup_environment():
    """
    Set up the basic environment for the robot
    """
    stage = omni.usd.get_context().get_stage()

    # Create ground plane
    ground_path = Sdf.Path("/World/ground")
    ground = UsdGeom.Mesh.Define(stage, ground_path)
    ground.CreatePointsAttr([(0, 0, 0), (10, 0, 0), (10, 10, 0), (0, 10, 0)])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 2, 3])
    ground.CreateFaceVertexCountsAttr([3, 3])

    # Apply physics to ground
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    carb.log_info("Environment setup completed")


def run_basic_simulation():
    """
    Run a basic simulation to test the humanoid robot
    """
    # This function would typically contain code to run a basic simulation
    # In a real Isaac Sim environment, this would involve:
    # - Starting the physics simulation
    # - Applying forces or torques to the robot
    # - Observing the robot's behavior
    # - Collecting sensor data
    # - etc.

    carb.log_info("Basic simulation setup completed. In Isaac Sim, press Play to run the simulation.")


# Main execution
if __name__ == "__main__":
    create_humanoid_robot_stage()
    setup_environment()
    run_basic_simulation()