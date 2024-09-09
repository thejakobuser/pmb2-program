import numpy as np
import omni
import omni.ext
import omni.ui as ui
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid  # Add this import
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Gf, Usd, Sdf, UsdPhysics
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.physics_context import PhysicsContext
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import clear_stage



# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[jakob.pmb2] some_public_function was called with x: ", x)
    return x ** x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class JakobPmb2Extension(omni.ext.IExt):

    def __init__(self) -> None:
        super().__init__()
        self.frankas_view = None
        self._world = None
        self.vehicle = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        return
    
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[jakob.pmb2] jakob pmb2 startupa")

        stage = omni.usd.get_context().get_stage()
        # Add a physics scene prim to stage
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
        # Set gravity vector
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        PhysicsContext()

        if World.instance() is None:
            self._world = World()  # Pass your world settings if needed
        else:
            self._world = World.instance()



        self._window = ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            
                def load_world():

                    # Check if the world exists
                    world = self._world

                    if world:
                        print("World exists")

                        # world.instance().clear_instance()
                        # Add a ground plane to the world
                        world = World.instance()


                    else:
                        print("World does not exist")

                    clear_stage()

                    stage = omni.usd.get_context().get_stage()
                    # Add a physics scene prim to stage
                    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
                    # Set gravity vector
                    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
                    scene.CreateGravityMagnitudeAttr().Set(981.0)


                    # Create a new empty world
                    world = World()
                    world.scene.add_default_ground_plane(z_position=0.0)
                    world.reset()
                    world.initialize_physics()


                        




                def add_cube():
                    world = self._world
                    if world:
                        world.scene.add(
                            DynamicCuboid(
                                prim_path="/World/random_cube",
                                name="fancy_cube",
                                position=np.array([0.3, 0.3, 0.3]),
                                scale=np.array([0.0515, 0.0515, 0.0515]),
                                color=np.array([0, 0, 1.0]),
                            )
                        )
                    else:
                        print("World does not exist")

                def load_panda():
                    world = self._world
                    print(world)

                    asset_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

                    print("11111 hihihi hohoho hahah")



                    add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")

                   
                def panda_articulation():
                    world = self._world

                    # Check if the Franka prim exists, if not, load it
                    franka_prim = world.scene.stage.GetPrimAtPath("/World/Franka_1")
                    if not franka_prim.IsValid():
                        asset_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
                        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
                        print("Franka robot loaded")

                    # Ensure the world is updated
                    world.reset()

                    # Create or update the articulation view
                    if self.frankas_view is None or not self.frankas_view.initialized:
                        self.frankas_view = ArticulationView(prim_paths_expr="/World/Franka_1", name="frankas_view")
                        world.scene.add(self.frankas_view)
                        print("Franka articulation view created:", self.frankas_view)
                    else:
                        print("Using existing Franka articulation view:", self.frankas_view)

                    # Set root body pose for a single Franka robot
                    position = np.array([-1.0, 1.0, 0])
                    orientation = np.array([1.0, 0.0, 0.0, 0.0])
                    self.frankas_view.set_world_poses(positions=position.reshape(1, 3), orientations=orientation.reshape(1, 4))

                    # Set the joint positions for the single articulation
                    joint_positions = np.array([1, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04])
                    self.frankas_view.set_joint_positions(joint_positions.reshape(1, -1))

                    print("Franka robot position and joint positions updated")

                def load_robot():
                    world = self._world

                    # Define the path to the robot asset
                    asset_path = r"C:/Users/lakfe/Desktop/simulation/old/pmb/pmb.usd"

                    # Load the robot into the stage
                    add_reference_to_stage(asset_path, "/World/pmb")

                    # Check if the robot is loaded
                    pmb_prim = world.scene.stage.GetPrimAtPath("/World/pmb")
                    if not pmb_prim:
                        print("Failed to load robot at /World/pmb")
                        return

                    print("Robot loaded successfully at /World/pmb")

                    self.vehicle = ArticulationView("/World/pmb")
                    print("ArticulationView created for /World/pmb")

                    print(self.vehicle)

                    # Check if the articulation view is valid
                    if not self.vehicle:
                        print("Failed to create ArticulationView for /World/pmb")
                        return

                    self.vehicle.initialize()
                    print("Vehicle initialized:", self.vehicle.initialized)



                # List all joints of robot
                def list_joints():
                    world = self._world
                    print("Listing all joints under /World/pmb:")
                    pmb_prim = world.scene.stage.GetPrimAtPath("/World/pmb")
                    if pmb_prim:
                        for prim in Usd.PrimRange(pmb_prim):
                            if prim.IsA(UsdPhysics.Joint):
                                print(prim.GetPath())
                    else:
                        print("/World/pmb prim not found")


                # Drive the robot
                def drive_robot():
                    world = self._world
                    vehicle = self.vehicle

                    print("Driving the robot")
                    # if not vehicle.handles_initialized:

                    print(vehicle.initialized)

                    # vehicle.dof_properties["stiffness"].Set(1000)

                    # Get left and right wheel joints
                    # left_wheel_joint = world.scene.get_joint_at_path("/World/pmb/wheel_left_joint")
                    # right_wheel_joint = world.scene.get_joint_at_path("/World/pmb/wheel_right_joint")

                    # # Set the drive velocity for the left and right wheels
                    # left_wheel_joint.set_drive_velocity(1.0)  # Adjust the velocity as needed
                    # right_wheel_joint.set_drive_velocity(1.0)  # Adjust the velocity as needed

                def drive_panda():
                    world = self._world
                    vehicle = self.frankas_view

                    dc = _dynamic_control.acquire_dynamic_control_interface()
                    articulation = dc.get_articulation("/World/Franka_1")
                    dc.wake_up_articulation(articulation)
                    dof_ptr = dc.find_articulation_dof(articulation, "panda_joint2")
                    dc.set_dof_position_target(dof_ptr, 0)

                    print("Driving the robot")


                with ui.VStack(spacing=5):
                    label = ui.Label("PMB2 Simulation")

                    # Add a button to load the world
                    ui.Button("World", clicked_fn=load_world)
                    ui.Button("Cube", clicked_fn=add_cube)
                    ui.Button("Robot", clicked_fn=load_robot)
                    ui.Button("List Joints", clicked_fn=list_joints)
                    ui.Button("Drive robot", clicked_fn=drive_robot)
                    ui.Button("Panda", clicked_fn=load_panda)
                    ui.Button("Panda Articulation", clicked_fn=panda_articulation)
                    ui.Button("Drive Panda", clicked_fn=drive_panda)



    def on_shutdown(self):
        print("[jakob.pmb2] jakob pmb2 shutdown")
