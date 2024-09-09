import numpy as np
import omni.ext
import omni.ui as ui
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid  # Add this import
from omni.isaac.core.utils.stage import add_reference_to_stage




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
        self._world = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        return
    
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[jakob.pmb2] jakob pmb2 startup")

        if World.instance() is None:
            self._world = World(**self._world_settings)  # Pass your world settings if needed
        else:
            self._world = World.instance()

        self._count = 0

        self._window = ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            
                def load_world():

                    # Check if the world exists
                    world = self._world
                    if world:
                        print("World exists")

                        # Add a ground plane to the world
                        world.scene.add_default_ground_plane()
                    else:
                        print("World does not exist")

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

                def load_robot():
                    world = self._world

                    # Define the path to the robot asset
                    asset_path = r"C:/Users/lakfe/Desktop/simulation/old/pmb/pmb.usd"

                    # Load the robot into the stage
                    add_reference_to_stage(asset_path, "/World/pmb")


                def list_joints():
                    world = self._world
                    if world:
                        print("Listing all joints under /World/pmb:")
                        pmb_prim = world.scene.stage.GetPrimAtPath("/World/pmb")
                        if pmb_prim:
                            from pxr import Usd, UsdPhysics
                            for prim in Usd.PrimRange(pmb_prim):
                                if prim.IsA(UsdPhysics.Joint):
                                    print(prim.GetPath())
                        else:
                            print("/World/pmb prim not found")
                    else:
                        print("World does not exist")

                # Stack the buttons horizontally
                with ui.VStack(spacing=5):
                    label = ui.Label("PMB2 Simulation")

                    # Add a button to load the world
                    ui.Button("World", clicked_fn=load_world)
                    ui.Button("Cube", clicked_fn=add_cube)
                    ui.Button("Robot", clicked_fn=load_robot)
                    ui.Button("List Joints", clicked_fn=list_joints)



    def on_shutdown(self):
        print("[jakob.pmb2] jakob pmb2 shutdown")
