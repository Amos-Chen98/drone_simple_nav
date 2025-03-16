# drone_simple_nav

## About

This is a minimum implementation of drone navigation.

**How it works:**

1. The planner uses A* to generate a collision-free path. Then a simple trajectory of constant speed is generated from the path.
2. Real-time position and velocity commands are derived from the trajectory and used for visualization. The commands follow the data structure defined in [mavros_msgs/PositionTarget](https://docs.ros.org/en/jade/api/mavros_msgs/html/msg/PositionTarget.html).

## **Installation**

Before installing this project, please make sure the following dependencies have been successfully installed and configured.

ROS1: https://wiki.ros.org/noetic/Installation/Ubuntu

```bash
sudo apt install ros-noetic-octomap*
sudo apt install graphviz graphviz-dev
pip install octomap-python
pip install pyquaternion
pip install scipy
pip install transitions[diagrams]

# Optional (not used in simulation, for debug only)
sudo apt install ros-noetic-octovis
```

Manually compile and install octomap (a Python binding of the original octomap package): In your preferred path, run

```bash
sudo apt update
sudo apt install -y git cmake build-essential liboctomap-dev
git clone https://github.com/OctoMap/octomap.git
cd octomap
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
pip install pybind11
```

Then, install this project as a ROS workspace

```bash
git clone https://github.com/Amos-Chen98/drone_simple_nav.git
cd drone_simple_nav
catkin build
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
```

## Usage

### Trajectory planning

```bash
roslaunch simulator run_sim.launch
```

RViz will pop up, then you can set the global target using `2D Nav Goal`. The drone will perform 3D planning and fly to the target.

Currently, the z position of the target is set in `src/simulator/launch/run_sim.launch`.

There are two ways of performing collision check:

1. Using Octomap (default): The octomap is loaded from a local .bt file. Collision checking is conducted by verifying whether the selected point and its neighboring points exist within the Octree.
2. KDtree in PCL: One ROS node reads a local point cloud file and streams the point cloud through a ROS topic of type [sensor_msgs/PointCloud2](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html). Collision check is performed using the point cloud.

There are three available navigation modes, controlled by the argument `replan_mode` in `src/simulator/launch/run_sim.launch`.

1. `global`: Plans once, generating a complete trajectory from the start to the target position.
2. `fix_time`: **(Default)** Continuously performs local planning until reaching the target, with a fixed replanning interval.
3. `rush`: Continuously performs local planning until reaching the target, immediately triggering replanning after the last replanning is finished.

Other major configurable parameters exist in the following three files:

`src/simulator/launch/run_sim.launch`

`src/planner/launch/planner_config.yaml`

`src/planner/launch/manager_config.yaml`

## Customized development

### 1. Batch random generation of Gazebo world

Run `src/simulator/scripts/generate_worlds.py`

This command will automatically generate a batch of gazebo world files.

The configurable parameters are listed in `src/simulator/scripts/generator_config.yaml`

### **2. Generate octomap and pointcloud offline from Gazebo world** 

Below are the instructions on how to generate ground truth octomap and/or pointcloud.

This function is based on the package `sim_gazebo_plugins`. To use the plugin, you need to edit your desired .world file to recognize the plugin. Simply open your .world file in a text editor and add the following line just before the final `<world>` tag (i. e. in between the `<world>` tags):

(If you are using the default `poles.world`, or any worlds generated from `generate_worlds.py`, this step is not required because the plugin has already been included.)

```
<plugin name='gazebo_octomap' filename='libBuildOctomapPlugin.so'/>
```

To generate a .pcd file and a .bt file , execute the following commands:

```bash
roslaunch simulator load_world.launch gazebo_world:=<your_world_file>
# Replace <your_world_file> with the filename of the world you wish to build a map from. This name should not not contain ".world"

rosservice call /world/build_octomap '{bounding_box_origin: {x: 0, y: 0, z: 15}, bounding_box_lengths: {x: 30, y: 30, z: 30}, leaf_size: 0.1, filename: output_filename.bt}'
```

Then the .pcd file and the .bt file will be generated in the path: /home/{username}/.ros/.

The `rosservice` call includes adjustable variables: the bounding box origin and lengths, both in meters. 

The lengths extend symmetrically in both (+/-) directions from the origin. For example, with an origin at `(0, 0, 0)` and bounding_box_lengths `{x: 30, y: 30, z: 30}`, the bounding box spans **-15 to +15 meters** in the X and Y directions, and **0 to 30 meters** in the Z direction.