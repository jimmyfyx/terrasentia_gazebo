# terrasentia_gazebo

## Files not included from actual workspace
- Folder `/terra_description`
- All plants and terrain models in `/terra_worlds/models`

## Additional files added
- `/terra_worlds/scripts/create_env_random.py` Used to generate random environments in `.sdf`.
- `/terra_utils/scripts/rowswitch.py` Node for publishing MPC paths.
- `/terra_worlds/models/model.config` An additionl `model.config` used when generating random environements.

## `create_env_random.py`
With specified `num_envs` and `num_routes`, the script can generate random environments in `.sdf`. Each environment is saved in 
`env_{env_idx}` in `/terra_worlds/models`. Meanwhile, it can generate config file for routes information in `/terra_worlds/configs/env_{env_idx}`.

## `rowswitch.py`
This is a ROS node that is used to publish real-time MPC paths based on current robot position and routes information in `/terra_worlds/configs/env_{env_idx}`.

## Record rosbag files (Manually)
- Create the environment with `create_env_random.py`
- Specify the environment in `farm.world` and `record_data.launch`
- Specify the rosbag and environment config path in `record_data.launch`
- Run
```
roslaunch terra_gazebo demo_world.launch
python3 mpc_node.py
roslaunch terra_gazebo record_data.launch
```

## Record rosbag files (Automatically)
- Place `record_data.py` with the other `terra_xxx` folders in the same directory
- Create the environments with `create_env_random.py`
- Specify the paths in the initialization of `RecordTrajectoryLauncher` of `record_data.py`
    - `devel_dir`: The path of the `devel` folder in ROS workspace
    - `mpc_path`: The folder containing the `mpc_node.py` script
    - `world_file`: The path of the world file `farm.world`
    - `env_config_path`: The folder containing configurations for all environments (`.../terra_worlds/configs`)
    - `rosbag_path`: The folder to save rosbags
    - `gazebo_launch_file` and `record_launch_file` should not require changes
- Install the package `coloredlogs` with the command `pip install coloredlogs`
- Run
```
python3 record_data.py [args: env_start_idx, env_end_idx]
```

## Customize terrain
### Example terrain
- Download the heightmap model: https://drive.google.com/drive/folders/1KaRjKiA6zuJBjkgqsXdxFpO_XuU7Z07H?usp=sharing
- Place the folder under `/terra_worlds/models`
- Check `farm.world`, make sure it includes the heightmap model `heightmap_cornfield`
- Run the scripts as normal
### Customize terrain
Gazebo allows the use of a heightmap image to customize terrain. The grayscale image must be square with a width and length of (2^n) + 1 (etc. 1025x1025). 
For every pixel, black (0), represents zero height; white (255), represents a peak. The heightmap image should be placed in `/meshes/heightmaps` within a 
heightmap model folder. 

In `model.sdf`, make sure it includes the correct path to the heightmap image for both collision and visual. Also, the size of the terrain can be specified. 
```
<size>30 30 0.25</size>
```
The size is specified for both collision and visual, representing the 3D size of the terrain in meters (xyz). Particularly, the specified height represents the maximum height for the terrain, which are represented by the white pixels in the heightmap. 
### Corresponding modifications for scripts
When a new terrain is applied, modify the following parameters of the robot:
- In `/terra_gazebo/launch/spawn_robot/launch`, modify the initial z position of the robot according to the maximum height of the terrain
- Similarly, in `rowswitch.py`, under the function `set_robot_state()`, modify the z position of the robot when respawning


