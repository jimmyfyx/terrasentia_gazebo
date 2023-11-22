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

## `Record rosbag files`
- Create the environment with create_env_random.py
- Specify the environment in farm.world and record_data.launch
- Specify the rosbag path in record_data.launch
- Run
```
roslaunch terra_gazebo demo_world.launch
python3 mpd_node.py
roslaunch terra_gazebo record_data.launch
```