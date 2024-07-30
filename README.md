# terrasentia_gazebo

## Files not included from actual workspace
- All plants and terrain models in `/terra_worlds/models`
- Perception model checkpoint in `/terra_utils/scripts/models`

Get these folders from this following box link https://uofi.box.com/s/ys206zst9wedlws7lwdvq5qfriy9mng4 


## Run the following command to launch the gazebo environment and robot 
roslaunch terra_gazebo terra_gazebo.launch

## List of different options available for environments

There are 34 different plant environments available (20 environments of corn, 7 environments of sorghum and 7 environments of tobacco)

In order to change the environment, change line 48 in `/terra_worlds/worlds/farm.world` Following are the choices of environments available, use one of the names from the following list.

[`corn_plot_1`,`corn_plot_2`,`corn_plot_3`,`corn_plot_4`,`corn_plot_5`,`corn_plot_6`,`corn_plot_7`,`corn_plot_8`,`corn_plot_9`,`corn_plot_10`,`corn_plot_11`,`corn_plot_12`,`corn_plot_13`,`corn_plot_14`,`corn_plot_15`,`corn_plot_16`,`corn_plot_17`,`corn_plot_18`,`corn_plot_19`,`corn_plot_20`,`sorghum_plot_2`,`sorghum_plot_3`,`sorghum_plot_4`,`sorghum_plot_5`,`sorghum_plot_6`,`sorghum_plot_7`,`sorghum_plot_8`,`tobacco_plot_2`,`tobacco_plot_3`,`tobacco_plot_4`,`tobacco_plot_5`,`tobacco_plot_6`,`tobacco_plot_7`,`tobacco_plot_8`]

## Rostopic in which the control output from your RL policy should be published
`/terrasentia/cmd_vel' You should publish a Twist message (http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) in this rostopic

## Notes on rostopics that can be used to formulate the reward function

Use the following rostopics as input to the RL policy
`/terrasentia/vision/left/keypoint_heatmap` and '`/terrasentia/vision/right/keypoint_heatmap`

The following rostopics can be used to define the dense reward function

`/terrasentia/heading_error` - This topic denotes the relative angle of the robot with respect to the crop row. We desire this to be 0
`/terrasentia/distance_error` - This topic denotes the distance of the robot in meters measured from the middle of the lane. We desire this also to be 0.

Note the following 2 topics can be used to formulate a dense reward to train the controller. But since we do not have access to these in real world, formulate a sparse reward that applies a negative penalty when the robot crashes into a plant. Use the following ground truth rostopic to get the velocity of the robot along the x axis and compare this with the applied action to determine whether the robot crashed into plant or not. Use a curriculum based training where initially you apply higher weight for dense reward but reduce it gradually to make it 0 during final stages of training.
`/terrasentia/ground_truth`

Also, there are some random gaps in the crop rows because of which the robot might escape from the desired row and enter the neighboring one. So you can treat this also as a scenario of robot crashing into the plant. But since the velocity based crash detection would not work here, you can add an if condition that checks the absolute value of distance error is more than 0.38 to determine this. (We use 0.38 since the width of the crop row is 0.76 and hence min and max value of distance error is -0.38 and 0.38 respectively)





