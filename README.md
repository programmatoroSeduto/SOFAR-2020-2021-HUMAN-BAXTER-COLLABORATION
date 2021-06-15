# SofAR 2020/2021 - HUMAN BAXTER COLLABORATION

*Francesco Ganci, Zoe Betta, Lorenzo Causa, Federico Zecchi*

# Setting up and Running the project

## Prerequisites

- The project uses the "vanilla" version of the Unity simulation, with no changes. Link to the environment: [here](https://github.com/TheEngineRoom-UniGe/SofAR-Human-Robot-Collaboration.git). *Note well: we tried to make this work with no changes, so __if you modify this environment, the simulation couldn't properly work.__* 
- Some packages are required in order to esecute the project in the simulated environment. You can find all you need in our repository, folder `dependencies`.

## How to set up the project

In order to install the package:

1. create a workspace where to put the package. 
2. Make sure you have the reuired components in order to make the project run! 
3. Put the folder *controller_baxter* into the *src* folder of your workspace
4. then, build the workspace by `catkin_make`.

Here are the references to the required packages.

### DEPT -- MoveIt

In order to install MoveIt, launch the following instruction:

```bash
sudo apt install ros-noetic-moveit
```

### DEPT -- human_baxter_collaboration - ros_tcp_endpoint - (Unity Project) Human-Robot-Collaboration

You can find [here](https://github.com/TheEngineRoom-UniGe/SofAR-Human-Robot-Collaboration.git) the repository, containing all the tools for running the simulated Baxter. 

- **ROS SIDE**: download the repository into your ROS environment, and then copy the entire content of the folder *ROS Packages* alongside with the *controller_baxter* package. You can also directly copy the folder itself into *src*: the system is capable of recognizing the package. 
- **UNITY SIDE**: download the content of the folder *Unity Projects* wherever you want in your Host system, and open it with Unity. 

### DEPT -- baxter_common

You can find [here](https://github.com/RethinkRobotics/baxter_common) the package, which contains the description of Baxter. This is required in order to get the plan in MoveIt. 

For installing it, simply go into your workspace, and then launch this command:

```bash
git clone https://github.com/RethinkRobotics/baxter_common.git -b master baxter_common
```


## How to run the project

We're using Windows 10 as host for Unity and the Docker Virtual Machine through a [localhost](http://localhost) connection. 

First of all, in your ROS environment, launch the server:

```bash
roslaunch human_baxter_collaboration human_baxter_collaboration.launch &
```

On windows, launch the package `MMILauncher.exe`: it will let the human move in the simulation.

If the connection works fine, a print similar to this one should appear on your shell:

```bash
ROS-Unity Handshake received, will connect to 192.168.65.2:5005
```

Launch the Unity environment, then "play"→ start simulation.

Here you can launch all the components of our project. Make sure the simulation is running before launching these components.

First of all, launch `baxter_at_home` node:

```bash
rosrun controller_baxter baxter_at_home > /dev/null &
```

Then, launch the node `controller_baxter` which allows the robot to move:

```bash
rosrun controller_baxter controller_baxter > /dev/null & 
```

Last step: launch the task manager. After this, the robot will immediately start moving.

```bash
rosrun controller_baxter task_manager.py
```

## Time to Practice!

In case it is not possible to set up the project here we provided two small video experiments for our project.

- Test 1, using `unity_tf` topic

    [BAXTER-FINAL-unity.mp4](https://drive.google.com/file/d/1bliD6EbrQrFFnVxbXdXl74VSJtnRRKKW/view?usp=sharing)

- Test 2, using `tf` topic

    [BAXTER-FINAL-tf.mp4](https://drive.google.com/file/d/1p-_naDokhO7L7R_C7RtqwC0Slp7nUhX5/view?usp=sharing)

# Configuring the task manager

In the folder `controller_baxter/include/controller_baxter/sim_infos.py` you can find all the settings needed in order to let the task manager reasoning on the situation, and other parameters. Here is a little explaination of the most important ones. 

## Channels

- `use_unity`: if true, the task manager directly uses the topic `unity_tf` as source; otherwise, `tf` is used. This parameter was added thinking on the real robot implementation. 
If you don't need somethin special, i suggest you to set this as `True`: so you'll get the best performances.
- `topic_unity_tf`, `topic_tf`: names of the source topics of the task manager. You're not supposed to modify these parameters.
- `server_movement_controller`, `server_baxter_at_home`: names of the two services used by the controller. Don't modify.

## Logging

Each time the node receives a new message from the source topic, a counter is incremented. This counter let you to manage how many messages wait before a log. 

- `log_freq`: how many messages before a short log
- `log_freq_extended`: how many messages before an extended log. All the positions of the bocks are printed on the screen.
- `use_verbose`: useful in debugging. If `True`, you can see all the results of the intermediate evaluations. This coul help if you think there could be some problems. *Pay attention: sometimes this kind of log could gve some wrong informations*. Please be stick to the code if you want to use such logging mode.

## Object sizes

- `sz_cube`: size of a cube. All the blocks are equal.
- `sz_hand`: "radius" of the hand
- `sz_goal`: size of the goal box. 
sz_goal[0]: size along x (along the longest side of the table)
sz_goal[2]: size along z (orthogonal, on the plane, towards the human operator)
- `sz_table`: size of the table. 
sz_tablel[0]: size along x (along the longest side of the table)
sz_table[2]: size along z (orthogonal, on the plane, towards the human operator)

## Alert Distances

- `minimum_distance_parallel`: the task manager prefers the parallel execution is the bocks are too close, in order to avoid any collision by grippers. See our report for more informations.
- `center_dispacement`: the distance from the center of the table to which the controller_baxter node put the block on the left. See the report.
