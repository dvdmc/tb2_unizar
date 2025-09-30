# Turtlebot2 Unizar Nav

This repository is intended to be an entry point to anyone working with [Turtlebot2](https://www.turtlebot.com/turtlebot2/) in Unizar. The platform is quite old for ROS2 and this repository merges multiple sources from `Kobuki`, `Turtlebot2` descriptions, and sensor integrations. This repository currently contains unified code to run:

- Turtlebot2 Kobuki driver
- Turtlebot2 URDF description
- Hokuyo laser
- Realsense camera
- Nav2 stack with localization based on Hokuyo
- Localization with Optitrack
- Pending: Gazebo simulation

It can be used as a template to create more complex projects. This repository can be used both at the robot and the ground station computers.

## Installation

The current standalone installation is by using the Docker provided below. If you don't want to use docker, you can follow the steps in the Dockerfile to install everything manually.

Connect to the turtlebot via ssh.
```
ssh kobuki@{ROBOT_IP}
```
Clone the repository in the robot.
```
git clone ...
```
On the machine that will connect to the sensors (robot mainly), use:
```
sudo usermod -aG dialout $USER
```
for using the serial connections to the Kobuki, and Hokuyo laser.

Now you can directly proceed to the following Docker installation steps.

### Docker installation & running

We provide a Dockerfile to run the system within Docker. This allows for easy installation of all the dependencies. Also, this Dockerfile works as a reference for anything that should be done in a host machine in order to have the system ready. You can install and **run (entrypoint for demos)** the Docker with:

```bash
xhost + # To allow the Docker use the host graphical interface
sudo docker compose up -d # To run the docker-compose.yml and keep it running in the background. It will compile the container the first time you run it (~20min on Intel NUC).
sudo docker exec -it tb2_unizar /bin/bash # To attach a terminal to the Docker.
```
By default, there will be a workspace called `tb2_unizar_compiled_ws` with everything needed for the demos.

You will need to install the Docker in the robot computer for running all the navigation software and on the ground station where you will run RViz and send the goals.

## Basic usage (demos)

Once you are in the Docker, you can source the workspace in the terminal:
```
source tb2_unizar_compiled_ws/install/setup.bash
```
Move to the `tb2_unizar` package:
```
cd tb2_unizar_compiled_ws/src/tb2_unizar
```
Finally, run the command:
```
./start_turtlebot.bash -m hokuyo
```
This will start everything needed for navigating in the I3A laboratory with Nav2 on a TB2 with a Hokuyo laser sensor on the top connected with USB.

For visualization, on another computer, you can run docker and attach a terminal. Then, you just need to run the RViz visualization:
```
source tb2_unizar_compiled_ws/install/setup.bash
ros2 run rviz2 rviz2 -d tb2_unizar_compiled_ws/src/tb2_unizar/config/rviz.rviz
```

To stop everything:
- On the robot, run `./stop_turtlebot.bash` in any of the open terminals.
- On the ground station, just do Ctrl+c to stop the RViz visualization.
- On both computers, run:
```bash
exit
sudo docker compose down
```

## Usage development

The previous version is an already compiled and ready to use system that directly installs all the launchfile to deploy demos ASAP. If you want to work on developing anything else, you have two options:

### 1. Mount the tb2_unizar directory in a workspace

With this option you just mount the current repository with the `tb2_unizar` package inside `/root/tb2_unizar_ws/src/tb2_unizar`. The instructions are in the `docker-compose.yml` file. This won't persist your builds and only allows to modify the files in the current repository.

### 2. Mount your own workspace (recommended)

Execute the `create_workspace.bash` script to create a workspace folder in the current directory. You can `git clone` any ROS 2 packages in the `src` folder and mount the `tb2_unizar` package in the `src/tb2_unizar` (see instructions in the `docker-compose.yml`). The advantage of this option is that you can edit anything in your workspace with VSCode. Additionally, the `build`, `install`, and `log` folders will be persisted across Docker executions.

### Configure your own TB2

The entrypoint should be `start_turtlebot.bash`, which will start a `tmux` session following the description of a `tmuxinator` template. Check these files to understand the required modules to run.

## TB2 Unizar package structure

The `tb2_unizar` package included in this repository is structured as follows:

- `config`: config files for the different execution modes. They will generally be loaded by `tmuxinator` files and `launch` files.
- `include` / `src`: code for required nodes. This package only contains specific nodes that we need.
- `launch`: our own launch files. We try to use standalone launch files when possible (nav2 bringup and mocap4ros2) but we need to make slight changes to others (hokuyo and kobuki).
- `maps`: our saved maps.
- `meshes`/`urdf`: a backup of the URDFs and meshes from the `turtlebot_description` ROS (1) package that include specific files for the Turtlebot 2.
- `tmuxinator`: templates for orchestrated system launch. They are called from the `start_turtlebot.bash` script.

## Steps for Hokuyo / Realsense run

WIP: Explain config sensors and preliminary checks. Map recording and navigation.

- Configure correctly the `scan` topic on the `Hokuyo` and `Nav2` config files.

## Steps for Optitrack

For running this TB2 with Optitrack, you need the [optitrack_pose_broadcaster](https://github.com/dvdmc/optitrack_pose_broadcaster) package and
configure the Optitrack computer IP and the robot local IP on `config/optitrack/mocap4r2_optitrack.yaml`.

### Specific params

- The Kobuki node `publish_tf` is set to false because the `gt_pose_broadcaster` node will publish this transform.
- The Nav2 map is a dummy one (TODO: Check if this is still required), and `use_localization` is set to false.
- The ground truth TF for odometry is `odom`, but the odometry topic is `gt_pose`.
- Deactivated the `scan` observations in the `collision_monitor`. Otherwise, there will be no `cmd_vel`message.
