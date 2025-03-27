# Turtlebot2 Unizar Nav

This repository is intended to be an entry point to anyone working with [Turtlebot2](https://www.turtlebot.com/turtlebot2/) in Unizar. The platform is quite old for ROS2 and this repository merges multiple sources from `Kobuki`, `Turtlebot2` descriptions, and sensor integrations. This repository currently contains unified code to run:

- Turtlebot2 Kobuki driver
- Turtlebot2 URDF description
- Hokuyo laser
- Realsense camera
- Nav2 stack with localization based on Hokuyo
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

### Docker installation

We provide a Dockerfile to run the system within Docker. This allows for easy install of all the dependencies. Also, this Dockerfile works as a reference of anything that should be done in a host machine in order to have the system ready. You can install and run the Docker with:

```bash
xhost + # To allow the Docker use the host graphical interface
sudo docker compose up -d # To run the docker-compose.yml and keep it running in the background. It will compile the container the first time you run it.
sudo docker exec -it tb2_unizar /bin/bash # To attach a terminal to the Docker.
```
By default, there will be a workspace called `tb2_unizar_compiled_ws` with everything needed for the demos.

You will need to do install the Docker in the robot computer for running all the software and the ground station where you want to run RViz and send the goals.

## Basic usage (demos)

Once you are in the Docker, you can run:
```
cd tb2_unizar_compiled_ws
```
And then you can source the workspace in the terminal:
```
source install/setup.bash
```
Move to the `tb2_unizar` package:
```
cd src/tb2_unizar
```
Finally, run the command:
```
./start_turtlebot.bash
```

For visualization, on another computer. You just need to run the RViz visualization:
```
cd tb2_unizar_compiled_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d config/rviz.rviz
```

## Usage development

The previous version is an already compiled and ready to use system that directly installs all the launchfile to deploy demos ASAP. If you want to work on developing anything else, you have two options:

### 1. Mount the tb2_unizar directory in a workspace

With this option you just mount the current repository with the `tb2_unizar` package inside `/root/tb2_unizar_ws/src/tb2_unizar`. The instructions are in the `docker-compose.yml` file. This won't persist your builds and only allows to modify the files in the current repository.

### 2. Mount your own workspace

Execute the `create_workspace.bash` script to create a workspace folder in the current directory. You can `git clone` any ROS 2 packages in the `src` folder and mount the `tb2_unizar` package in the `src/tb2_unizar` (see instructions in the `docker-compose.yml`). The advantage of this option is that you can edit anything in your workspace with VSCode. Additionally, the `build`, `install`, and `log` folders will be persisted across Docker executions.