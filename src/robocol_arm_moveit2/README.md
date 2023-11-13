# Robocol ARM MoveIt2
This is a ROS package implementation for ROS2 Robocol's robotic arm, which has is a 6 DoF robotic arm with steppers in its joints and a claw end effector.

# Building
In order to build this project you'll have to:

- Create (or use) a ROS workspace (ws from now on).
- Clone this repository to the *src* directory of the ws.
- Build your *ws* with the new package included.
- Run your files.

If you don't have a ws, make one. In this example *robocol_ws* will be done.

- Make your ws base directories:

```
mkdir -p ~/robocol_ws/src
```
- Go to the src directory of your ws:

```
cd ~/robocol_ws/src
```
- Clone the repository inside the src directory of your ROS ws:

```
git clone https://github.com/jcgutierrezg/robocol_arm_moveit2.git
```
- Go to the root of your ws:

```
cd ~/robocol_ws
```
- Build your ws:

```
colcon build
```
If the packages are correctly built you're done.

# Running nodes

## Main node
To run your node first build your ws. Then follow these steps.

- Go to the root of your ws

```
cd ~/robocol_ws
```

- Source your ws:

```
source ~/robocol_ws/install.setup.bash
```

- Run *main_node* node from *robocol_arm_moveit2* package with ROS2:

```
ros2 run robocol_arm_moveit2 main_node
```
