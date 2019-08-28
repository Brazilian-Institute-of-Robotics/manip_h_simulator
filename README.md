# MoveIt! Package for Manipulator-H

The **goal** from this package, called **manipulator_h_moveit**, is to propose a simulated control from Manipulator-H from ROBOTIS using MoveIt! framework.

![mhmi](https://user-images.githubusercontent.com/32513366/63880395-078cd380-c9a4-11e9-8364-3731cf085654.png)

**Source:** [ROBOTIS](http://www.robotis.us/robotis-manipulator-h/) and [MoveIt! Motion Planning Framework](https://moveit.ros.org/)
 
**OBS** This package was discontinued, because ROBOTIS stop to develop this model.

## **Requirements**

This package presents the following basic requirements in your computer:

- ROS version: **Kinetic**
- OS: **Ubuntu 16.04**
- Gazebo version: **7.0**

Besides that, it is necessary to have **MoveIt! 1.0** and **MongoDB** installed in your computer:

### MoveIt!
```sh    
sudo sudo apt-get install ros-kinetic-moveit
```
### MongoDB
To install the complete version, follow the steps bellow in a terminal.
```sh    
wget -qO - https://www.mongodb.org/static/pgp/server-4.0.asc | sudo apt-key add -
```
```sh    
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu xenial/mongodb-org/4.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.0.list
```
```sh    
sudo apt-get update 
```
```sh    
sudo apt-get install -y mongodb-org
```

For more info about this installation, acess [here](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/).

## Dependências de outros pacotes
Besides **manipulator_h_moveit** package, your catkin workspace most include in your **src folder** the following packages:

- [ROBOTIS-MANIPULATOR-H](https://github.com/KaikeWesleyReis/ROBOTIS-MANIPULATOR-H.git) (master branch)

    ```sh
    git clone https://github.com/KaikeWesleyReis/ROBOTIS-MANIPULATOR-H.git
    ```
- [warehouse_ros_mongo](https://github.com/ros-planning/warehouse_ros_mongo.git) (jade-devel branch)
    ```sh
    git clone https://github.com/ros-planning/warehouse_ros_mongo.git
    ```
- [qt_ros](https://github.com/stonier/qt_ros) (indigo branch)
    ```sh
    git clone https://github.com/stonier/qt_ros.git
    ```
- [ROBOTIS-Math](https://github.com/ROBOTIS-GIT/ROBOTIS-Math/tree/master) (master branch)
    ```sh
    git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git
    ```
- [ROBOTIS-Framework](https://github.com/ROBOTIS-GIT/ROBOTIS-Framework) (master branch)
    ```sh
    git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git
    ```
- [ROBOTIS-Framework-msgs](https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs) (master branch)
    ```sh
    git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git
    ```
- [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) (master branch)
    ```sh
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    ```

## **Dependencies from MoveIt! to control**

The packages listed bellow are necessary to use MoveIt! with ROS Control and Gazebo. Your installation is necessary is your **ROS directory**.

- **ROS Control**: A set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces.

- **Gazebo ROS Control**: Package for integrating the ros_control controller architecture with the Gazebo simulator.

- **Controller Manager**: The controller_manager provides a hard-realtime-compatible loop to control a robot mechanism.

- **Joint Trajectory Controller**: Controller for executing joint-space trajectories on a group of joints.

- **Joint State Controller**: Controller to publish joint state

- **Position Controllers**: Controller package to move your robot based in given positions.

To install, in a terminal:
```sh
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-gazebo-ros-control ros-kinetic-controller-manager ros-kinetic-joint-trajectory-controller ros-kinetic-joint-state-controller
ros-kinetic-position-controllers
```
Then in your **catkin workspace folder** run:
```sh
rosdep install --from-paths src --ignore-src -r -y
```
```sh
catkin_make # or catkin build (choose the one you're using it)
```

## **Usage**

### **1. Go to Goal**

Run Gazebo simulation and MoveIt! back-end:

```sh
roslaunch manipulator_h_moveit mh_deploy_gazebo_moveit.launch
```
Run the application:

```sh
rosrun manipulator_h_moveit mh_goToGoal.py
```

### **2. MoveIt! Benchmark**
To know more about MoveIt! Benchmarking and understand more terms such as query or log and to understand how to create it, follow the tutorials [here](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/benchmarking/benchmarking_tutorial.html).

Run the **demo.launch** to create your queries and start state (within the package, there are already queries and a start state):

```sh
roslaunch manipulator_h_moveit demo.launch db:=true
```

Run the benchmark process:

```sh
roslaunch manipulator_h_moveit mh_benchmark.launch
```

To generate your database:
```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py diretorio_do_log
```
Onde **diretorio_do_log** is the path (where the created log is located) for one of your logs created after the previous command.

To analyze your results, acess [Planner Arena](http://plannerarena.org/) and update your database **.db**.

**OBS:** To change general options from the benchmark process such as the planners, log path, runs and so on change in *mh_benchmark_opts.yaml* in config folder.
