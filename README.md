# MoveIt! Package for Manipulator-H

The **goal** from this package, called **manipulator_h_moveit**, is to propose a simulated control for Manipulator-H from ROBOTIS using MoveIt! framework.

![mhmi](https://user-images.githubusercontent.com/32513366/63880395-078cd380-c9a4-11e9-8364-3731cf085654.png)

**Source:** [ROBOTIS](http://www.robotis.us/robotis-manipulator-h/) and [MoveIt! Motion Planning Framework](https://moveit.ros.org/)

## **Requirements**

The basic requirements for this package:

- ROS version: **Kinetic**
- OS: **Ubuntu 16.04**
- Gazebo version: **7.0**

Besides that, it is necessary to have **MoveIt! 1.0** and **MongoDB** installed in your computer.

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

For more info about MongoDB installation, acess [here](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/).

## **Workspace Dependencies**
Besides **manipulator_h_moveit** itself, your catkin workspace must include in your **src folder** the following packages:

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

## **Requirements for MoveIt!**

The packages listed bellow are necessary to use MoveIt! with ROS Control and Gazebo:

To install them, run in a terminal:
```sh
sudo apt-get install ros-kinetic-ros-control ros-kinetic-gazebo-ros-control ros-kinetic-controller-manager ros-kinetic-joint-trajectory-controller ros-kinetic-joint-state-controller
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
Through ```mh_goToGoal.py``` the user can see the manipulator control through different formats: 
- Setting the joints (previous defined in the code)
- Command to a predetermined position, defined in MoveIt! Setup Assistant
- Set a arbitrary coordinate XYZ, by the user itself, in manipulator workspace

### **2. MoveIt! Benchmark**
MoveIt! Benchmarking helps the researcher to defined the best planner trajectory given your parameters, environment and goals. To a better comprehension of this tool, some terms needs to be clarify:
- **planner**: Algorithm capable to calculate a path (trajectory) between two points.
- **start state**: Position where your robot start at the very beginning. Used to avoid any colision scenario.
- **scene**: It is your environment itself, can contains your robot and obstacles.
- **query**: A path defined by a *start state* and *end state*. Beside that, a query have information about the scene where it was set, i.e. a query is contain in a scene. 
- **runs**: Numeric parameter. Is the number of attempts that a planner will have to calculate a query's trajectory.
- **timeout**: Numeric parameter. Is the maximum time (in seconds) allowed for a planner to find a solution.

For more information about it and others tutorials, you can acess
[here](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/benchmarking/benchmarking_tutorial.html).

### **2.1. Setting your environment for a Benchmark**

Run the **demo.launch**:

```sh
roslaunch manipulator_h_moveit demo.launch db:=true
```

#### **2.1.1. Create your Scene!**
Here, you will be using RViz to create your scenes, queries and start states. The steps here was already created before, **so it is only necessary if you want to create your own tests**, if it is not the case go to **section 2.1.2.**

Following the steps in *MotionPlanning*:
1. In *Context tab*, click on **Connect** to create a link with your database created by MongoDB.
2. In *Scene Objects tab*, you can upload **.stl** files that will work as obstacles where you have the option to scale and change it positions in the grid. After that, click on **Publish Scene**.
3. After defining your obstacles in the grid, go to *Stored Scenes tab* and save your scene clicking in **Save Scene**. Double click to rename it.
4. To create a query, define your start and end point in *Planning tab*, go to *Stored Scenes tab*, click on your **sceneName** and then click on **Save Query**. Double click to rename it.
5. To save a start state, define a Start State in *Planning tab*, go to *Stored States* and then click on **Save Start**.

#### **2.1.2. Load Created Scene**
This package has a created scene within queries and start states. The scene is shown bellow:

![scene](https://user-images.githubusercontent.com/32513366/64129981-d4788480-cd95-11e9-9e99-a6a9a98059d7.png)

To load it, follow this steps in *MotionPlanning*:
1. In *Context tab*, click in **Connect** to create a link with your database created by MongoDB.
2. In *Stored Scenes tab*, click on **mh_obstacles_3** and then click on **Load Scene**
3. To load a query from this scene, click on the **left arrow** in mh_obstacles_3 and click in one of the queries that appears. After that click on **Load Query**.

Here you can see the presented scene with a specific query loaded, where the start pose is in white and end pose is pink.
![scene](https://user-images.githubusercontent.com/32513366/64177595-37215d00-ce36-11e9-8cb4-a9f06771f251.png)



### **2.2 Setting your Benchmark options**
To change any configuration detail for your benchmark, modify **mh_benchmark_opts.yaml** in **config** folder. There, you will find those sections:
- **warehouse**: Default, related to MongoDB connection
- **parameters**: Here you can set the parameters for the process itself as *runs*, *timeout*, *queries and start states names*, *moveGroup name* and path definition to save(*output_directory*)
- **planner**: The set of algorithms that you want to eval.

**PS**: Probably a bug from MoveIt! Benchmarking, but all the queries will be evaluated regardless the queries that you set in **parameters**

### **2.3. Run the Benchmark**
```sh
roslaunch manipulator_h_moveit mh_benchmark.launch
```
This process can take a while to finish. After closing, will generate a series of *log files* related to specific queries in your *output_directory*.

### **2.4. Generate your Databases for analysis**

```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py <diretorio_do_log>
```

Or, if you want to **plot the results too**, you can use this command:

```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py -p <plotName> <diretorio_do_log>
```

Where **<diretorio_do_log>** is the path (where the created log is located, defined in *mh_benchmark_opts.yaml*) for one of your logs created after step **2.3**. Every query generate one log file.

**PS**: It is necessary to rename manually the **.db** generated in your *home directory*.

Inside this package, there is a folder called *db_examples* related to this step.

### **2.5. Analyze results**
Acess [Planner Arena](http://plannerarena.org/) and update your database **.db** or you can analyze the results from the second command in step 2.4.

## Doubts
For any doubt, you can get in contact through the github **issue** channel or email.
