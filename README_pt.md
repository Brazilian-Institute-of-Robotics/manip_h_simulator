# Pacote MoveIt! para o Manipulator-H

O **objetivo** deste pacote, chamado de **manipulator_h_moveit**, é proporcionar o controle *simulado* do Manipulator-H da ROBOTIS utilizando o MoveIt!

![mhmi](https://user-images.githubusercontent.com/32513366/62636701-682e7080-b910-11e9-9602-45f32902fac9.png)

**Fonte:** [ROBOTIS](http://www.robotis.us/robotis-manipulator-h/) and [MoveIt! Motion Planning Framework](https://moveit.ros.org/)

## **Requisitos**

Esse pacote apresenta os seguintes requisitos básicos necessários para sua máquina:

- versão ROS: **Kinetic**
- versão de sistema: **Ubuntu 16.04**
- versão Gazebo: **7.0**

Além disso, faz-se necessário a presença do **MoveIt! 1.0** e **MongoDB** instalados em seu computador:

### MoveIt!
```sh    
sudo sudo apt-get install ros-kinetic-moveit
```
### MongoDB
Para instalar a versão completa, siga os seguintes passos abaixo em um terminal.
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

Para mais informações sobre a instalação do MongoDB acesse [aqui](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/)

## Dependências de outros pacotes
Além do **manipulator_h_moveit**, seu catkin workspace precisa incluir, na pasta *src*, os seguintes pacotes:
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

## Dependências do MoveIt! para o controle
Os pacotes abaixos são necessários para aplicações usando o MoveIt! em conjunto com ROS e o Gazebo e devem ser **instalados no diretório do ROS**.

- **ROS Control**: Conjunto de pacotes que incluem interfaces de controle, gerenciadores de controle, transmissores e hardware_interfaces.

- **Gazebo ROS Control**: Pacote para integrar a arquitetura de controle do *ros_control* com o simulador Gazebo.

- **Controller Manager**: O *controller_manager* proporciona um loop em tempo real-compatível com o mecanismo do robô.

- **Joint Trajectory Controller**: Controle para executar trajetórias junta-espaço em um grupo de juntas determinadas.

- **Joint State Controller**: Pacote de controle para publicar o estado da(s) junta(s).

- **Position Controllers**: Pacote de controle que responde a partir das posições.

Para instala-los, digite em um terminal:
```sh
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-gazebo-ros-control ros-kinetic-controller-manager ros-kinetic-joint-trajectory-controller ros-kinetic-joint-state-controller
ros-kinetic-position-controllers
```
Ao final, já na sua workspace *catkin_ws*
```sh
rosdep install --from-paths src --ignore-src -r -y
```
```sh
catkin_make # or catkin build (choose the one you're using it)
```


## **Uso do pacote**

### **1. Go to Goal (vá para uma posição)**

Para levantar a simulação e o back-end do MoveIt!

```sh
roslaunch manipulator_h_moveit mh_deploy_gazebo_moveit.launch
```

Para funcionar a aplicação go to goal (vá para o destino)

```sh
rosrun manipulator_h_moveit mh_goToGoal.py
```

### **2. MoveIt! Benchmark**
Para saber mais sobre como funciona o MoveIt! Benchmarking e entender alguns termos como query, log e start state além aprender como cria-los, siga os tutoriais [aqui](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/benchmarking/benchmarking_tutorial.html).

Para avaliar os planejadores, gerando conjuntos de logs:

```sh
roslaunch manipulator_h_moveit mh_benchmark.launch
```
Para gerar o banco de dados
```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py diretorio_do_log
```
Onde **diretorio_do_log** é o caminho (onde se localiza em sua máquina) para um dos logs criados pelo comando anterior (esse caminho está definido no arquivo *mh_benchmark_opts.yaml*) e deve ser passado como argumento no comando acima.

Para avaliar seus resultados acesse [Planner Arena](http://plannerarena.org/), e faça um update do seu banco de dados com terminação **db**.

**OBS:** Para alterar as opções gerais do benchmark como os planejadores que serão avaliados (apenas OMPL implementado), altere *mh_benchmark_opts.yaml* na pasta config.




