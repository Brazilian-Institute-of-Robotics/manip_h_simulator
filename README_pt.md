# Pacote MoveIt! para o Manipulator-H

O **objetivo** deste pacote, chamado de **manipulator_h_moveit**, é proporcionar o controle simulado do Manipulator-H da ROBOTIS utilizando o MoveIt!

![mhmi](https://user-images.githubusercontent.com/32513366/63880395-078cd380-c9a4-11e9-8364-3731cf085654.png)

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
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 2930ADAE8CAF5059EE73BB4B58712A2291FA4AD5
```
```sh    
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu xenial/mongodb-org/3.6 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-3.6.list
```
```sh    
sudo apt-get update 
```
```sh    
sudo apt-get install -y mongodb-org
```

Para mais informações sobre a instalação do MongoDB acesse [aqui](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/)

## Dependências de outros pacotes
Além do **manipulator_h_moveit**:

    ```sh
    git clone -b develop https://github.com/Brazilian-Institute-of-Robotics/manipulator_h_moveit.git
    ```
seu catkin workspace precisa incluir, na pasta *src*, os seguintes pacotes:
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

Em sua pasta **src** instale uma das dependências do pacote **warehouse-ros-mongo**, necessária para poder utilizar o comando de construção do catkin, como é proposto nesse [link](https://github.com/ros-planning/warehouse_ros_mongo):
```sh
git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git
```
```sh
sudo apt-get install scons
```
```sh
cd mongo-cxx-driver
```
```sh
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
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

Para rodar a aplicação *go to goal* (vá para o destino)

```sh
rosrun manipulator_h_moveit mh_goToGoal.py
```

Através do ```mh_goToGoal.py``` o usuário pode controlar o manipulador de diversas formas estabelecidas:
- Definir posição nas juntas (estabelecido no código)
- Enviar para uma posição pré definida durante o MoveIt! Setup Assistant
- Definir uma coordenada XYZ qualquer, pelo usuário, possível no espaço de trabalho do manipulador

### **2. MoveIt! Benchmark**
MoveIt! Benchmarking ajuda o pesquisador a definir o melhor planejador de trajetória dado os parâmetros, ambiente e objetivos. Para uma melhor compreensão dessa ferramenta, alguns termos precisam ser esclarecidos:
- **planner**: Algoritmo capaz de calcular uma trajetória entre dois pontos para o manipulador.
- **start state**: Posição onde o robô começa bem no início. Usado para evitar cenários de colisão.
- **scene**: Ambiente propriamente dito, nele contém o robô e os obstáculos (opcional).
- **query**: Um caminho definido por dois pontos. Vale lembrar que uma *query* possui informação sobre a *scene* em que ela foi criada, portanto ela está contida em uma *scene*. 
- **runs**: Valor numérico. Diz o quanto de tentativas terá um planejador para calcular a trajetória para um *query*.
- **timeout**:  Valor numérico. Diz o tempo máximo (em segundo) permitido para o planejador encontrar uma solução.

Para mais informações sobre essa ferramenta e outros tutoriais (em inglês), você pode acessar [aqui](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/benchmarking/benchmarking_tutorial.html).

### **2.1. Definindo seu ambiente para avaliar um Benchmark**

Rode em um terminal **demo.launch**:

```sh
roslaunch manipulator_h_moveit demo.launch db:=true
```

#### **2.1.1. Crie sua Scene!**
Aqui, utilizaremos o RViz para criar as *scenes*, *queries* e *start states*. Esses passos já foram realizados anteriormente, **então apenas os siga caso queira aplicar seus próprios testes em ambientes próprios**, caso não queira repeti-los pule para a seção **2.1.2.**.

Seguindo os passos na aba *MotionPlanning*:
1. Na aba *Context tab*, clique em **Connect** para criar um link com o database criado através do MongoDB.
2. Na aba *Scene Objects*, você pode importar arquivos **.stl** para funcionar como obstáculos, onde você possui a opção de aumentar a escala do mesmo e de move-lo pelo ambiente. Após isso, clique em **Publish Scene**.
3. Após definir seus obstáculos no ambiente, vá para a aba *Stored Scenes* e salve sua *scene* clicando em **Save Scene**. Para renomear, clique duplo em cima da *scene* recém criada.
4. Para criar um *query*, defina a posição de inicio/fim na aba *Planning tab*, vá para a aba *Stored Scenes*, clique na sua *scene* criada e então clique em **Save Query**. Para renomear, clique duplo em cima da *query* salva.
5. Para salvar o *start state*, defina-o na aba *Planning*, vá para *Stored States* e então clique em **Save Start**. Para renomear, clique duplo em cima do *start state* salva.

#### **2.1.2. Carregar sua Scene!**
Como dito, o passo **2.1.1.** possui uma *scene* já definida com *queries* e *start states* demonstrado abaixo:

![scene](https://user-images.githubusercontent.com/32513366/64129981-d4788480-cd95-11e9-9e99-a6a9a98059d7.png)

Para carrega-la, siga os passos em *MotionPlanning*:
1. Na aba *Context*, clique em **Connect** para criar um link entre o database e sua aplicação.
2. Na aba *Stored Scenes*, clique em **mh_obstacles_3** e então clique em **Load Scene**
3. Para carregar as *queries* da *scene*, clique na seta esquerda em **mh_obstacles_3** e então clique em uma das *queries*. Depois disso, clique em **Load Query**.

Aqui tem um exemplo de uma *query* carregada no ambiente, onde em branco temos seu início e em rosa sua posição final (destino):
![scene](https://user-images.githubusercontent.com/32513366/64177595-37215d00-ce36-11e9-8cb4-a9f06771f251.png)

### **2.2 Configurações de opção para o Benchmark**
Para mudar detalhes de configuração para o *benchmark*, modifique **mh_benchmark_opts.yaml** na pasta **config**. Lá, você encontrará essas seções principais:
- **warehouse**: Padrão. Relacionado a conexão com o MongoDB.
- **parameters**: Aqui padrões gerais de configuração do processo são encontrados como *runs*, *timeout*, *queries and start states*, *moveGroup name* (relacionado a modelagem via MoveIt!) and e a definição de diretório para salvar os resultados (*output_directory*).
- **planner**: Conjunto de planejadores que você irá avaliar.

**PS**: Provavelmente um bug do MoveIt! Benchmarking, mas todas as *queries* avaliadas independente se foram definidas em **parameters**.

### **2.3. Rode o Benchmark**
```sh
roslaunch manipulator_h_moveit mh_benchmark.launch
```
Esse processo, a depender das configurações, pode demorar bastante. Após seu encerramento, ele irá gerar uma serie de arquivos *log* relacionado a *queries* específicas em seu diretório de salvamento *output_directory*.

### **2.4. Gerar banco de dados para análises futuras**

```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py <diretorio_do_log>
```
Exemplo:
```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py /tmp/moveit_benchmarks/manipulator_h/benchMH_pHD_pb2pa_kwr_2019-09-10T18:52:16.929276.log
```

Ou, se deseja **fazer os gráficos dos resultados também**, pode usar em um terminal o comando:

```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py -p <plotName> <diretorio_do_log>
```
Example:
```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py -p benchResults /tmp/moveit_benchmarks/manipulator_h/benchMH_pHD_pb2pa_kwr_2019-09-10T18:52:16.929276.log
```

Onde **<diretorio_do_log>** é o caminho para os arquivos *log*(definido anteriormente). Cada *query* possui um arquivo *log* específico. Então cada arquivo gerado  pelos comandos acima são referentes a uma *query* específica.

**PS**: É necessário alterar manualmente o nome do arquivo **.db** gerado em sua Home.

**PS**: Dentro deste pacote, existe uma pasta chamada *db_examples* relacionada a esta etapa.

### **2.5. Analyze results**
Para analisar os resultados gerados no arquivo **.db** acesse [Planner Arena](http://plannerarena.org/) e *update* seu arquivo para gerar plot iterativos.

## Doubts
Para qualquer dúvida, fique livre para entrar em contato (e-mail ou linkedIn) ou criar um **issue**.
