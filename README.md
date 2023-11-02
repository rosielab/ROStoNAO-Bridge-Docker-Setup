
## Steps to connect over ROS

Below are steps to setting up a naoqi_driver ros node on your computer and connecting it to the Pepper robot.
### 1. Setting up your computer

#### Operating System and CPU Architecture
I am currently running all this on a Ubuntu VM on an ARM based Mac machine. The operating system should not matter as the ROS bridge is run in a docker container. However, depending on the architecture of your CPU may need to switch the base ros image the Docker image is based on.

To do this simply switch the FROM line that is commented in the Dockerfile
```
FROM arm64v8/ros:noetic-robot
# FROM amd64/ros:noetic-robot

```


#### Installing Docker
To avoid having to install ROS and all the dependencies I am running the ROS bridge in a docker container. However, that means that to run the bridge yourself you will need to install Docker. Follow these instructions [here](https://www.docker.com/get-started/) for that.

### 2. Build the Docker Image

Once you have docker installed. You will need to build the image from this repo. 

The image can be built using the docker build command. We also tag the image with a name. NOTE: this must be done from within the directory that contains the Dockerfile.
```
docker build -t my-ros-image .
```
Once the build process is done you should be able to see the image you created after running 
```
docker image ls
```

### 3. Run Docker Image
The next step is to run the docker image in interactive mode. We run this in interactive mode so we are able to run ros command from the command line.

To run the image, use the following command.
```
docker run -it my-ros-image
```
You should then be able to interact with the image via the command line.

### 4. Build Catkin Project
Next, you will need to build the ros-nao-bridge ros node from it's source code. To do this we need to source ros, build catkin project, and then source the devel/setup.bash script.
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
catkin_make
source ./devel/setup.bash
```

### 5. Turn off Automated Life
In order to control the Pepper movements via ROS you will want to shutoff Peppers automated life (the system that makes pepper look around and idle). To do this ssh into your Pepper robot and then run the commands.

```
ssh nao@<naoip>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```
### 6. Launch Naoqi Driver
Next you need to launch the ROS bridge. This will launch a ros_score along with the naoqi_driver node. This node will be the point of contact with the Pepper robot.

To do this we use a roslaunch command. This command requires many arguments that you will need to fill yourself. Here is a quick explanation of what they are:
- nao_ip: This is the IP address of the Pepper robot. 
- nao_port: This is the port that the bridge will need to connect to on the robot. Lots of online resource say this port is 9559. However, I found that my particular robot was listening on 9503. The best way to find this is to use the Robot Viewer on Android studio. It should display the port needed.
- roscore_ip: This is the IP address of the machine that is running your ros_core. If you don't run your own ros_core and instead let roslaunch do that for you then just keep it as the localhost IP.
- network_interface: This is the network_interface of the Pepper robot. You can find this by running `ifconfig` in an ssh shell connected to the Pepper robot.
- username/password: These are the credentials for your Pepper robot. They should both be "nao".

```
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=10.0.0.4 nao_port:=9503 roscore_ip:=127.0.0.1 network_interface:=eth0 username:=nao password:=nao
```

### 7. Test the bridge

The final step is to test out the connection made by the naoqi_driver node. 

To do this you will need another terminal connected to the docker image to run your commands in. In a new terminal, find the docker container's id and then use docker exec to start a bash instance attached to that container.
```
docker ps
# copy container id
docker exec -it <container-id> bash
```

Now that you are connected to the container, you can get the info on the naoqi_driver after sourcing the ROS setup.
```
source /opt/ros/noetic/setup.bash
rosnode info /naoqi_driver
```

Finally, if all of that looks okay, you can actually publish joint data to the Pepper robot to turn its head. The following line should make the robot move its head slightly. Try switching up the numbers to see different results.
```
rostopic pub /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "{header: {seq: 0, stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5,0.1], speed: 0.1, relative: 0}"
```

### 8. You're Done!
You should now be able to publish and read messages from the Pepper robot through ROS! For more information on how to use this bridge check out the below links:

https://github.com/ros-naoqi/naoqi_driver
https://wiki.ros.org/noetic# ROStoNAO-Bridge-Docker-Setup
