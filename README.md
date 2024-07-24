# RoboSapiensAdaptivePlatform-turtlebotsim

Run the Turtlebot4 gazebo simulator using Robosapiense Adaptive Platform 

# RUN The gazebo simulator using Docker file 

```bash 
cd docker 
```
if you have nvidia graphic use:

```bash 
docker compose up simdeploynvidia
```
otherwise: 
```bash 
docker compose up simdeploymesa
```
Then use ctrl+shift+P and run "Rebuild and Reopen in container" to run the code in the dev container 

In terminal run : 
```bash 
python3 RaP_Lidar_Occlusion.py
```


# Step1: RUN The Robot or simulator whithout Docker 

Run the simulation using this command 

```sh
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze slam:=true nav2:=true rviz:=true
```
Or turn on the real robot 


### Auxiullary ROS2 node on the robot 
scan_safe and spin functions are working using this nodes 

```sh
cd scan_relay_ws
```
```sh
rosdep install -i --from-path spin_interfaces --rosdistro humble
```
```sh
colcon build --packages-select spin_interfaces
```

```sh
 rosdep install -i --from-path demo_bringup --rosdistro humble
 rosdep install -i --from-path scan_modifier --rosdistro humble
 rosdep install -i --from-path spinning_controller --rosdistro humble
rosdep install -i --from-path topic_param_bridge --rosdistro humble
```

```sh
colcon build --packages-select demo_bringup scan_modifier spinning_controller topic_param_bridge

```
Run all auxiullary nodes: 
```sh
cd scan_relay_ws
source install/setup.bash
ros2 launch demo_bringup real_demo_tb4.launch.py
```
### oclude the lidar on simulator 

docker exec container name! 
```sh
ros2 topic pub --once /scan_config std_msgs/msg/UInt16MultiArray "{data:[300, 800]}"
```

#### Useful Notes
If you can't see the robot ros topics:

```sh
export ROS_DOMAIN_ID=0
```
Useful Termonal commands for real robot: 
```sh
ros2 launch turtlebot4_navigation slam.launch.py sync:=false

```
```sh
ros2 launch turtlebot4_viz view_robot.launch.py
```

```sh
ros2 launch turtlebot4_navigation nav2.launch.py

```

Useful ROS2 commands to remember: 

- Read IP: ros2 topic echo /ip
- Read LiDAR data raw: ros2 topic echo /scan
- See topics being published: ros2 topic list
- Undock robot: ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
- Dock robot: ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"


## Step2: RUN The MQTT broker and ROS2MQTTBridge   

Run the simulation using this command 

```sh
sudo service emqx start
source scan_relay_ws/install/setup.bash
python3 ROS2MQTTBridge/ROS2MqttBridge.py 
```

## Step3: RUN The Robosaoiense Adaptive Platform 

```sh
sudo service emqx start
python3 Rap_Lidar_Occlusion.py 
```
