# RoboSapiensAdaptivePlatform-turtlebotsim

Run the Turtlebot4 gazebo simulator using Robosapiense Adaptive Platform 

# RUN The gazebo simulator Docker file 

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