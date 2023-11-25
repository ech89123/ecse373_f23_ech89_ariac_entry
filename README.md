# ECSE 373 Laboratory #5 (Fall 2023)
The ariac_entry package runs on  the ARIAC 2019 environment using ROS Noetic. More about ARIAC 2019 can be read here: [Link]( https://bitbucket.org/osrf/ariac/wiki/2019/Home)  

More information about the two workspaces can be found here:     
[Simulation Environment Workspace](https://github.com/cwru-eecs-373/cwru_ariac_2019.git)  
[ARIAC Node](https://github.com/cwru-eecs-373/ecse_373_ariac.git)  

## Preparing the Lab 
Create the ARIAC workspace:  
```
# Create a catkin workspace for the simulation environment
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src

# Clone the repository
git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git

# Install any missing dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build the simulator environment
cd ../

# Install the simulator environment
sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"

# Make sure your system is aware of the new package
source /opt/ros/noetic/setup.bash
```
Then create the ARIAC node workspace:  
```
# Make a workspace for the ARIAC node.
mkdir -p ~/ecse_373_ariac_ws/src
cd ~/ecse_373_ariac_ws/src

# Clone the GIT repository for this laboratory
git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git

# Install any missing dependencies
rosdep install --from-paths ecse_373_ariac --ignore-src -r -y

# Add it to your ROS environment
cd ../
catkin_make
source devel/setup.bash
```

## Launching the Lab 
After the package the required two workspaces have been installed from the previous step, the following three lines can be ran to run the package:
```
source devel/setup.bash
roscore &  
roslaunch ariac_entry entry.launch  
```
## What the Package Does  
The package starts the competition and gives a strong error message if the competition fails to contact the start_competition service. A less strong error message is sent if the contact is successful, but the competition sill is unsuccessful in starting.  
  
The package then subscribes to the Orders topic and takes in orders and uses the material_location service to find the correct bin for the first product of the first shipment of each order. The package also subscribes to all logical_cameras and stores the information.  
  
Sends a message using ROS_WARN to give the bin number and the position of the part in reference to the camera and in reference to the arm.
