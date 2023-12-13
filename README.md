# ECSE 373 Final Project (Fall 2023)
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
The next step is downloading ariac_entry (this repository) and ik_service (another repository found on this same GitHub account). Put them in the same catkin workspaces and make sure to run the following commands from the workspace level:
```
source devel/setup.bash
catkin_make ariac_entry
catkin_make ik_service
```
After the packages the required two workspaces have been installed from the previous step, the following three lines can be ran to run the package.  The first line starts the simulation, and the second line starts the node and the competition:
```
roslaunch ariac_entry entry.launch &
rosrun ariac_entry ariac_entry
```
## What the Package Does  
## Project Flow Chart
## Known Errors
The final lab seems to have all working subcomponents. This means that the proper arm movement, grabbing and dropping the gripper, moving along the beam, going to correct components, and completing an order can all be done. The problem with the final submission comes with putting all the parts together consistently and accurately. The project does not run as wanted but due to the time constraints I have done my best to put all subcomponents of the project together. Please refer to the source code to see the subcomponents and the work put in to implement them.   
  
NOTE: On occasion, the lab fails when the node is started. It is unknown why but just restart the launch process and it will work.


