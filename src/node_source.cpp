#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"

//lab 7 
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ik_service/PoseIK.h" 
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

//phase 1 for Gripper
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"

ros::ServiceClient material_locations_client; 
std::vector<osrf_gear::Order> orderVector;
osrf_gear::LogicalCameraImage::ConstPtr cameraVector[10];
tf2_ros::Buffer tfBuffer;

//lab 7
sensor_msgs::JointState joint_states;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectoryAction;
ros::ServiceClient ik_client;
std::string currentJointStatesName;
int count = 0;

//phase 1
ros::ServiceClient gripper_client;



void operateGripper(bool attach) {
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = attach;

    if (gripper_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Gripper operation %s", attach ? "Attched" : "Detached");
        } else {
            ROS_WARN("Gripper operation failed");
        }
    } else {
        ROS_ERROR("Failed to call the gripper service");
    }
}

void actionServer(trajectory_msgs::JointTrajectory joint_trajectory) {
	
  control_msgs::FollowJointTrajectoryAction joint_trajectoryAction;
	
  joint_trajectoryAction.action_goal.goal.trajectory = joint_trajectory;
  joint_trajectoryAction.action_goal.header.seq = count++;
  joint_trajectoryAction.action_goal.header.stamp = ros::Time::now();
  joint_trajectoryAction.action_goal.header.frame_id = "/world";
  joint_trajectoryAction.action_goal.goal_id.stamp = ros::Time::now();
  joint_trajectoryAction.action_goal.goal_id.id = std::to_string(count);
  
  actionlib::SimpleClientGoalState state = trajectoryAction->sendGoalAndWait(joint_trajectoryAction.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
  ROS_INFO("Action Server: [%i] %s", state.state_, state.toString().c_str());
}


void moveArm(osrf_gear::Model model, std::string frame, double polarity) {	
	
  geometry_msgs::TransformStamped transformStamped;
	
  try {
    transformStamped = tfBuffer.lookupTransform("arm1_base_link", frame, ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s]", transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str());
		
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }        
    
  geometry_msgs::PoseStamped part_pose; 
  geometry_msgs::PoseStamped goal_pose;

  part_pose.pose = model.pose;
  tf2::doTransform(part_pose, goal_pose, transformStamped);
	
  goal_pose.pose.position.z += (0.05 * polarity);	
	
  geometry_msgs::Point position = goal_pose.pose.position;
  geometry_msgs::Quaternion orientation = goal_pose.pose.orientation;
	
  ROS_WARN("(Arm Reference) Pose: [%f] [%f] [%f]", position.x, position.y, position.z);
	
  ik_service::PoseIK ik_pose; 
  ik_pose.request.part_pose = goal_pose.pose;	
  ik_client.call(ik_pose);
    
  int num_sols = ik_pose.response.num_sols;
    
  ROS_INFO("Solutions Returned: [%i]", num_sols);
	
  if (num_sols == 0) {
    return;
  }
	
  trajectory_msgs::JointTrajectory joint_trajectory;
    
  int count = 0;
  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now(); 
  joint_trajectory.header.frame_id = "/world"; 
  joint_trajectory.joint_names.clear();

  joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
  joint_trajectory.joint_names.push_back("shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("shoulder_lift_joint");
  joint_trajectory.joint_names.push_back("elbow_joint");
  joint_trajectory.joint_names.push_back("wrist_1_joint");
  joint_trajectory.joint_names.push_back("wrist_2_joint");
  joint_trajectory.joint_names.push_back("wrist_3_joint");
    
  joint_trajectory.points.resize(2);
    
  joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	
  for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < joint_states.name.size(); indz++) {
      if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
        joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
        break;
      }
    }
  }

  joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
  joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
  joint_trajectory.points[1].positions[0] = joint_states.position[1];
  joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

  double target_angle = 3.0 / 2.0 * M_PI; 
  int best_solution_index = -1;
  double best_angle = 10000; 
  for (int i = 0; i < num_sols; i++) {
    double pan_angle = ik_pose.response.joint_solutions[i].joint_angles[0];
    double shoulder_angle = ik_pose.response.joint_solutions[i].joint_angles[1];
    double wrist_1_angle = ik_pose.response.joint_solutions[i].joint_angles[3];
    
    if (abs(M_PI - pan_angle) >= M_PI / 2 || abs(M_PI - wrist_1_angle) >= M_PI / 2) {
      continue;
    }

    double dist = std::min(fabs(shoulder_angle - target_angle), 2.0 * M_PI - fabs(shoulder_angle - target_angle));
    if (dist < best_angle){
      best_angle = dist;
      best_solution_index = i;
    }
  }
  
  for (int indy = 0; indy < 6; indy++) {
    joint_trajectory.points[1].positions[indy+1] = ik_pose.response.joint_solutions[best_solution_index].joint_angles[indy];
  }
  
  
  
  
  joint_trajectory.points[1].time_from_start = ros::Duration(5.0);

  actionServer(joint_trajectory);
  ros::Duration(joint_trajectory.points[1].time_from_start).sleep();
}

void moveBase(double base_pos) {	
  trajectory_msgs::JointTrajectory joint_trajectory;  
  int count = 0;
  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now(); 
  joint_trajectory.header.frame_id = "/world"; 
  joint_trajectory.joint_names.clear();

  joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
  joint_trajectory.joint_names.push_back("shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("shoulder_lift_joint");
  joint_trajectory.joint_names.push_back("elbow_joint");
  joint_trajectory.joint_names.push_back("wrist_1_joint");
  joint_trajectory.joint_names.push_back("wrist_2_joint");
  joint_trajectory.joint_names.push_back("wrist_3_joint");
    
  joint_trajectory.points.resize(2);
    
  joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	
  for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < joint_states.name.size(); indz++) {
      if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
        joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
        break;
      }
    }
  }


  joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
  joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
  joint_trajectory.points[1].positions[0] = joint_states.position[1];
  joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
	
  for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < joint_states.name.size(); indz++) {
      if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
        joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
        joint_trajectory.points[1].positions[indy] = joint_states.position[indz];
        break;
      }
    }
  }
  
  joint_trajectory.points[1].positions[0] = base_pos;
  joint_trajectory.points[1].time_from_start = ros::Duration(5.0);
  actionServer(joint_trajectory);
}

void processOrder() {
  if (orderVector.size() == 0) {
    return;
  }

  osrf_gear::Order firstOrder = orderVector[0];

  for(osrf_gear::Shipment shipment: firstOrder.shipments) {
    for(osrf_gear::Product product:shipment.products) {	
				
      osrf_gear::GetMaterialLocations material_locations_srv;
      material_locations_srv.request.material_type = std::string(product.type);
      material_locations_client.call(material_locations_srv);
      
      std::string bin = "";
      
      for (osrf_gear::StorageUnit unit : material_locations_srv.response.storage_units){
        bin = unit.unit_id;
        break;
      }

      if (bin == "") {
        continue;
      }
      
      //Go through each of the cameras and if it does not equal std::string::npos (which means not found)
      //Then we know we have found the right camera and pass that for the images array
      
      int i = -1;
    
      if (std::string("/ariac/logical_camera_bin1").find(bin) != std::string::npos) {
        std::cout << "here";
        i = 0;
      } 
      if (std::string("/ariac/logical_camera_bin2").find(bin) != std::string::npos) {
        i = 1;
      } 
      if (std::string("/ariac/logical_camera_bin3").find(bin) != std::string::npos) {
        i = 2;
      } 
      if (std::string("/ariac/logical_camera_bin4").find(bin) != std::string::npos) {
        i = 3;
      } 
      if (std::string("/ariac/logical_camera_bin5").find(bin) != std::string::npos) {
        i = 4;
      } 
      if (std::string("/ariac/logical_camera_bin6").find(bin) != std::string::npos) {
        i = 5;
      } 
      if (std::string("/ariac/logical_camera_agv1").find(bin) != std::string::npos) {
        i = 6;
      } 
      if (std::string("/ariac/logical_camera_agv2").find(bin) != std::string::npos) {
        i = 7;
      } 
      if (std::string("/ariac/quality_control_sensor_1").find(bin) != std::string::npos) {
        i = 8;
      } 
      if (std::string("/ariac/quality_control_sensor_2").find(bin) != std::string::npos) {
        i = 9;     
      }
      
      
      for (osrf_gear::Model model : cameraVector[i]->models) {
        if (strstr(product.type.c_str(), model.type.c_str())) {
          geometry_msgs::Point positionBin = model.pose.position;
						
          ROS_WARN("Type: [%s]", product.type.c_str());
          ROS_WARN("(Camera Reference) Type: [%s], Bin: [%s], Pose: [%f] [%f] [%f]", product.type.c_str(), bin.c_str(), positionBin.x, positionBin.y, positionBin.z);
						
          std::string frame = "logical_camera_"+bin+"_frame";				

            double base_pos = 0; 
						
	    if (bin == "bin1") {
	      base_pos = -1.916;
	    } else if (bin == "bin2") {
	      base_pos = -1.15; 
	    } else if (bin == "bin3") {
	      base_pos = -0.6;
	    } else if (bin == "bin4") {
	      base_pos = 0.6;
	    } else if (bin == "bin5") {
	      base_pos = 1.15;
	    } else if (bin == "bin6") {
	      base_pos = 1.9;
	    }
						
	    ROS_INFO("Moving base the base");			
	    moveBase(base_pos); //move base	
	    moveArm(model, frame, 1); //move arm above
	    moveArm(model, frame, -1); //move arm down
	    operateGripper(true); //pick up
	    moveArm(model, frame, 1); //move arm up
	    moveArm(model, frame, -1); //move arm down 
	    operateGripper(false); //drop 
	    
	    //do again for phase 1
	    moveArm(model, frame, 1); //move arm above
	    moveArm(model, frame, -1); //move arm down
	    operateGripper(true); //pick up
	    moveArm(model, frame, 1); //move arm up
	    moveArm(model, frame, -1); //move arm down 
	    operateGripper(false); //drop 
	    
  	    break;					
          }
        }				
    }
  }
  orderVector.erase(orderVector.begin());
}

void orderCallback(const osrf_gear::Order msg) {
	orderVector.push_back(msg);
	ROS_INFO("Order has been recived");
}

void cameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& cameraImage) {
	cameraVector[index] = cameraImage;
}

void jointCallback(const sensor_msgs::JointState msg) {
	joint_states = msg;
	
	std::string current;
	
	for (std::string s : joint_states.name) {
        current += s + " ";
    }
    
	ROS_INFO_STREAM_THROTTLE(10, current.c_str()); 
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "ariac_entry");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderCallback);
  material_locations_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  tf2_ros::TransformListener tfListener(tfBuffer);
  
  //lab7
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1000, jointCallback);
  trajectoryAction = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/ariac/arm1/arm/follow_joint_trajectory/", true);
  ik_client = n.serviceClient<ik_service::PoseIK>("/pose_ik");


  //phase 1
  gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
  
  std::vector<ros::Subscriber> camera_list;

  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1", 1000, boost::bind(cameraCallback, 0, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2", 1000, boost::bind(cameraCallback, 1, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3", 1000, boost::bind(cameraCallback, 2, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4", 1000, boost::bind(cameraCallback, 3, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5", 1000, boost::bind(cameraCallback, 4, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 1000, boost::bind(cameraCallback, 5, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 1000, boost::bind(cameraCallback, 6, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 1000, boost::bind(cameraCallback, 7, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 1000, boost::bind(cameraCallback, 8, _1)));
  camera_list.push_back(n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 1000, boost::bind(cameraCallback, 9, _1)));
 
  
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  begin_client.waitForExistence();
  std_srvs::Trigger begin_comp;
  
  //starts the competition
  int service_call_succeeded;
  service_call_succeeded = begin_client.call(begin_comp);
   
  if (service_call_succeeded == 0) {
    ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  } else {
    if (begin_comp.response.success){
      ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
    } else {
      ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
    }
  }
  
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok) {
    processOrder();
    loop_rate.sleep();
  }
  
  	
  return 0;
}  

