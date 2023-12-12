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
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"
#include "ik_service/PoseIK.h"

ros::ServiceClient material_locations_client;
ros::ServiceClient ik_client;
std::vector<osrf_gear::Order> orderVector;
osrf_gear::LogicalCameraImage::ConstPtr cameraVector[10]; 
tf2_ros::Buffer tfBuffer;

//lab 7
sensor_msgs::JointState joint_states;
static const double THROTTLE = 10;
std::string currentJointStatesName;
ik_service::PoseIK ik_pose;


double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];
trajectory_msgs::JointTrajectory desired;
ros::Publisher trajectory_pub;


trajectory_msgs::JointTrajectory get_arm_trajectory(geometry_msgs::Point dest) {
	// Where is the end effector given the joint angles.
	// joint_states.position[0] is the linear_arm_actuator_joint
	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];

	ur_kinematics::forward((double*) &q_pose, (double*) &T_pose);
	
	T_des[0][3] = (double) dest.x;
	T_des[1][3] = (double) dest.y;
	T_des[2][3] = (double) dest.z;
	T_des[3][3] = 1.0;

	T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
	T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
	T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
	T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
	
	
	int num_sols = ur_kinematics::inverse((double*) &T_des, (double*) &q_des);
	
       trajectory_msgs::JointTrajectory joint_trajectory;
       static int count = 0;
       joint_trajectory.header.seq = count++;
       joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); // When was this message created.
       joint_trajectory.header.frame_id = "/world";                           // Frame in which this is specified

       // Set the names of the joints being used. All must be present.
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
      joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

      joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
      //joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
      
      
	if (num_sols == 0) {
		ROS_WARN("NO INITIAL SOLUTIONS FOUND");
		joint_trajectory.header.frame_id = "empty";
        return joint_trajectory;
	}
	
	//WHAT IS THE POSE
	//ik_service::PoseIK ik_pose;
	//geometry_msgs::Pose part_pose;
	//part_pose.position.x = dest.x;
	//part_pose.position.y = dest.y;
	//part_pose.position.z = dest.z;
	//ik_pose.request.part_pose = part_pose;
	
	//if (ik_client.call(ik_pose)) {
          //ROS_INFO("Call to ik_service returned [%i] solutions", ik_pose.response.num_sols + 1);
          //ROS_INFO("One set of joint angles:");
          //ROS_INFO_STREAM(ik_pose.response.joint_solutions[0]);
          //trajectory_msgs::JointTrajectory joint_trajectory = trajectoryInnit();
	
	//if (ik_pose.response.num_sols == 0) {
		//ROS_WARN("NO INITIAL SOLUTIONS FOUND");
		//joint_trajectory.header.frame_id = "empty";
        //return joint_trajectory;
	//}
	
	
	// Set the start point to the current position of the joints from joint_states.
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	
	// Enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
        joint_trajectory.points[1].positions[indy + 1] = ik_pose.response.joint_solutions[0].joint_angles[indy];
       }

       joint_trajectory.points[1].time_from_start = ros::Duration(5.0);
	
	return joint_trajectory;  
}




void processOrder() {
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
										
          geometry_msgs::TransformStamped transformStamped;
	
          try {
            transformStamped = tfBuffer.lookupTransform("arm1_base_link", std::string("logical_camera_"+bin+"_frame"), ros::Time(0.0), ros::Duration(1.0));
            ROS_DEBUG("Transform to [%s] from [%s]", transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str());
		
            } catch (tf2::TransformException &ex) {
              ROS_ERROR("%s", ex.what());
            }

            geometry_msgs::PoseStamped part_pose, goal_pose;

            part_pose.pose = model.pose;
            tf2::doTransform(part_pose, goal_pose, transformStamped);
	
            goal_pose.pose.position.z += 0.10; 
            goal_pose.pose.orientation.w = 0.707;
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.707;	
            goal_pose.pose.orientation.z = 0.0;
	
            tf2::doTransform(part_pose, goal_pose, transformStamped);
            geometry_msgs::Point positionArm = goal_pose.pose.position;
            ROS_WARN("(Arm Reference) Type: [%s], Bin: [%s], Pose: [%f] [%f] [%f]", product.type.c_str(), bin.c_str(), positionArm.x, positionArm.y, positionArm.z);
	    
	    //lab 7
	    // Test point
	    geometry_msgs::Point test;
	    test.x = -0.4;
	    test.y = 0.0;
	    test.z = 0.2;
	
	    trajectory_msgs::JointTrajectory joint_trajectory = get_arm_trajectory(test);
	
	    if (joint_trajectory.header.frame_id != "empty") {
	      ROS_INFO("The arm is in motion");
	      trajectory_pub.publish(joint_trajectory);
	    }				
	    //break;
          }
        }				
    }
  }
  orderVector.erase(orderVector.begin());
}

void orderCallback(const osrf_gear::Order msg) {
	orderVector.push_back(msg);
	ROS_INFO("Order has been recived");
	processOrder();
}

void jointCallback(const sensor_msgs::JointState msg) {
  joint_states = msg;
  //ROS_INFO_STREAM_THROTTLE(2, joint_states.position[0]);
  //ROS_INFO_STREAM_THROTTLE(2, joint_states.position[1]);
  //ROS_INFO_STREAM_THROTTLE(2, joint_states.position[2]);
  std::string str;
  for (std::string s : joint_states.name) {
        str = str + " " + s + " ";
  }
  currentJointStatesName = str;

}
void cameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& cameraImage) {
	cameraVector[index] = cameraImage;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_entry");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
    
  ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderCallback);
  material_locations_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  ros::Subscriber jointSub = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1000, jointCallback);
  trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 1000);
  //lab 7
  ik_client = n.serviceClient<ik_service::PoseIK>("pose_ik");
  //ros::service::waitForService(ik_service, 5000);
  
  tf2_ros::TransformListener tfListener(tfBuffer);

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
  
  ros::Rate loop_rate(1);
  while (ros::ok) {
      ROS_INFO_STREAM_THROTTLE(THROTTLE, currentJointStatesName);
      loop_rate.sleep();
    }
    	return 0;
}
