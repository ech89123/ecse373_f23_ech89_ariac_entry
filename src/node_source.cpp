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

ros::ServiceClient material_locations_client;
std::vector<osrf_gear::Order> orderVector;
osrf_gear::LogicalCameraImage::ConstPtr cameraVector[10]; 

tf2_ros::Buffer tfBuffer;


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
						
          std::string sourceFrame = "logical_camera_"+bin+"_frame";				
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
	processOrder();
}

void cameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& cameraImage) {
	cameraVector[index] = cameraImage;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ariac_entry");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderCallback);
  material_locations_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

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
  

  ros::Rate loop_rate(10);
  
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
  
  ros::spin();
  return 0;
}
