#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

std::string laser_front_name = "hokuyo_front_link",
            laser_back_name = "hokuyo_back_link";

sensor_msgs::PointCloud2 cloud_front, cloud_back;
void transformLaser(const sensor_msgs::LaserScan &data) {
  int max_size = (data.angle_max - data.angle_min) / data.angle_increment;
  pcl::PointCloud<pcl::PointXYZ> dummy_cloud;
  dummy_cloud.width = max_size;
  dummy_cloud.height = 1;
  dummy_cloud.is_dense = false;
  dummy_cloud.points.resize(max_size); // dummy_cloud.width * dummy_cloud.height
  for (int i = 0; i < max_size; ++i) {
    dummy_cloud.points[i].x =
        data.ranges[i] * cos(i * data.angle_increment + data.angle_min);
    dummy_cloud.points[i].y =
        data.ranges[i] * sin(i * data.angle_increment + data.angle_min);
    dummy_cloud.points[i].z = 0.07;
  }

  if (data.header.frame_id == laser_front_name) {
    pcl::toROSMsg(dummy_cloud, cloud_front);
    cloud_front.header = data.header;
  } else if (data.header.frame_id == laser_back_name) {
    pcl::toROSMsg(dummy_cloud, cloud_back);
    cloud_back.header = data.header;
  }
}

std_msgs::Float64 laser_front_high, laser_back_high;
void checkLaserPosition(const sensor_msgs::JointState &data) {
  for (std::size_t i = 0; i < data.name.size(); ++i) {
    if (data.name[i] == "hokuyo_joint_front") {
      laser_front_high.data = data.position[i];
    } else if (data.name[i] == "hokuyo_joint_back") {
      laser_back_high.data = data.position[i];
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "synthesis_laser");
  ros::NodeHandle n;

  auto scan_front_sub =
      n.subscribe("/rrbot/laser/scan_front", 10, transformLaser);
  auto scan_back_sub =
      n.subscribe("/rrbot/laser/scan_back", 10, transformLaser);
  auto scan_base_pub = n.advertise<sensor_msgs::PointCloud2>("scan_base", 10);

  auto joint_states_laser_sub =
      n.subscribe("joint_states", 10, checkLaserPosition);
  auto laser_front_high_pub = n.advertise<std_msgs::Float64>(
      "/my_robo/laser_front_controller/command", 10);
  auto laser_back_high_pub = n.advertise<std_msgs::Float64>(
      "/my_robo/laser_back_controller/command", 10);

  std::string target_name = "base_link";
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tr_base, tr_laser_front, tr_laser_back;
  tf2::Quaternion laser_q;

  tr_base.header.frame_id = "base_footprint";
  tr_base.child_frame_id = target_name;
  laser_q.setRPY(0, 0, 0);
  tr_base.transform.rotation.x = laser_q.x();
  tr_base.transform.rotation.y = laser_q.y();
  tr_base.transform.rotation.z = laser_q.z();
  tr_base.transform.rotation.w = laser_q.w();
  tr_base.transform.translation.x = 0;
  tr_base.transform.translation.y = 0;
  tr_base.transform.translation.z = 0.1;

  tr_laser_front.header.frame_id = target_name;
  tr_laser_front.child_frame_id = laser_front_name;
  laser_q.setRPY(0, 0, 0);
  tr_laser_front.transform.rotation.x = laser_q.x();
  tr_laser_front.transform.rotation.y = laser_q.y();
  tr_laser_front.transform.rotation.z = laser_q.z();
  tr_laser_front.transform.rotation.w = laser_q.w();
  tr_laser_front.transform.translation.x = 0.2;
  tr_laser_front.transform.translation.y = 0;
  tr_laser_front.transform.translation.z = 0.07;

  tr_laser_back.header.frame_id = target_name;
  tr_laser_back.child_frame_id = laser_back_name;
  laser_q.setRPY(0, 0, M_PI);
  tr_laser_back.transform.rotation.x = laser_q.x();
  tr_laser_back.transform.rotation.y = laser_q.y();
  tr_laser_back.transform.rotation.z = laser_q.z();
  tr_laser_back.transform.rotation.w = laser_q.w();
  tr_laser_back.transform.translation.x = -0.2;
  tr_laser_back.transform.translation.y = 0;
  tr_laser_back.transform.translation.z = 0.07;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    sensor_msgs::PointCloud2 cloud_base, cloud_base_from_front,
        cloud_base_from_back;
    tr_base.header.stamp = ros::Time::now();
    br.sendTransform(tr_base);

    // Change lrf high
    laser_front_high_pub.publish(laser_front_high);
    laser_back_high_pub.publish(laser_back_high);
    tr_laser_front.transform.translation.z = laser_front_high.data;
    tr_laser_back.transform.translation.z = laser_back_high.data;
    tr_laser_front.header.stamp = ros::Time::now();
    tr_laser_back.header.stamp = ros::Time::now();
    br.sendTransform(tr_laser_front);
    br.sendTransform(tr_laser_back);

    geometry_msgs::TransformStamped tr_base_front, tr_base_back;
    try {
      tr_base_front = tfBuffer.lookupTransform(target_name, laser_front_name,
                                               ros::Time(), ros::Duration(1.0));

      tr_base_back = tfBuffer.lookupTransform(target_name, laser_back_name,
                                              ros::Time(), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try {
      tf2::doTransform(cloud_front, cloud_base_from_front, tr_base_front);
      tf2::doTransform(cloud_back, cloud_base_from_back, tr_base_back);
    } catch (std::runtime_error &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    cloud_base = cloud_base_from_front;
    cloud_base.width += cloud_base_from_back.width;
    cloud_base.row_step += cloud_base_from_back.row_step;
    cloud_base.data.insert(cloud_base.data.end(),
                           cloud_base_from_back.data.begin(),
                           cloud_base_from_back.data.end());
    scan_base_pub.publish(cloud_base);

    loop_rate.sleep();
  }
}
