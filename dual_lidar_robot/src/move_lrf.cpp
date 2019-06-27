#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

sensor_msgs::PointCloud2 cloud_front, cloud_back;
std::string laser_front_name = "hokuyo_front_link",
            laser_back_name = "hokuyo_back_link";

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

auto main(int argc, char **argv) -> int {
  ros::init(argc, argv, "synthesis_laser");
  ros::NodeHandle n;

  auto scan_front_sub =
      n.subscribe("/rrbot/laser/scan_front", 10, transformLaser);
  auto scan_back_sub =
      n.subscribe("/rrbot/laser/scan_back", 10, transformLaser);
  auto scan_base_pub = n.advertise<sensor_msgs::PointCloud2>("scan_base", 10);

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
