/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "multi_lidar_calibrator.h"

#include <functional>
#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

void ROSMultiLidarCalibratorApp::PublishCloud(
	const pcl::PointCloud<PointT>::ConstPtr & in_cloud_to_publish_ptr,
	const PointCloudMsg::ConstSharedPtr & source_cloud_msg)
{
	PointCloudMsg cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header.frame_id = parent_frame_;
	// The calibrated cloud represents the child measurement, so retain its
	// timestamp even though the output frame is the parent frame.
	cloud_msg.header.stamp = source_cloud_msg->header.stamp;
	calibrated_cloud_publisher_->publish(cloud_msg);
}

void ROSMultiLidarCalibratorApp::PointsCallback(
	const PointCloudMsg::ConstSharedPtr & in_parent_cloud_msg,
	const PointCloudMsg::ConstSharedPtr & in_child_cloud_msg)
{
	pcl::PointCloud<PointT>::Ptr parent_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr child_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr child_filtered_cloud(new pcl::PointCloud<PointT>);

	pcl::fromROSMsg(*in_parent_cloud_msg, *parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *child_cloud);

	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

	DownsampleCloud(child_cloud, child_filtered_cloud, voxel_size_);

	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	ndt.setTransformationEpsilon(ndt_epsilon_);
	ndt.setStepSize(ndt_step_size_);
	ndt.setResolution(ndt_resolution_);
	ndt.setMaximumIterations(ndt_iterations_);

	ndt.setInputSource(child_filtered_cloud);
	ndt.setInputTarget(parent_cloud);

	pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

	Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
	Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());

	const Eigen::Matrix4f initial_guess =
		(init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	if (current_guess_ == Eigen::Matrix4f::Identity()) {
		current_guess_ = initial_guess;
	}

	ndt.align(*output_cloud, current_guess_);

	std::cout << "Normal Distributions Transform converged: " << ndt.hasConverged()
	          << " score: " << ndt.getFitnessScore()
	          << " likelihood: " << ndt.getTransformationLikelihood() << std::endl;

	std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

	// Transform the unfiltered input cloud using the found transform.
	pcl::transformPointCloud(*child_cloud, *output_cloud, ndt.getFinalTransformation());

	current_guess_ = ndt.getFinalTransformation();

	const Eigen::Matrix3f rotation_matrix = current_guess_.block(0, 0, 3, 3);
	const Eigen::Vector3f translation_vector = current_guess_.block(0, 3, 3, 1);
	std::cout << "This transformation can be replicated using:" << std::endl;
	std::cout << "ros2 run tf2_ros static_transform_publisher "
	          << translation_vector.transpose() << " "
	          << rotation_matrix.eulerAngles(2, 1, 0).transpose() << " " << parent_frame_
	          << " " << child_frame_ << std::endl;

	std::cout << "Corresponding transformation matrix:" << std::endl
	          << std::endl << current_guess_ << std::endl << std::endl;

	PublishCloud(output_cloud, in_child_cloud_msg);
}

void ROSMultiLidarCalibratorApp::DownsampleCloud(
	pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
	pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
	double in_leaf_size)
{
	pcl::VoxelGrid<PointT> voxelized;
	voxelized.setInputCloud(in_cloud_ptr);
	voxelized.setLeafSize(
		static_cast<float>(in_leaf_size), static_cast<float>(in_leaf_size),
		static_cast<float>(in_leaf_size));
	voxelized.filter(*out_cloud_ptr);
}

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
	: Node(kNodeName),
	  voxel_size_(declare_parameter<double>("voxel_size", 0.1)),
	  ndt_epsilon_(declare_parameter<double>("ndt_epsilon", 0.01)),
	  ndt_step_size_(declare_parameter<double>("ndt_step_size", 0.1)),
	  ndt_resolution_(declare_parameter<double>("ndt_resolution", 1.0)),
	  initial_x_(declare_parameter<double>("x", 0.0)),
	  initial_y_(declare_parameter<double>("y", 0.0)),
	  initial_z_(declare_parameter<double>("z", 0.0)),
	  initial_roll_(declare_parameter<double>("roll", 0.0)),
	  initial_pitch_(declare_parameter<double>("pitch", 0.0)),
	  initial_yaw_(declare_parameter<double>("yaw", 0.0)),
	  ndt_iterations_(declare_parameter<int>("ndt_iterations", 400)),
	  current_guess_(Eigen::Matrix4f::Identity())
{
	const auto points_parent_topic =
		declare_parameter<std::string>("points_parent_src", "points_raw");
	const auto points_child_topic =
		declare_parameter<std::string>("points_child_src", "points_raw");

	RCLCPP_INFO(get_logger(), "[%s] points_parent_src: %s", kNodeName, points_parent_topic.c_str());
	RCLCPP_INFO(get_logger(), "[%s] points_child_src: %s", kNodeName, points_child_topic.c_str());
	RCLCPP_INFO(get_logger(), "[%s] voxel_size: %.2f", kNodeName, voxel_size_);
	RCLCPP_INFO(get_logger(), "[%s] ndt_epsilon: %.2f", kNodeName, ndt_epsilon_);
	RCLCPP_INFO(get_logger(), "[%s] ndt_step_size: %.2f", kNodeName, ndt_step_size_);
	RCLCPP_INFO(get_logger(), "[%s] ndt_resolution: %.2f", kNodeName, ndt_resolution_);
	RCLCPP_INFO(get_logger(), "[%s] ndt_iterations: %d", kNodeName, ndt_iterations_);
	RCLCPP_INFO(
		get_logger(), "[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f",
		kNodeName, initial_x_, initial_y_, initial_z_, initial_roll_, initial_pitch_, initial_yaw_);

	calibrated_cloud_publisher_ =
		create_publisher<PointCloudMsg>("/points_calibrated", rclcpp::QoS(1));

	cloud_parent_subscriber_ = std::make_unique<message_filters::Subscriber<PointCloudMsg>>();
	cloud_parent_subscriber_->subscribe(this, points_parent_topic, rmw_qos_profile_sensor_data);
	RCLCPP_INFO(get_logger(), "[%s] Subscribing to... %s", kNodeName, points_parent_topic.c_str());

	cloud_child_subscriber_ = std::make_unique<message_filters::Subscriber<PointCloudMsg>>();
	cloud_child_subscriber_->subscribe(this, points_child_topic, rmw_qos_profile_sensor_data);
	RCLCPP_INFO(get_logger(), "[%s] Subscribing to... %s", kNodeName, points_child_topic.c_str());

	cloud_synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
		SyncPolicyT(100), *cloud_parent_subscriber_, *cloud_child_subscriber_);
	cloud_synchronizer_->registerCallback(
		std::bind(
			&ROSMultiLidarCalibratorApp::PointsCallback, this,
			std::placeholders::_1, std::placeholders::_2));

	RCLCPP_INFO(
		get_logger(), "[%s] Publishing PointCloud to... /points_calibrated", kNodeName);
	RCLCPP_INFO(get_logger(), "[%s] Ready. Waiting for data...", kNodeName);
}
