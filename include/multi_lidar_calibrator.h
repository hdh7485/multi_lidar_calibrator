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

#ifndef PROJECT_MULTI_LIDAR_CALIBRATOR_H
#define PROJECT_MULTI_LIDAR_CALIBRATOR_H

#include <memory>
#include <string>

#include <Eigen/Core>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ROSMultiLidarCalibratorApp : public rclcpp::Node
{
public:
	ROSMultiLidarCalibratorApp();

private:
	static constexpr const char * kNodeName = "multi_lidar_calibrator";

	using PointT = pcl::PointXYZI;
	using PointCloudMsg = sensor_msgs::msg::PointCloud2;
	using SyncPolicyT = message_filters::sync_policies::ApproximateTime<
		PointCloudMsg, PointCloudMsg>;

	void PointsCallback(
		const PointCloudMsg::ConstSharedPtr & in_parent_cloud_msg,
		const PointCloudMsg::ConstSharedPtr & in_child_cloud_msg);

	void DownsampleCloud(
		pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
		pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
		double in_leaf_size);

	void PublishCloud(
		const pcl::PointCloud<PointT>::ConstPtr & in_cloud_to_publish_ptr,
		const PointCloudMsg::ConstSharedPtr & source_cloud_msg);

	double voxel_size_;
	double ndt_epsilon_;
	double ndt_step_size_;
	double ndt_resolution_;

	double initial_x_;
	double initial_y_;
	double initial_z_;
	double initial_roll_;
	double initial_pitch_;
	double initial_yaw_;

	int ndt_iterations_;

	std::string parent_frame_;
	std::string child_frame_;

	Eigen::Matrix4f current_guess_;

	rclcpp::Publisher<PointCloudMsg>::SharedPtr calibrated_cloud_publisher_;

	// Destroy the synchronizer before the subscribers it observes.
	std::unique_ptr<message_filters::Subscriber<PointCloudMsg>> cloud_parent_subscriber_;
	std::unique_ptr<message_filters::Subscriber<PointCloudMsg>> cloud_child_subscriber_;
	std::unique_ptr<message_filters::Synchronizer<SyncPolicyT>> cloud_synchronizer_;
};

#endif  // PROJECT_MULTI_LIDAR_CALIBRATOR_H
