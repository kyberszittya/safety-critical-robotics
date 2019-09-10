/*
 * main.cpp
 *
 *  Created on: Sep 10, 2019
 *      Author: kyberszittya
 */
#include <limits>
#include <memory>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/LaneArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <monitoring_msgs/MonitorFeedbackArray.h>

class RobotFirstOrderState
{
public:
	Eigen::Vector3d pos;
	Eigen::Quaterniond ori;
};

class RobotFirstOrderStateSubscriber
{
private:
	std::unique_ptr<RobotFirstOrderState> state;
	std::shared_ptr<ros::NodeHandle> nh;
	// State subscribers
	std::string     topic_name_current_pose;
	ros::Subscriber sub_current_pose;
	std::string     topic_name_current_velocity;
	ros::Subscriber sub_current_velocity;
	// Private singleton publisher

public:
	RobotFirstOrderStateSubscriber(std::shared_ptr<ros::NodeHandle> nh): state(new RobotFirstOrderState()), nh(nh) {}

	void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		state->pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
		//ori << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
	}

	void cbCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{

	}

	void init(std::shared_ptr<ros::NodeHandle> nh)
	{
		// TODO: with parameters
		// State subscribers
		sub_current_velocity = nh->subscribe("current_velocity", 100, &RobotFirstOrderStateSubscriber::cbCurrentVelocity, this);
		sub_current_pose     = nh->subscribe("current_pose",     100, &RobotFirstOrderStateSubscriber::cbCurrentPose,     this);
	}

	void lateralDistanceBetweenPoint(geometry_msgs::PointStamped::ConstPtr& point, autoware_msgs::LaneArray::ConstPtr& lane, const int& closest_waypoint_id, double& distance)
	{
		distance = std::numeric_limits<double>::infinity();
		double lookahead_distance = 0.0;
		for (int i = closest_waypoint_id; i < lane->lanes.size(); i++)
		{
			double dx = (lane->lanes[0].waypoints[i].pose.pose.position.x - state->pos[0]);
			double dy = (lane->lanes[0].waypoints[i].pose.pose.position.y - state->pos[1]);
			double d_dist = sqrt(dx*dx - dy*dy);

			lookahead_distance += d_dist;
		}

	}

	void longitudinalDistanceBetweenPoint(geometry_msgs::PointStamped::ConstPtr& point, autoware_msgs::LaneArray::ConstPtr& lane, const int& current_lane_id, double& distance)
	{

	}

	friend class ObstacleMonitor;
};

class ObstacleMonitor
{
private:
	std::shared_ptr<ros::NodeHandle> nh;
	std::unique_ptr<RobotFirstOrderStateSubscriber> ego_state_subscriber;
	ros::Subscriber sub_detected_object_topic;
	// Publish event stream
	monitoring_msgs::MonitorFeedback event_detected_object;
	ros::Publisher pub_event_detected_object;
	// Autoware lane
public:
	ObstacleMonitor(std::shared_ptr<ros::NodeHandle> nh): nh(nh), ego_state_subscriber(new RobotFirstOrderStateSubscriber(nh)) {}



	void cbDetectedObject(const autoware_msgs::CloudClusterArray::ConstPtr& msg)
	{
		for (const auto& cluster: msg->clusters)
		{
			if (cluster.centroid_point.point.x < 14.0 && abs(cluster.centroid_point.point.y) < 2.0)
			{
				monitoring_msgs::MonitorFeedback event;
				event.eval = 1;
				event.severity = monitoring_msgs::MonitorFeedback::SEVERITY_WARNING;
				// Using lateral distance
				event.signal_id = "LANE_OBJECT_DETECTED";
				event.val = abs(cluster.centroid_point.point.y);
				pub_event_detected_object.publish(event);
			}
		}

	}

	void init()
	{
		sub_detected_object_topic = nh->subscribe("/detection/lidar_detector/cloud_clusters", 10, &ObstacleMonitor::cbDetectedObject, this);
		// Event stream
		pub_event_detected_object = nh->advertise<monitoring_msgs::MonitorFeedback>("monitor/events/detected_object", 1000);
	}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "ros_object_detection_monitor");
	std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
	ObstacleMonitor monitor(nh);
	monitor.init();
	ros::spin();
	return 0;
}
