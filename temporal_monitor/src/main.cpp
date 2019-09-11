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
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <monitoring_msgs/MonitorFeedbackArray.h>
#include <std_msgs/Int32.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

class RobotFirstOrderState
{
public:
	Eigen::Vector3d pos;
	Eigen::Quaterniond ori;
};

const double MAX_LATERAL_DISTANCE = 2.4;
const double MAX_LATERAL_DISTANCE_SQR = MAX_LATERAL_DISTANCE * MAX_LATERAL_DISTANCE;
const double MAX_LOOKAHEAD_DISTANCE = 15.5;
const double MAX_LOOKAHEAD_DISTANCE_SQR = MAX_LOOKAHEAD_DISTANCE * MAX_LOOKAHEAD_DISTANCE;



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
	geometry_msgs::TransformStamped _tr_base_link;
	geometry_msgs::TransformStamped _tr_velodyne;
	tf2_ros::Buffer _buffer;
	tf2_ros::TransformListener _listener;
	geometry_msgs::PoseStamped pose;
	// Pose
	ros::Publisher  pub_obstacle_lane_id;
public:
	RobotFirstOrderStateSubscriber(std::shared_ptr<ros::NodeHandle> nh): state(new RobotFirstOrderState()), nh(nh), _listener(_buffer) {}

	void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		pose = *msg;
		state->pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
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

	void distanceBetweenPointAndWaypoint(const geometry_msgs::PointStamped& point, const autoware_msgs::Lane& lane, const int& closest_waypoint_id, double& distance, unsigned int& lane_id)
	{
		distance = std::numeric_limits<double>::infinity();
		double lookahead_distance = 0.0;
		try
		{
			for (unsigned int i = closest_waypoint_id; i < lane.waypoints.size(); i++)
			{
				geometry_msgs::PoseStamped waypoint_transformed;
				geometry_msgs::PointStamped point_transformed;
				_tr_base_link = _buffer.lookupTransform("base_link", "map", pose.header.stamp, ros::Duration(10.0));
				_tr_velodyne  = _buffer.lookupTransform("velodyne", "base_link", pose.header.stamp, ros::Duration(10.0));
				tf2::doTransform(lane.waypoints[i].pose, waypoint_transformed, _tr_base_link);
				tf2::doTransform(point, point_transformed, _tr_velodyne);
				double _d_obsx = waypoint_transformed.pose.position.x - point_transformed.point.x;
				double _d_obsy = waypoint_transformed.pose.position.y - point_transformed.point.y;
				double dx = (waypoint_transformed.pose.position.x - point_transformed.point.x);
				double dy = (waypoint_transformed.pose.position.y - point_transformed.point.y);
				double d_dist = sqrt(dx*dx + dy*dy);
				//lookahead_distance += d_dist;
				if ((_d_obsx*_d_obsx < MAX_LOOKAHEAD_DISTANCE_SQR) && (_d_obsy*_d_obsy < MAX_LATERAL_DISTANCE_SQR))
				{
					lane_id = i;
					distance = std::min(distance, sqrt(_d_obsx * _d_obsx - _d_obsy * _d_obsy));
				}
				else if (lookahead_distance > MAX_LOOKAHEAD_DISTANCE)
				{
					break;
				}
			}
		}
		catch (tf2::LookupException& e)
		{
			ROS_ERROR_STREAM("Unable to find transform: " << e.what());
		}
	}

	friend class ObstacleMonitor;
};

class ObstacleMonitor
{
private:

	std::shared_ptr<ros::NodeHandle> nh;
	std::unique_ptr<RobotFirstOrderStateSubscriber> ego_state_subscriber;
	ros::Subscriber sub_detected_object_topic;
	ros::Subscriber sub_closest_waypoint_id;
	ros::Subscriber sub_base_waypoints;
	std::atomic<unsigned int> closest_waypoint_id;
	autoware_msgs::Lane msg_current_base_lane;
	// Publish event stream
	monitoring_msgs::MonitorFeedback event_detected_object;
	ros::Publisher pub_event_detected_object;
	ros::Publisher pub_event_obstacle_lane_id;

	// Autoware lane
public:
	ObstacleMonitor(std::shared_ptr<ros::NodeHandle> nh): nh(nh), ego_state_subscriber(new RobotFirstOrderStateSubscriber(nh)) {}


	void cbClosestLaneId(const std_msgs::Int32::ConstPtr& msg)
	{
		closest_waypoint_id = msg->data;
		ROS_DEBUG_STREAM("Closest waypoint: " << closest_waypoint_id);
	}

	void cbBaseWaypoints(const autoware_msgs::Lane::ConstPtr& msg)
	{
		msg_current_base_lane = *msg;

	}


	void cbDetectedObject(const autoware_msgs::CloudClusterArray::ConstPtr& msg)
	{
		for (const auto& cluster: msg->clusters)
		{
			double distance;
			unsigned int lane_id;
			ego_state_subscriber->distanceBetweenPointAndWaypoint(cluster.centroid_point, msg_current_base_lane, closest_waypoint_id, distance, lane_id);
			ROS_DEBUG_STREAM("Closest object distance " << distance);
			if (distance < MAX_LOOKAHEAD_DISTANCE)
			{
				monitoring_msgs::MonitorFeedback event;
				event.eval = 1;
				event.severity = monitoring_msgs::MonitorFeedback::SEVERITY_WARNING;
				// Using lateral distance
				event.signal_id = "LANE_OBJECT_DETECTED";
				event.val = abs(2.0);
				event.header.stamp = ros::Time::now();
				pub_event_detected_object.publish(event);
			}
		}

	}

	void init()
	{
		sub_detected_object_topic = nh->subscribe("/detection/lidar_detector/cloud_clusters", 10, &ObstacleMonitor::cbDetectedObject, this);
		// Autoware specific
		sub_closest_waypoint_id = nh->subscribe("/closest_waypoint", 10, &ObstacleMonitor::cbClosestLaneId, this);
		sub_base_waypoints = nh->subscribe("/base_waypoints", 10, &ObstacleMonitor::cbBaseWaypoints, this);
		// Event stream
		pub_event_detected_object = nh->advertise<monitoring_msgs::MonitorFeedback>("monitor/events/detected_object", 1000);
		pub_event_obstacle_lane_id = nh->advertise<std_msgs::Int32>("/monitor/events/obstacle_lane_id", 1000);
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
