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
#include <monitoring_msgs/MonitorFeedback.h>
#include <monitoring_msgs/MonitorFeedbackArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <planner_msgs/LocalPlannerObjectDetection.h>

class RobotFirstOrderState
{
public:
	Eigen::Vector3d pos;
	Eigen::Quaterniond ori;
};

const double MAX_LATERAL_DISTANCE = 1.4;
const double MAX_LATERAL_DISTANCE_SQR = MAX_LATERAL_DISTANCE * MAX_LATERAL_DISTANCE;
const double MAX_LOOKAHEAD_DISTANCE = 16.8;
const double MAX_LOOKAHEAD_DISTANCE_SQR = MAX_LOOKAHEAD_DISTANCE * MAX_LOOKAHEAD_DISTANCE;
// Avoidance constants
const double AVOIDANCE_TRAJECTORY_POINT_LENGTH = 37;

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
	geometry_msgs::PointStamped last_detected_object;
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

	void distanceBetweenPointAndWaypoint(const geometry_msgs::PointStamped& point, const autoware_msgs::Lane& lane,
			const int& closest_waypoint_id,
			double& longitudinal_distance,
			double& lateral_distance,
			unsigned int& lane_id)
	{
		lateral_distance = std::numeric_limits<double>::infinity();
		longitudinal_distance = std::numeric_limits<double>::infinity();
		double lookahead_distance = 0.0;
		try
		{
			int target_waypoints;
			if (closest_waypoint_id + AVOIDANCE_TRAJECTORY_POINT_LENGTH > lane.waypoints.size())
			{
				target_waypoints = lane.waypoints.size();
			}
			else
			{
				target_waypoints = closest_waypoint_id + AVOIDANCE_TRAJECTORY_POINT_LENGTH;
			}
			for (unsigned int i = closest_waypoint_id; i < target_waypoints; i++)
			{
				geometry_msgs::PoseStamped waypoint_transformed;
				geometry_msgs::PointStamped point_transformed;
				_tr_base_link = _buffer.lookupTransform("base_link", "map", pose.header.stamp, ros::Duration(10.0));
				_tr_velodyne  = _buffer.lookupTransform("velodyne", "base_link", pose.header.stamp, ros::Duration(10.0));
				tf2::doTransform(lane.waypoints[i].pose, waypoint_transformed, _tr_base_link);
				tf2::doTransform(point, point_transformed, _tr_velodyne);
				double _d_obsx = point_transformed.point.x;
				double _d_obsy = point_transformed.point.y;
				double dx = (waypoint_transformed.pose.position.x - point_transformed.point.x);
				double dy = (waypoint_transformed.pose.position.y - point_transformed.point.y);
				double d_dist = sqrt(dx*dx + dy*dy);
				//lookahead_distance += d_dist;
				if (d_dist < 2.0)
				{
					lateral_distance = waypoint_transformed.pose.position.y - point_transformed.point.y;
					longitudinal_distance = _d_obsx;
					ROS_INFO_STREAM("Obstacle detected near to the waypoint" << lateral_distance << '\t' <<
							longitudinal_distance << '\t' << _d_obsy);
					last_detected_object = point_transformed;
					lane_id = i;
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
	ros::Publisher pub_local_planner_object_detection; 			///< Publish planner object detection
	ros::Publisher  pub_detected_object_marker;
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
			double lateral_distance;
			double longitudinal_distance;
			unsigned int lane_id;
			ego_state_subscriber->distanceBetweenPointAndWaypoint(cluster.centroid_point,
					msg_current_base_lane, closest_waypoint_id,
					longitudinal_distance,
					lateral_distance,
					lane_id);
			if (longitudinal_distance < MAX_LOOKAHEAD_DISTANCE && lateral_distance < MAX_LATERAL_DISTANCE)
			{
				auto event_time = ros::Time::now();
				monitoring_msgs::MonitorFeedback event;
				// Publish local planner object detection
				planner_msgs::LocalPlannerObjectDetection localplanner_msg;
				localplanner_msg.avoidance_start_index = closest_waypoint_id;
				localplanner_msg.avoidance_end_index   = closest_waypoint_id + AVOIDANCE_TRAJECTORY_POINT_LENGTH;
				localplanner_msg.header.stamp = event_time;
				localplanner_msg.event.signal_id = "LANE_OBJECT_DETECTED";
				localplanner_msg.event.header.stamp = event_time;
				localplanner_msg.event.val = abs(2.0);
				localplanner_msg.event.severity = monitoring_msgs::MonitorFeedback::SEVERITY_WARNING;
				localplanner_msg.event.eval = 1;
				pub_local_planner_object_detection.publish(localplanner_msg);
				visualization_msgs::Marker object_marker;
				object_marker.header.frame_id = "base_link";
				object_marker.header.stamp = ros::Time::now();


				object_marker.color.a = 1.0;
				object_marker.color.r = 1.0;
				object_marker.color.g = 0.0;
				object_marker.color.b = 0.0;
				object_marker.type = visualization_msgs::Marker::SPHERE;
				object_marker.scale.x = 1.0;
				object_marker.scale.y = 1.0;
				object_marker.scale.z = 1.0;
				object_marker.pose.position.x = ego_state_subscriber->last_detected_object.point.x;
				object_marker.pose.position.y = ego_state_subscriber->last_detected_object.point.y;
				object_marker.pose.position.z = ego_state_subscriber->last_detected_object.point.z;
				pub_detected_object_marker.publish(object_marker);
				ROS_INFO_STREAM("Obstacle detected at: " << lateral_distance << '\t' << longitudinal_distance << '\n');

			}
		}

	}

	void init()
	{
		// Event stream
		pub_local_planner_object_detection =
				nh->advertise<planner_msgs::LocalPlannerObjectDetection>("/monitor/events/local_planner/object_detection", 1000);
		pub_detected_object_marker =
				nh->advertise<visualization_msgs::Marker>("monitor/events/local_planner/detected_object/marker", 10);

		sub_detected_object_topic = nh->subscribe("/detection/lidar_detector/cloud_clusters", 10, &ObstacleMonitor::cbDetectedObject, this);
		// Autoware specific
		sub_closest_waypoint_id = nh->subscribe("/closest_waypoint", 10, &ObstacleMonitor::cbClosestLaneId, this);
		sub_base_waypoints = nh->subscribe("/base_waypoints", 10, &ObstacleMonitor::cbBaseWaypoints, this);
	}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "ros_object_detection_monitor");
	std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
	ObstacleMonitor monitor(nh);
	monitor.init();
	ROS_INFO("Node initialized: object detection monitor");
	ros::spin();
	return 0;
}
