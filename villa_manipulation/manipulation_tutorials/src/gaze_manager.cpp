#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <hsr_kinematics/head_kinematics.hpp>
#include <tmc_eigen_bridge/eigen_bridge.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_utils/pose_utils.hpp>

#include <boost/thread.hpp>
#include <boost/format.hpp>

namespace {
  const double kRate = 10.0;
  const char* kOriginLink = "map";
  const char* kRobotBaseLink = "base_link";
}

class gaze_manager {
 public:
  gaze_manager() :
    is_activating_(false),
    tf_buffer_(),
    listener_(tf_buffer_) {
    ros::NodeHandle nh("gaze_manager");

    joint_state_sub_ = nh.subscribe("/hsrb/joint_states", 1,
                                    &gaze_manager::JointStateCallback_, this);
  }


  void Fixing() {
    if (!is_activating_) {
      return;
    }
    //geometry_msgs::TransformStamped transform_stamped;
    //try {
    //transform_stamped =
      //tf_buffer_.lookupTransform(kOriginLink,
                                 //kRobotBaseLink,
                                 //ros::Time(0),
                                 //ros::Duration(1.0));
    //} catch(const tf2::TransformException& ex) {
      //ROS_ERROR_STREAM((boost::format("Cannot transform goal pose from '%1%' frame to '%2%' frame.")
                        //% kOriginLink % kRobotBaseLink).str());
      //return;
    //}
    //sensor_msgs::JointState target_head_angle;
    //geometry_msgs::Pose origin_to_robot;
    //origin_to_robot.position.x = transform_stamped.transform.translation.x;
    //origin_to_robot.position.y = transform_stamped.transform.translation.y;
    //origin_to_robot.orientation = transform_stamped.transform.rotation;
    //if (!ComputeGazeTargetPointHeadAngle_(origin_to_robot,
                                          //origin_to_target_point_,
                                          //&target_head_angle)) {
      //ROS_ERROR("Failed to compute head angle to gaze target.");
      //return;
    //}
    //trajectory_msgs::JointTrajectory traj;
    //traj.joint_names.push_back("head_pan_joint");
    //traj.joint_names.push_back("head_tilt_joint");
    //trajectory_msgs::JointTrajectoryPoint point;
    //point.positions.push_back(target_head_angle.position[0]);
    //point.positions.push_back(target_head_angle.position[1]);
    //point.time_from_start = ros::Duration(0.01);
    //traj.points.push_back(point);
    //head_trajectory_pub_.publish(traj);
  }

 private:
  ros::Publisher head_trajectory_pub_;
  ros::Subscriber activation_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber target_point_sub_;
  tf2_ros::Buffer tf_buffer_;

  mutable boost::mutex mutex_;
  bool is_activating_;
  tf2_ros::TransformListener listener_;
  sensor_msgs::JointState current_joint_state_;


  void JointStateCallback_(const sensor_msgs::JointState& msg) {
    boost::mutex::scoped_lock lock(mutex_);
    current_joint_state_ = msg;
  }


};

void Run() {
  gaze_manager node;
  ros::Rate rate(kRate);
  while (ros::ok()) {
    node.Fixing();
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazed_point_fixing_node");
  Run();
  return EXIT_SUCCESS;
}
