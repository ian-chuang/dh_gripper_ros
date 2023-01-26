/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a DH gripper
 * code modified from https://github.com/jr-robotics/robotiq/tree/noetic-devel/robotiq_2f_gripper_action_server
 */

#include <dh_gripper_action_server/dh_gripper_action_server.h>

namespace dh_gripper_action_server
{

DHGripperActionServer::DHGripperActionServer(const std::string& name, const DHGripperParams& params)
  : nh_()
  , as_(nh_, name, false)
  , action_name_(name)
  , gripper_params_(params)
{
  as_.registerGoalCallback(boost::bind(&DHGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&DHGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe(gripper_params_.state_topic_, 1, &DHGripperActionServer::stateCB, this);
  goal_pub_ = nh_.advertise<dh_gripper_msgs::GripperCtrl>(gripper_params_.control_topic_, 1);

  as_.start();

  current_state_.is_initialized = false;
}

void DHGripperActionServer::goalCB()
{
  // Check to see if the gripper is in an active state where it can take goals
  if (current_state_.is_initialized == false)
  {
    ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
    return;
  }

  GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  try
  {
    GripperCtrl ctrl_msg = goalToGripperCtrl(current_goal);
    goal_pub_.publish(ctrl_msg);
    current_control = ctrl_msg;
  }
  catch (BadArgumentsError& e)
  {
    ROS_INFO("%s bad goal issued to gripper", action_name_.c_str());
  }
}

void DHGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void DHGripperActionServer::stateCB(const GripperState::ConstPtr& msg)
{
  current_state_ = *msg;

  if (msg->is_initialized == false) {
    issueInitialization();
    return;
  }

  if (!as_.isActive()) return;

  if (msg->position == current_control.position)
  {
    ROS_INFO("%s succeeded (reached target position)", action_name_.c_str());
    GripperCommandResult result;
    result.position = msg->position;
    result.effort = msg->target_force; // not the measured force
    result.stalled = false;
    result.reached_goal = true;
    as_.setSucceeded(result);
  }
  else if (msg->grip_state == OBJECT_CAUGHT) {
    ROS_INFO("%s succeeded (gripped object and stalled)", action_name_.c_str());
    GripperCommandResult result;
    result.position = msg->position;
    result.effort = msg->target_force; // not the measured force
    result.stalled = true;
    result.reached_goal = false;
    as_.setSucceeded(result);
  }
  else
  {
    GripperCommandFeedback feedback;
    feedback.position = msg->position;
    feedback.effort = msg->target_force; // not the measured force
    feedback.stalled = false;
    feedback.reached_goal = false;
    as_.publishFeedback(feedback);
  }
}

GripperCtrl DHGripperActionServer::goalToGripperCtrl(GripperCommandGoal goal) {
  double angle = goal.command.position;
  double max_effort = goal.command.max_effort;

  if (max_effort == 0) {
    max_effort = gripper_params_.default_effort_;
  }

  if (
    angle < gripper_params_.min_angle_ ||
    angle > gripper_params_.max_angle_ ||
    max_effort < gripper_params_.min_effort_ ||
    max_effort > gripper_params_.max_effort_
  ) 
  {
    throw BadArgumentsError();
  }

  double gripper_ctrl_position = 
    MAX_POSITION - 
    (angle - gripper_params_.min_angle_) * 
    (MAX_POSITION - MIN_POSITION) / 
    (gripper_params_.max_angle_ - gripper_params_.min_angle_) 
    + MIN_POSITION;

  GripperCtrl ctrl_msg;
  ctrl_msg.initialize = false;
  ctrl_msg.position = gripper_ctrl_position;
  ctrl_msg.force = max_effort;
  ctrl_msg.speed = gripper_params_.speed_;

  return ctrl_msg;
}

void DHGripperActionServer::issueInitialization()
{
  ROS_INFO("Initializing DH gripper for action server: %s", action_name_.c_str());
  GripperCtrl ctrl_msg;
  ctrl_msg.initialize = true;
  ctrl_msg.position = 0;
  ctrl_msg.force = 0;
  ctrl_msg.speed = 0;
  goal_pub_.publish(ctrl_msg);
}
} // end dh_gripper_action_server namespace
