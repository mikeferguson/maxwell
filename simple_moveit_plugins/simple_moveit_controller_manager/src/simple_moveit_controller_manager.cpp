/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  Copyright (c) 2013, Michael E. Ferguson
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, E. Gil Jones, Michael Ferguson */
/* This is the PR2-controller manager, made more generic, aimed at Maxwell */

#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <map>

namespace simple_moveit_controller_manager
{

static const double DEFAULT_MAX_GRIPPER_EFFORT = 10000.0;
static const double GRIPPER_OPEN = 0.086;
static const double GRIPPER_CLOSED = 0.0;

template<typename T>
class ActionBasedControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ActionBasedControllerHandle(const std::string &name, const std::string &ns) :
    moveit_controller_manager::MoveItControllerHandle(name),
    namespace_(ns),
    done_(true)
  {
    controller_action_client_.reset(new actionlib::SimpleActionClient<T>(name_ +"/" + namespace_, true));
    unsigned int attempts = 0;
    while (ros::ok() && !controller_action_client_->waitForServer(ros::Duration(5.0)) && ++attempts < 3)
      ROS_INFO_STREAM("Waiting for " << name_ + "/" + namespace_ << " to come up");

    if (!controller_action_client_->isServerConnected())
    {
      ROS_ERROR_STREAM("Action client not connected: " << name_ + "/" + namespace_);
      controller_action_client_.reset();
    }
    
    last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  bool isConnected() const
  {
    return controller_action_client_;
  }

  virtual bool cancelExecution() 
  {   
    if (!controller_action_client_)
      return false;
    if (!done_)
    {
      ROS_INFO_STREAM("Cancelling execution for " << name_);
      controller_action_client_->cancelGoal();
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      done_ = true;
    }
    return true;
  }
  
  virtual bool waitForExecution(const ros::Duration &timeout = ros::Duration(0))
  { 
    if (controller_action_client_ && !done_)
      return controller_action_client_->waitForResult(timeout);
    return true;
  }

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
  {
    return last_exec_;
  }

protected:

  void finishControllerExecution(const actionlib::SimpleClientGoalState& state)
  {
    ROS_DEBUG_STREAM("Controller " << name_ << " is done with state " << state.toString() << ": " << state.getText());
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else
      if (state == actionlib::SimpleClientGoalState::ABORTED)
        last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
      else
        if (state == actionlib::SimpleClientGoalState::PREEMPTED)
          last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
        else
          last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    done_ = true;
  }

  moveit_controller_manager::ExecutionStatus last_exec_;
  std::string namespace_;
  bool done_;
  boost::shared_ptr<actionlib::SimpleActionClient<T> > controller_action_client_;
};

class GripperControllerHandle : public ActionBasedControllerHandle<control_msgs::GripperCommandAction>
{
public:
  GripperControllerHandle(const std::string &name, const std::string &ns  = "gripper_action") :
    ActionBasedControllerHandle<control_msgs::GripperCommandAction>(name, ns),
    closing_(false)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_INFO_STREAM("new trajectory to " << name_);
    if (!controller_action_client_)
      return false;
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("The simple gripper controller cannot execute multi-dof trajectories.");
      return false;
    }
    
    if (trajectory.joint_trajectory.points.size() != 1)
    {
      ROS_ERROR("The simple gripper controller expects a joint trajectory with one point only, but %u provided)", (unsigned int)trajectory.joint_trajectory.points.size());
      return false;
    }

    if (trajectory.joint_trajectory.points[0].positions.empty())
    {
      ROS_ERROR("The simple gripper controller expects a joint trajectory with one point that specifies at least one position, but 0 positions provided)");
      return false;
    }
    
    control_msgs::GripperCommandGoal goal;
    goal.command.max_effort = DEFAULT_MAX_GRIPPER_EFFORT;
    if (!trajectory.joint_trajectory.points[0].velocities.empty())
      goal.command.max_effort = trajectory.joint_trajectory.points[0].velocities[0];
    
    goal.command.position = trajectory.joint_trajectory.points[0].positions[0];
    /*if (trajectory.joint_trajectory.points[0].positions[0] > 0.5)
    {
      goal.command.position = GRIPPER_OPEN;
      closing_ = false;
      ROS_DEBUG_STREAM("Sending gripper open command");
    }
    else
    {
      goal.command.position = GRIPPER_CLOSED;
      closing_ = true;
      ROS_DEBUG_STREAM("Sending gripper close command");
    }*/

    controller_action_client_->sendGoal(goal,
					boost::bind(&GripperControllerHandle::controllerDoneCallback, this, _1, _2),
					boost::bind(&GripperControllerHandle::controllerActiveCallback, this),
					boost::bind(&GripperControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }
  
private:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::GripperCommandResultConstPtr& result)
  {
    // the gripper action reports failure when closing the gripper and an object is inside
    //if (state == actionlib::SimpleClientGoalState::ABORTED && closing_)
      finishControllerExecution(actionlib::SimpleClientGoalState::SUCCEEDED);
    //else
      //finishControllerExecution(state);
  }
  
  void controllerActiveCallback() 
  {
    ROS_DEBUG_STREAM("Controller " << name_ << " started execution");
  }
  
  void controllerFeedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr& feedback)
  {
  }
  
  bool closing_;
};

class SimpleFollowJointTrajectoryControllerHandle : public ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>
{
public:
  
  SimpleFollowJointTrajectoryControllerHandle(const std::string &name, const std::string &ns = "follow_joint_trajectory") :
    ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>(name, ns)
  {  
  }
  
  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_INFO_STREAM("new trajectory to " << name_);
    if (!controller_action_client_)
      return false;
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("The PR2 FollowJointTrajectory controller cannot execute multi-dof trajectories.");
      return false;
    }
    if (done_)
      ROS_DEBUG_STREAM("Sending trajectory to FollowJointTrajectory action for controller " << name_);
    else
      ROS_DEBUG_STREAM("Sending continuation for the currently executed trajectory to FollowJointTrajectory action for controller " << name_);
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory.joint_trajectory;
    controller_action_client_->sendGoal(goal,
					boost::bind(&SimpleFollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
					boost::bind(&SimpleFollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
					boost::bind(&SimpleFollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }
  
protected:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::FollowJointTrajectoryResultConstPtr& result)
  {
    finishControllerExecution(state);
  }
  
  void controllerActiveCallback() 
  {
    ROS_DEBUG_STREAM("Controller " << name_ << " started execution");
  }
  
  void controllerFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
  {
  }
};

class SimpleMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  
  SimpleMoveItControllerManager() : node_handle_("~")
  { 
    /* need to initialize controllers_ from the server, as we have no standard
       way of reading controller names */
    XmlRpc::XmlRpcValue controller_list;
    if (node_handle_.hasParam("controller_list"))
    {
      node_handle_.getParam("controller_list", controller_list);
      if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        ROS_WARN("Controller list should be specified as an array");
      else
        for (int i = 0 ; i < controller_list.size() ; ++i)
          if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
            ROS_WARN("Name and joints must be specifed for each controller");
          else
          {
            try
            {
              std::string name = std::string(controller_list[i]["name"]);
              std::string ns;
              if (controller_list[i].hasMember("ns"))
                ns = std::string(controller_list[i]["ns"]);
              if (controller_list[i]["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray)
              {
                int nj = controller_list[i]["joints"].size();
                for (int j = 0 ; j < nj ; ++j)
                  controller_joints_[name].push_back(std::string(controller_list[i]["joints"][j]));
                
                moveit_controller_manager::MoveItControllerHandlePtr new_handle;
                if ( name == "gripper_controller" )
                {
                  new_handle.reset(ns.empty() ? new GripperControllerHandle(name) : new GripperControllerHandle(name, ns));
                  if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
                  {
                    ROS_INFO_STREAM("Added controller for " << name );
	                controller_handles_[name] = new_handle;
                  }
                }
                else
                {
                  new_handle.reset(ns.empty() ? new SimpleFollowJointTrajectoryControllerHandle(name) : new SimpleFollowJointTrajectoryControllerHandle(name, ns));
                  if (static_cast<SimpleFollowJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
                  { 
                    ROS_INFO_STREAM("Added controller for " << name );
	                controller_handles_[name] = new_handle;
                  }
                }
              }
              else
                ROS_WARN_STREAM("The list of joints for controller " << name << " is not specified as an array");
            }
            catch (...)
            {
              ROS_ERROR("Unable to parse controller information");
            }
          }
    }
    else
    {
	  ROS_ERROR_STREAM("No controllers specified.");
    }
  }
  
  virtual ~SimpleMoveItControllerManager()
  {
  }
  
  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = controller_handles_.find(name);
    if (it != controller_handles_.end())
      return it->second;
    else
      ROS_FATAL_STREAM("No such controller: " << name);
  }
  
  virtual void getControllersList(std::vector<std::string> &names)
  {    
    for (std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = controller_handles_.begin() ; it != controller_handles_.end() ; ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM("Returned " << names.size() << " controllers in list");
  }
  
  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }
  
  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }
  
  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    std::map<std::string, std::vector<std::string> >::const_iterator it = controller_joints_.find(name);
    if (it != controller_joints_.end())
    {
      joints = it->second;
      ROS_INFO_STREAM("Returned " << joints.size() << " joints for " << name);
    }
    else
    {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on the param server?", name.c_str());
      joints.clear();
    }
  }
  
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    /* Controllers need to be loaded and active to be used. */
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.loaded_ = true;
    state.active_ = true;
    state.default_ = true;
    return state;
  }
  
  virtual bool loadController(const std::string &name)
  {
    /* All of our controllers are already loaded. */
    return true;
  }
  
  virtual bool unloadController(const std::string &name)
  {
    /* Cannot unload our controllers */
    return false;
  }
  
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate)
  {
    /* Cannot switch our controllers */
    return false;
  }
  
protected:
  
  ros::NodeHandle node_handle_;
  ros::NodeHandle root_node_handle_;
  std::map<std::string, std::vector<std::string> > controller_joints_;
  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> controller_handles_;
};

}

PLUGINLIB_EXPORT_CLASS(simple_moveit_controller_manager::SimpleMoveItControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
