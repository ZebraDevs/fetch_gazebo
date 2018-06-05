/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2015, Fetch Robotics Inc.
 *  Copyright (c) 2013-2014, Unbounded Robotics Inc.
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
 *   * Neither the name of Fetch Robotics nor the names of its
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

// Author: Michael Ferguson

#ifndef FETCH_GAZEBO_JOINT_HANDLE_H
#define FETCH_GAZEBO_JOINT_HANDLE_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <control_toolbox/pid.h>
#include <gazebo/physics/physics.hh>
#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller_manager.h>

namespace gazebo
{

class JointHandle : public robot_controllers::JointHandle
{
  enum CommandState
  {
    MODE_DISABLED,
    MODE_CONTROL_EFFORT,
    MODE_CONTROL_VELOCITY,
    MODE_CONTROL_POSITION
  };

public:
  JointHandle(physics::JointPtr& joint,
              const double velocity_limit,
              const double effort_limit,
              const bool continuous) :
    joint_(joint),
    actual_velocity_(0.0),
    mode_(MODE_DISABLED)
  {
    ros::NodeHandle nh("~");

    // Load controller parameters
    position_pid_.init(ros::NodeHandle(nh, getName() + "/position"));
    velocity_pid_.init(ros::NodeHandle(nh, getName() + "/velocity"));

    // Set effort limit and continuous state
    velocity_limit_ = velocity_limit;
    effort_limit_ = effort_limit;
    continuous_ = continuous;
  }
  virtual ~JointHandle()
  {
  }

  /**
   * @brief Set a position command for this joint.
   * @param position Position command in radians or meters.
   * @param velocity Velocity command in rad/sec or meters/sec.
   * @param effort Effort command in Nm or N.
   */
  virtual void setPosition(double position, double velocity, double effort)
  {
    // ControllerManager clears these all each cycle, so just
    // set mode and accumulate outputs of each controller.
    desired_position_ += position;
    desired_velocity_ += velocity;
    desired_effort_ += effort;
    mode_ = MODE_CONTROL_POSITION;
  }

  /**
   * @brief Set a velocity command for this joint.
   * @param velocity Velocity command in rad/sec or meters/sec.
   * @param effort Effort command in Nm or N.
   */
  virtual void setVelocity(double velocity, double effort)
  {
    desired_velocity_ += velocity;
    desired_effort_ += effort;
    if (mode_ != MODE_CONTROL_POSITION)
      mode_ = MODE_CONTROL_VELOCITY;
  }

  /**
   * @brief Set an effort command for this joint.
   * @param effort Effort command in Nm or N.
   */
  virtual void setEffort(double effort)
  {
    desired_effort_ += effort;
    if (mode_ != MODE_CONTROL_POSITION &&
        mode_ != MODE_CONTROL_VELOCITY)
      mode_ = MODE_CONTROL_EFFORT;
  }

  /** @brief Get the position of the joint in radians or meters. */
  virtual double getPosition()
  {
    if (continuous_)
    {
      return angles::normalize_angle(joint_->Position(0));
    }
    return joint_->Position(0);
  }

  /** @brief Get the velocity of the joint in rad/sec or meters/sec. */
  virtual double getVelocity()
  {
    return actual_velocity_;
  }

  /** @brief Get applied effort of a joint in Nm or N. */
  virtual double getEffort()
  {
    return applied_effort_;
  }

  /* Is this joint continuous (has no position limits). */
  virtual bool isContinuous()
  {
    return continuous_;
  }

  /** @brief Get the minimum valid position command. */
  virtual double getPositionMin()
  {
    return joint_->LowerLimit(0);
  }

  /** @brief Get the maximum valid position command. */
  virtual double getPositionMax()
  {
    return joint_->UpperLimit(0);
  }

  /** @brief Get the maximum velocity command. */
  virtual double getVelocityMax()
  {
    if (velocity_limit_ < 0.0)
      return joint_->GetVelocityLimit(0);
    else
      return velocity_limit_;
  }

  /** @brief Get the maximum effort command. */
  virtual double getEffortMax()
  {
    /*
     * gzsdf has a major flaw when using continuous joints. It appears gazebo
     * cannot handle continuous joints and so it sets the limits to +/-1e16.
     * This is fine, except it drops the limit effort and limit velocity.
     * Lack of limit effort causes the robot to implode to the origin if the
     * controllers are not tuned or experience a disturbance. This little hack
     * lets us limit the controller effort internally.
     */
    if (effort_limit_ < 0.0)
      return joint_->GetEffortLimit(0);
    else
      return effort_limit_;
  }

  /** @brief Get the name of this joint. */
  virtual std::string getName()
  {
    return joint_->GetName();
  } 

  /** @brief Reset the command. */
  virtual void reset()
  {
    desired_position_ = 0.0;
    desired_velocity_ = 0.0;
    desired_effort_ = 0.0;
    mode_ = MODE_DISABLED;
  }

  /** @brief Actually apply updates to gazebo */
  void update(const ros::Time now, const ros::Duration dt)
  {
    actual_velocity_ += 0.1 * (joint_->GetVelocity(0) - actual_velocity_);

    double effort = 0.0;
    if (mode_ == MODE_CONTROL_POSITION)
    {
      double p_error = angles::shortest_angular_distance(getPosition(), desired_position_);
      double v = position_pid_.computeCommand(p_error, dt) + desired_velocity_;
      v = std::min(getVelocityMax(), std::max(-getVelocityMax(), v));
      double t = velocity_pid_.computeCommand(v - actual_velocity_, dt);
      effort = t + desired_effort_;
    }
    else if (mode_ == MODE_CONTROL_VELOCITY)
    {
      double t = velocity_pid_.computeCommand(desired_velocity_ - actual_velocity_, dt);
      effort = t + desired_effort_;
    }
    else if (mode_ == MODE_CONTROL_EFFORT)
    {
      effort = desired_effort_;
    }

    // Limit effort so robot doesn't implode
    double lim = getEffortMax();
    applied_effort_ = std::max(-lim, std::min(effort, lim));

    // Actually update
    joint_->SetForce(0, applied_effort_);
  }

private:
  physics::JointPtr joint_;

  double desired_position_;
  double desired_velocity_;
  double desired_effort_;
  
  /// control mode
  int mode_;

  control_toolbox::Pid position_pid_;
  control_toolbox::Pid velocity_pid_;

  /// Hack for continuous joints that fail to have velocity limits
  double velocity_limit_;

  /// Hack for continuous joints that fail to have effort limits
  double effort_limit_;

  // Is this joint continuous?
  bool continuous_;

  /// GetForce(0u) is not always right
  double applied_effort_;
  double actual_velocity_;

  // You no copy...
  JointHandle(const JointHandle&);
  JointHandle& operator=(const JointHandle&);
};

typedef boost::shared_ptr<JointHandle> JointHandlePtr;

}  // namespace gazebo

#endif  // FETCH_GAZEBO_JOINT_HANDLE_H
